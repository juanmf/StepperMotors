import threading
import time
import traceback
from concurrent.futures import Future
from multiprocess import Value

from RPi import GPIO
from stepper_motors_juanmf1.UnipolarController import UnipolarMotorDriver

from stepper_motors_juanmf1.myMath import cmp
from stepper_motors_juanmf1.BlockingQueueWorker import BlockingQueueWorker
from stepper_motors_juanmf1.Controller import MotorDriver, ThirdPartyAdapter
from stepper_motors_juanmf1.EventDispatcher import EventDispatcher
from stepper_motors_juanmf1.SortedDict import SortedDict
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint

from stepper_motors_juanmf1.ThreadOrderedPrint import flush_current_thread_only


class Navigation:
    """
    Navigation strategies is where PPS is actually senf to the motor drivers, by making use of their configured
    Acceleration strategies, it determines "sleep/wait" time.
    Several components can be a bottleneck while pulsing a driver:
    * Your RPi. Mine (RPi 4B) is currently handling 14.2K cycles/s (at 60 RPM & 200 SPR yields max 71 microsteps before
            slowing down the motor). This is due to time consumed computing next sleep times during pulse duration.
    * The motor max steps per second
    """
    _COMPLETED_FUTURE = None  # Class variable

    def __init__(self):
        Navigation.get_completed_future()
        self.driverPulseTimeNs = None

    def setDriverPulseTimeNs(self, *, driverPulseTimeUs):
        self.driverPulseTimeNs = 1000 * driverPulseTimeUs

    @staticmethod
    def get_completed_future():
        # Lazily initialize the Future if not already done
        if Navigation._COMPLETED_FUTURE is None:
            Navigation._COMPLETED_FUTURE = Future()
            Navigation._COMPLETED_FUTURE.set_result(True)
        return Navigation._COMPLETED_FUTURE

    def go(self, controller, targetPosition, accelerationStrategy, fn, interruptPredicate, eventInAdvanceSteps=10,
           eventName="steppingComplete"):
        return self.get_completed_future()

    @staticmethod
    def pulseController(controller: MotorDriver, stepRelativeToJobStart=None):
        controller.pulseStart(stepRelativeToJobStart)
        if controller.PULSE_TIME_MICROS:
            controller.usleep(controller.PULSE_TIME_MICROS)
            controller.pulseStop()

    @staticmethod
    def isInterruptible():
        pass

    @staticmethod
    def waitNextCycle(*, pulseStartNs, pulseDurationUs):
        # Using pulse time to catch up with user logic and sleepTime computation.
        pulseElapsedUs = (time.monotonic_ns() - pulseStartNs) // 1000
        remainingSleepUs = pulseDurationUs - pulseElapsedUs
        if remainingSleepUs > 10:
            # Arbitrarily just continue if remaining sleep time < 10us
            MotorDriver.usleep(remainingSleepUs)
        else:
            tprint(f"Warning: Can't process user fn, events and sleepTime calculations within pulse duration. "
                   f"remainingSleepUs {remainingSleepUs}")


class StaticNavigation(Navigation):
    def go(self, controller: MotorDriver, targetPosition, accelerationStrategy, fn, interruptPredicate,
           eventInAdvanceSteps=10, eventName="steppingComplete"):
        initialPosition = controller.getCurrentPosition()
        if initialPosition == targetPosition:
            return
        # Todo: find out  if -1 works as LOW (normally set to 0) for direction pin.
        signedDirection = cmp(targetPosition, controller.getCurrentPosition())
        controller.setDirection(signedDirection)
        accelerationStrategy.realDirection = signedDirection
        step = pulseCount = 0
        totalSteps = abs(initialPosition - targetPosition)
        for position in range(initialPosition, targetPosition, signedDirection):
            pulseStart = time.monotonic_ns()
            self.pulseController(controller, pulseCount)
            # Todo: see to make this work with actual position, StaticDelayPlanner assumes steps from 0 to totalSteps.
            step += 1
            pulseCount += accelerationStrategy.realDirection
            accelerationStrategy.computeSleepTimeUs(step, totalSteps)
            controller.setCurrentPosition(position)

            # fn should not consume many CPU instructions to avoid delays between steps.
            if fn:
                fn(controller.getCurrentPosition(), targetPosition, accelerationStrategy.realDirection,
                   controller.multiprocessObserver)

            if abs(targetPosition - position) == eventInAdvanceSteps:
                EventDispatcher.instance().publishMainLoop(eventName + "Advance", {'position': position})

            # Using pulse time to catch up with user logic and sleepTime computation.
            Navigation.waitNextCycle(pulseStartNs=pulseStart, pulseDurationUs=accelerationStrategy.currentSleepTimeUs)

        accelerationStrategy.done()
        controller.setDirection(GPIO.LOW)
        return Navigation.get_completed_future()

    @staticmethod
    def isInterruptible():
        return False


class DynamicNavigation(Navigation):

    def go(self, controller: MotorDriver, targetPosition, accelerationStrategy, fn, interruptPredicate,
           eventInAdvanceSteps=10, eventName="steppingComplete"):

        # Can cross targetPosition with some speed > 0, that's not a final state.
        pulseCount = 0
        while not (controller.getCurrentPosition() == targetPosition and accelerationStrategy.canStop()):
            if interruptPredicate():
                return Navigation.get_completed_future()

            pulseStart = time.monotonic_ns()
            self.pulseController(controller, pulseCount)

            # Direction is set in position based acceleration' state machine.
            # Todo: computeSleepTimeUs returning position is obscure.
            position = accelerationStrategy.computeSleepTimeUs(controller.getCurrentPosition(),
                                                        targetPosition,
                                                        lambda d: controller.setDirection(d))

            pulseCount += accelerationStrategy.realDirection
            controller.setCurrentPosition(position)
            # fn should not consume many CPU instructions to avoid delays between steps.
            if fn:
                fn(controller.getCurrentPosition(), targetPosition, accelerationStrategy.realDirection,
                   controller.multiprocessObserver)

            if (abs(targetPosition - position) == eventInAdvanceSteps
                    and cmp(targetPosition, position) == controller.accelerationStrategy.realDirection):
                EventDispatcher.instance().publishMainLoop(eventName + "Advance", {'position': position})

            Navigation.waitNextCycle(pulseStartNs=pulseStart, pulseDurationUs=accelerationStrategy.getCurrentSleepUs())


        # todo: check if still needed.
        accelerationStrategy.done()
        # Todo: GPIO.LOW is not controller specific. fix with controller.setDirection(controller.defaultDirection)
        #  or similar
        controller.setDirection(GPIO.LOW)
        return Navigation.get_completed_future()

    @staticmethod
    def isInterruptible():
        return True


class BasicSynchronizedNavigation(Navigation, BlockingQueueWorker):
    """
    In applications where several motors need to work together in close coordination, like 3D printing, we can't
    just have a set of independent drivers timing their pulses to their convenience. This Navigation ensures steps are
    locked into a rithm.
    """
    _instances = {}

    @staticmethod
    def getInstance(high=GPIO.HIGH, low=GPIO.LOW, countDownLatch=None, newMultitonKey=0):
        if newMultitonKey not in BasicSynchronizedNavigation._instances:

            instance = BasicSynchronizedNavigation(high, low, countDownLatch, newMultitonKey)
            BasicSynchronizedNavigation._instances[newMultitonKey] = instance
        return BasicSynchronizedNavigation._instances[newMultitonKey]

    class CountDownLatch:
        """
        Mimics multiprocessing Value.value interface to use this container instead of Value in non-multiprocess scenario
        """
        def __init__(self):
            self.value = 0

    def __init__(self, high=GPIO.HIGH, low=GPIO.LOW, countDownLatch=None, newMultitonKey=0):
        assert newMultitonKey not in BasicSynchronizedNavigation._instances
        print(f"New instance of {type(self)}")
        BlockingQueueWorker.__init__(self, self.__doGo, jobQueueMaxSize=4, workerName="SynchronizedNavigation")
        Navigation.__init__(self)
        # {startTimNs: [(controller, sleepTime), ...]}.
        self.pulsingControllers = SortedDict()
        self.high = high
        self.low = low
        self.lock = threading.Lock()
        self.upPins = []
        self.eventDispatcher = EventDispatcher.instance()
        self.countDownLatch = countDownLatch
        self.newMultitonKey = newMultitonKey

    def getCountDownLatch(self, default: CountDownLatch or Value = None):
        if self.countDownLatch is None:
            self.countDownLatch = default if default else BasicSynchronizedNavigation.CountDownLatch()
        return self.countDownLatch

    def setCountDown(self, value):
        """
        Will initialize self.countDownLatch assuming non-multiprocess scenario.
        @param value:
        @return:
        """
        if self.countDownLatch is None:
            self.countDownLatch = BasicSynchronizedNavigation.CountDownLatch()
        elif self.countDownLatch.value != 0:
            raise RuntimeError("Cant have more than one latch active at a time. Still pending "
                               f"{self.countDownLatch.value} motor jobs sent to start.")

        self.countDownLatch.value = value

    # Todo: this will be called from multiple threads, one per driver. Handle synchronization accordingly.
    # Waits to get all Synchronized controllers to go then coordinates their steps so that all sleep and step at once.
    def go(self, controller: MotorDriver, targetPosition, accelerationStrategy, fn, interruptPredicate,
           eventInAdvanceSteps=10, eventName="steppingComplete"):
        # Contiguous method calls in RPi are about 10uS apart. for a collision here it'd need to be 10K times faster.
        # this job is navigation level. there is a job per driver we need to complete independently.
        navJob = self.work([self.PulsingController(
                controller, controller.stepperMotor.getMinSleepTime(), targetPosition, fn, interruptPredicate,
                eventInAdvanceSteps=eventInAdvanceSteps, eventName=eventName, high=self.high)])
        return controller.currentJob.block

    def latchDown(self, pulsingController):
        self.putPulsingController(time.monotonic_ns(), pulsingController)
        if self.countDownLatch and self.countDownLatch.value != 0:
            while self.countDownLatch.value > 1:
                # Headsup, If there is a previous independent job running and then a few motors need to start together,
                # this extra looping might delay next pulse of running one.
                self.countDownLatch.value -= 1
                return True
            self.countDownLatch.value = 0

        return False

    def __doGo(self, pulsingController):
        if self.latchDown(pulsingController):
            return
        delegatedDuePulses = []  # When no available Pins send pulse instruction to Driver.
        duePulses = []
        duePulsesStates = []
        bipolarStepPins = []
        dueControllers = []
        try:
            while self.pulsingControllers:
                if self.hasQueuedJobs():
                    # Say driver_1 just put next pulse in pulsingControllers. and new job is from driver_1,
                    #  what next?
                    # Can't be, a running Driver because it's blocked, removed from pulsingControllers to check
                    # interrupted, then let go to pick up it's next job.
                    return
                pulseTime = time.monotonic_ns()
                while self.pulsingControllers:
                    duePulseTime, pulsingController = self.pulsingControllers.popitem()
                    if duePulseTime > pulseTime:
                        self.putPulsingController(duePulseTime, pulsingController)
                        break
                    if self.checkDone(pulsingController):
                        continue
                    pulsingController.addDueControllerFn(duePulses,
                                                         duePulsesStates,
                                                         dueControllers,
                                                         bipolarStepPins,
                                                         delegatedDuePulses)

                if duePulses or delegatedDuePulses:
                    GPIO.output(duePulses, duePulsesStates)
                    for thirdPartyAdapter in delegatedDuePulses:
                        thirdPartyAdapter.pulseStart()
                    self.updateSleepTimes(dueControllers, pulseTime, count=2)
                    while pulseTime + self.driverPulseTimeNs > time.monotonic_ns():
                        # Small active wait in case updateSleepTimes() didn't consume Driver's min pulse time.
                        pass
                    GPIO.output(bipolarStepPins, self.low)
                    for thirdPartyAdapter in delegatedDuePulses:
                        thirdPartyAdapter.pulseStop()
                    # Finish with remaining controllers after sending LOW
                    if dueControllers:
                        self.updateSleepTimes(dueControllers, pulseTime)
                dueControllers.clear()
                delegatedDuePulses.clear()
                duePulses.clear()
                duePulsesStates.clear()

        except Exception as e:
            tprint(f"Synchronized Navigation Failed: {e}")
            tprint(traceback.format_exc())
            flush_current_thread_only()

    def putPulsingController(self, nextPulse, pulsingController):
        """
        Adds next pulse, ensuring no collisions.
        For some reason RPi will have collisions from time.monotonic_ns()
        @return:
        """
        while nextPulse in self.pulsingControllers:
            nextPulse -= 1
        self.pulsingControllers[nextPulse] = pulsingController

    def updateSleepTimes(self, pulsingControllers: list['BasicSynchronizedNavigation.PulsingController'],
                         pulseTimeNs, count=-1):
        """
        Asks each pulsing controller's accelerationStrategy to recalculate pulse time.
        Fires pulsingController.eventName event when close to finish line.
        @param pulsingControllers: list of pulsingControllers that had just sent a pulse to their hardware drivers.
        @param pulseTimeNs: time at which the pulse start took place.
        @param count: number of controllers to deal with. Could be useful to do something else (like sending a LOW to
                      the drivers) before using the rest of the cycle to finish sleep computations. Send -1 for
                      unlimited. Defaults to 2, which in RPi 4B stretches a 15Us-15Us without events &
                      60Us-75Us with multiprocess events delay before giving back control.
        """
        while pulsingControllers and count != 0:
            count -= 1
            pulsingController = pulsingControllers.pop()
            pulsingController.controller.setCurrentPosition(
                (pulsingController.controller.accelerationStrategy.computeSleepTimeUs(
                    pulsingController.controller.getCurrentPosition(), pulsingController.targetPosition,
                    lambda d: pulsingController.controller.setDirection(d))))
            nextPulse = pulseTimeNs + 1000 * pulsingController.controller.accelerationStrategy.getCurrentSleepUs()
            self.putPulsingController(nextPulse, pulsingController)

            if pulsingController.fn is not None:
                pulsingController.fn(pulsingController.controller.getCurrentPosition(),
                                     pulsingController.targetPosition,
                                     pulsingController.controller.accelerationStrategy.realDirection,
                                     pulsingController.controller.multiprocessObserver)

            if pulsingController.eventInAdvanceSteps is not None:
                # Called while Stepper is in flight to finish this step.
                position = pulsingController.controller.getCurrentPosition()
                if (abs(pulsingController.targetPosition - position) == pulsingController.eventInAdvanceSteps
                    and cmp(pulsingController.targetPosition, position)
                        == pulsingController.controller.accelerationStrategy.realDirection):
                    # Firing event
                    self.eventDispatcher.publishMainLoop(pulsingController.eventName + "Advance",
                                                         {"position": position})

    def checkDone(self, pulsingController):
        controllerIsDone = False
        if (pulsingController.controller.getCurrentPosition() == pulsingController.targetPosition
                and pulsingController.controller.accelerationStrategy.canStop()):
            # todo: check if still needed.
            pulsingController.controller.accelerationStrategy.done()
            # Letting done Driver continue. This might disrupt timing for other pulsing controllers.
            pulsingController.controller.currentJob.block.set_result(True)
            controllerIsDone = True
        elif pulsingController.interruptPredicate():
            pulsingController.controller.currentJob.block.set_result(True)
            controllerIsDone = True
        return controllerIsDone

    class PulsingController:
        # TODO: Recycle instances to prevent garbage collection. Can help too with keeping pulseCount on interruptions
        def __init__(self, controller: MotorDriver, sleepTime, targetPosition, fn=None, interruptPredicate=None,
                     eventInAdvanceSteps=None, eventName="steppingComplete", high=GPIO.HIGH):
            self.controller: MotorDriver = controller

            if isinstance(controller, UnipolarMotorDriver):
                self.addDueControllerFn = self.addUnipolarPulses
            elif isinstance(controller, ThirdPartyAdapter):
                self.addDueControllerFn = self.addAdapterPulses
            else:
                self.addDueControllerFn = self.addBipolarPulses

            self.sleepTime = sleepTime
            self.targetPosition = targetPosition
            self.fn = fn
            self.interruptPredicate = interruptPredicate
            self.eventInAdvanceSteps = eventInAdvanceSteps
            self.eventName = eventName

            # Signed steps (actually pulses) relative to stepping job start, not absolute position.
            self.pulseCount = 0
            self.high = high

        def pulseStart(self):
            assert isinstance(self.controller, ThirdPartyAdapter)
            self.controller.pulseStart(self.pulseCount)

        def pulseStop(self):
            assert isinstance(self.controller, ThirdPartyAdapter)
            self.controller.pulseStop()

        def addAdapterPulses(self, duePulses: list, duePulsesStates: list, dueControllers: list,
                             bipolarStepPins: list, delegatedDuePulses: list):
            dueControllers.append(self)
            delegatedDuePulses.append(self)
            self.pulseCount += self.controller.accelerationStrategy.realDirection

        def addUnipolarPulses(self, duePulses: list, duePulsesStates: list, dueControllers: list,
                              bipolarStepPins: list, delegatedDuePulses):
            dueControllers.append(self)
            duePulses.extend(self.controller.stepGpioPin)
            duePulsesStates.extend(self.controller.sequence.getStepSequence(self.pulseCount))
            self.pulseCount += self.controller.accelerationStrategy.realDirection

        def addBipolarPulses(self, duePulses: list, duePulsesStates, dueControllers: list,
                             bipolarStepPins: list, delegatedDuePulses):
            dueControllers.append(self)
            duePulses.append(self.controller.stepGpioPin)
            bipolarStepPins.append(self.controller.stepGpioPin)
            duePulsesStates.append(self.high)
