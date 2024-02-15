# Static or responsive/dynamic navigation implementations.
import threading
import time
import traceback
from concurrent.futures import Future

from RPi import GPIO

from stepper_motors_juanmf1.myMath import cmp
from stepper_motors_juanmf1.BlockingQueueWorker import BlockingQueueWorker
from stepper_motors_juanmf1.Controller import MotorDriver
from stepper_motors_juanmf1.EventDispatcher import EventDispatcher
from stepper_motors_juanmf1.SortedDict import SortedDict
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint


class Navigation:
    _COMPLETED_FUTURE = None  # Class variable

    def __init__(self):
        Navigation.get_completed_future()
        self.driverPulseTimeNs = None

    def setDriverPulseTimeNs(self, *, driverPulseTimeUs):
        self.driverPulseTimeNs = 1000 * driverPulseTimeUs

    @classmethod
    def get_completed_future(cls):
        # Lazily initialize the Future if not already done
        if cls._COMPLETED_FUTURE is None:
            cls._COMPLETED_FUTURE = Future()
            cls._COMPLETED_FUTURE.set_result(True)
        return cls._COMPLETED_FUTURE

    def go(self, controller, targetPosition, accelerationStrategy, fn, interruptPredicate, eventInAdvanceSteps=10,
           eventName="steppingComplete"):
        return self.get_completed_future()

    @staticmethod
    def pulseController(controller: MotorDriver):
        # tprint(f"Setting step pin {controller.stepGpioPin} HIGH.")
        controller.pulseStart()
        controller.usleep(controller.PULSE_TIME_MICROS)
        controller.pulseStop()

    @staticmethod
    def isInterruptible():
        pass


class StaticNavigation(Navigation):

    def go(self, controller: MotorDriver, targetPosition, accelerationStrategy, fn, interruptPredicate,
           eventInAdvanceSteps=10, eventName="steppingComplete"):

        steps = abs(controller.getCurrentPosition() - targetPosition)
        # Todo: find out  if -1 works as LOW (normally set to 0) for direction pin.
        controller.setDirection(cmp(targetPosition, controller.getCurrentPosition()))
        for i in range(steps):
            self.pulseController(controller)

            accelerationStrategy.computeSleepTimeUs(i, steps)
            # tprint(f"Sleeping for {accelerationStrategy.currentSleepTimeUs} uS.")
            controller.usleep(accelerationStrategy.currentSleepTimeUs)
            # fn should not consume many CPU instructions to avoid delays between steps.
            if fn:
                fn(controller.getCurrentPosition(), targetPosition, accelerationStrategy.realDirection,
                   controller.multiprocessObserver)

            if abs(steps - i) == eventInAdvanceSteps:
                EventDiapatcher._instance().publishMainLoop(eventName + "Advance", {'position': i})

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
        while not (controller.getCurrentPosition() == targetPosition and accelerationStrategy.canStop()):
            if interruptPredicate():
                tprint("Interrupting stepping job.")
                return
            # Direction is set in position based acceleration' state machine.
            controller.setCurrentPosition(accelerationStrategy.computeSleepTimeUs(
                controller.getCurrentPosition(), targetPosition, lambda d: controller.setDirection(d)))

            self.pulseController(controller)

            micros = accelerationStrategy.getCurrentSleepUs()

            controller.usleep(micros)
            position = controller.getCurrentPosition()
            # fn should not consume many CPU instructions to avoid delays between steps.
            if fn:
                fn(controller.getCurrentPosition(), targetPosition, accelerationStrategy.realDirection,
                   controller.multiprocessObserver)

            if (abs(targetPosition - position) == eventInAdvanceSteps
                and cmp(targetPosition, position) == controller.accelerationStrategy.realDirection):
                EventDiapatcher._instance().publishMainLoop(eventName + "Advance", {'position': position})


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
    _instance = None

    def __new__(cls, *args, **kwargs):
        """
        Singleton
        """
        if not cls._instance:
            cls._instance = super(BasicSynchronizedNavigation, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self, high=GPIO.HIGH, low=GPIO.LOW):
        BlockingQueueWorker.__init__(self, self.__doGo, jobQueueMaxSize=4, workerName="SynchronizedNavigation")
        Navigation.__init__(self)
        # {startTimNs: [(controller, sleepTime), ...]}.
        self.pulsingControllers = SortedDict()
        self.high = high
        self.low = low
        self.lock = threading.Lock()
        self.upPins = []
        self.eventDispatcher = EventDispatcher.instance()

    # Todo: this will be called from multiple threads, one per driver. Handle synchronization accordingly.
    # Waits to get all Synchronized controllers to go then coordinates their steps so that all sleep and step at once.
    def go(self, controller: MotorDriver, targetPosition, accelerationStrategy, fn, interruptPredicate,
           eventInAdvanceSteps=10, eventName="steppingComplete"):
        # Contiguous method calls in RPi are about 10uS apart. for a collision here it'd need to be 10K times faster.
        # this job is navigation level. there is a job per driver we need to complete independently.
        navJob = self.work([self.PulsingController(
                controller, controller.stepperMotor.minSleepTime, targetPosition, fn, interruptPredicate,
                eventInAdvanceSteps=eventInAdvanceSteps, eventName=eventName)])
        return controller.currentJob.block

    def __doGo(self, pulsingController):
        self.putPulsingController(time.monotonic_ns(), pulsingController)
        duePulses = []
        dueControllers = []
        try:
            while self.pulsingControllers:
                if self.hasQueuedJobs():
                    # Say driver_1 just put next pulse in pulsingControllers. and new job is from driver_1,
                    #  what next?
                    # Can't be, a running Driver because it's blocked, removed from pulsingControllers to check
                    # interrupted, then let go to pick up it's next job.
                    return
                now = time.monotonic_ns()
                while self.pulsingControllers:
                    pulseTime, pulsingController = self.pulsingControllers.popitem()
                    if pulseTime > now:
                        self.putPulsingController(pulseTime, pulsingController)
                        break
                    if self.checkDone(pulsingController):
                        continue
                    duePulses.append(pulsingController.controller.stepGpioPin)
                    dueControllers.append(pulsingController)

                if duePulses:
                    GPIO.output(duePulses, self.high)
                    pulseTime = time.monotonic_ns()
                    self.updateSleepTimes(dueControllers, pulseTime, count=2)
                    while pulseTime + self.driverPulseTimeNs > time.monotonic_ns():
                        # Small active wait in case updateSleepTimes() didn't consume Driver's min pulse time.
                        pass
                    GPIO.output(duePulses, self.low)
                    # Finish with remaining controllers after sending LOW
                    if dueControllers:
                        self.updateSleepTimes(dueControllers, pulseTime)
                dueControllers.clear()
                duePulses.clear()

        except Exception as e:
            print(f"SOMETHING WRONG {e}")
            print(traceback.format_exc())

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
            tprint(f"Interrupting stepping job for {pulsingController.controller.workerName}.")
            pulsingController.controller.currentJob.block.set_result(True)
            controllerIsDone = True
        return controllerIsDone

    class PulsingController:
        def __init__(self, controller: MotorDriver, sleepTime, targetPosition, fn=None, interruptPredicate=None,
                     eventInAdvanceSteps=None, eventName="steppingComplete"):
            self.controller: MotorDriver = controller
            self.sleepTime = sleepTime
            self.targetPosition = targetPosition
            self.fn = fn
            self.interruptPredicate = interruptPredicate
            self.eventInAdvanceSteps = eventInAdvanceSteps
            self.eventName = eventName
