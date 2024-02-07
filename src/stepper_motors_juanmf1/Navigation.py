# Static or responsive/dynamic navigation implementations.
import asyncio
import threading
import time
from collections import OrderedDict
from concurrent.futures import Future

from RPi import GPIO
from stepper_motors_juanmf1.myMath import cmp
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint

from stepper_motors_juanmf1.BlockingQueueWorker import BlockingQueueWorker
from stepper_motors_juanmf1.Controller import BipolarStepperMotorDriver, MotorDriver
from stepper_motors_juanmf1.StepperMotor import StepperMotor

from src.stepper_motors_juanmf1.EventDispatcher import EventDispatcher
from src.stepper_motors_juanmf1.myMath import cmp


class Navigation:
    _COMPLETED_FUTURE = None  # Class variable

    def __init__(self):
        Navigation.get_completed_future()

    @classmethod
    def get_completed_future(cls):
        # Lazily initialize the Future if not already done
        if cls._COMPLETED_FUTURE is None:
            cls._COMPLETED_FUTURE = Future()
            cls._COMPLETED_FUTURE.set_result(True)
        return cls._COMPLETED_FUTURE

    def go(self, controller, targetPosition, accelerationStrategy, fn, interruptPredicate):
        pass

    @staticmethod
    def pulseController(controller):
        # tprint(f"Setting step pin {controller.stepGpioPin} HIGH.")
        controller.pulseStart()
        controller.usleep(controller.PULSE_TIME_MICROS)
        controller.pulseStop()

    @staticmethod
    def pulseControllerStart(controller):
        # tprint(f"Setting step pin {controller.stepGpioPin} HIGH.")
        controller.pulseStart()
        return time.monotonic_ns()

    @staticmethod
    def pulseControllerStop(controller):
        controller.pulseStop()
        return time.monotonic_ns()

    @staticmethod
    def isInterruptible():
        pass


class StaticNavigation(Navigation):

    def go(self, controller, targetPosition, accelerationStrategy, fn, interruptPredicate):
        steps = abs(controller.currentPosition - targetPosition)
        # Todo: find out  if -1 works as LOW (normally set to 0) for direction pin.
        controller.setDirection(cmp(targetPosition, controller.currentPosition))
        for i in range(steps):
            self.pulseController(controller)

            accelerationStrategy.computeSleepTimeUs(i, steps)
            # tprint(f"Sleeping for {accelerationStrategy.currentSleepTimeUs} uS.")
            controller.usleep(accelerationStrategy.currentSleepTimeUs)
            # fn should not consume many CPU instructions to avoid delays between steps.
            fn(controller.currentPosition, targetPosition, accelerationStrategy.realDirection)

        accelerationStrategy.done()
        controller.setDirection(GPIO.LOW)
        return Navigation._COMPLETED_FUTURE

    @staticmethod
    def isInterruptible():
        return False


class DynamicNavigation(Navigation):

    def go(self, controller, targetPosition, accelerationStrategy, fn, interruptPredicate):
        # Can cross targetPosition with some speed > 0, that's not a final state.
        while not (controller.currentPosition == targetPosition and accelerationStrategy.canStop()):
            if interruptPredicate():
                tprint("Interrupting stepping job.")
                return
            # Direction is set in position based acceleration' state machine.
            controller.setCurrentPosition(accelerationStrategy.computeSleepTimeUs(
                controller.currentPosition, targetPosition, lambda d: controller.setDirection(d)))

            self.pulseController(controller)

            # tprint(f"Driver's self.currentPosition: {controller.currentPosition}. targetPosition: {targetPosition}")
            micros = accelerationStrategy.getCurrentSleepUs()

            controller.usleep(micros)
            # fn should not consume many CPU instructions to avoid delays between steps.
            fn(controller.currentPosition, targetPosition, accelerationStrategy.realDirection)
            # tprint("")

        # todo: check if still needed.
        accelerationStrategy.done()
        # Todo: GPIO.LOW is not controller specific. fix with controller.setDirection(controller.defaultDirection)
        #  or similar
        controller.setDirection(GPIO.LOW)
        return Navigation._COMPLETED_FUTURE

    @staticmethod
    def isInterruptible():
        return True


class BasicSynchronizedNavigation(Navigation, BlockingQueueWorker):

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
        self.pulsingControllers = OrderedDict()
        self.high = high
        self.low = low
        self.lock = threading.Lock()
        self.upPins = []
        self.eventDispatcher = EventDispatcher()

    # Todo: this will be called from multiple threads, one per driver. Handle synchronization accordingly.
    # Waits to get all Synchronized controllers to go then coordinates their steps so that all sleep and step at once.
    def go(self, controller: MotorDriver, targetPosition, accelerationStrategy, fn, interruptPredicate,
           eventInAdvanceSteps=10, eventName="steppingComplete"):
        # Contiguous method calls in RPi are about 10uS apart. for a collision here it'd need to be 10K times faster.
        # this job is navigation level. there is a job per driver we need to complete independently.
        navJob = self.work([SynchronizedNavigation.PulsingController(
                controller, controller.stepperMotor.minSleepTime, targetPosition, fn, interruptPredicate,
                eventInAdvanceSteps=10, eventName="steppingComplete")])
        return controller.currentJob.block

    def __doGo(self, pulsingController):
        self.pulsingControllers[time.monotonic_ns()] = pulsingController
        while self.pulsingControllers:
            if self.hasQueuedJobs():
                return
            duePulses = []
            dueControllers = []
            now = time.monotonic_ns()
            while True:
                pulseTime, pulsingController = self.pulsingControllers.popitem()
                if pulseTime > now:
                    self.pulsingControllers[pulseTime] = pulsingController
                    break
                if self.checkDone(pulsingController):
                    continue
                duePulses.append(pulsingController.controller.stepGpioPin)
                dueControllers.append(pulsingController)

            if duePulses:
                GPIO.output(duePulses, self.high)
                pulseTime = time.monotonic_ns()
                self.upPins.extend(duePulses)
                self.updateSleepTimes(dueControllers, pulseTime)
                GPIO.output(duePulses, self.low)

    def updateSleepTimes(self, pulsingControllers: list, pulseTimeNs):
        while pulsingControllers:
            pulsingController = pulsingControllers.pop()
            pulsingController.controller.setCurrentPosition(
                (pulsingController.controller.accelerationStrategy.computeSleepTimeUs(
                    pulsingController.controller.currentPosition, pulsingController.targetPosition,
                    lambda d: pulsingController.controller.setDirection(d))))
            nextPulse = pulseTimeNs + 1000 * pulsingController.controller.accelerationStrategy.getCurrentSleepUs()
            self.pulsingControllers[nextPulse] = pulsingController

            # Called while Stepper is in flight to finish this step.
            stepsFromGoal = pulsingController.controller.currentPosition - pulsingController.targetPosition
            if (abs(stepsFromGoal) == pulsingController.eventInAdvanceSteps
                and cmp(pulsingController.targetPosition, pulsingController.controller.currentPosition)
                    == pulsingController.controller.realDirection):
                # Firing event
                self.eventDispatcher._dispatchMainLoop(pulsingController.eventName, {"stepsFromGoal": stepsFromGoal})

    def checkDone(self, pulsingController):
        controllerIsDone = False
        if (pulsingController.controller.currentPosition == pulsingController.targetPosition
                and pulsingController.controller.accelerationStrategy.canStop()):
            # todo: check if still needed.
            pulsingController.controller.accelerationStrategy.done()
            # Letting done or stopped Driver continue. This might disrupt timing for other pulsing controllers.
            pulsingController.controller.currentJob.block.set_result(True)
            controllerIsDone = True
        elif pulsingController.interruptPredicate():
            tprint(f"Interrupting stepping job for {pulsingController.controller.workerName}.")
            controllerIsDone = True
        return controllerIsDone


# Todo: In applications where several motors need to work together in close coordination, like 3D printing, we can't
#   just have a set of independent drivers timing their pulses to their convenience. This Navigation ensures steps are
#   locked into a rithm.
class SynchronizedNavigation(Navigation, BlockingQueueWorker):
    _instance = None

    def __new__(cls, *args, **kwargs):
        """
        Singleton
        """
        if not cls._instance:
            cls._instance = super(SynchronizedNavigation, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self, high=GPIO.HIGH, low=GPIO.LOW):
        # def __init__(self, jobConsumer, *, jobQueueMaxSize=2, workerName="John_Doe_Worker"):
        BlockingQueueWorker.__init__(self, self.__doGo, jobQueueMaxSize=4, workerName="SynchronizedNavigation")
        Navigation.__init__(self)
        # {startTimNs: [(controller, sleepTime), ...]}.
        self.pulsingControllers = OrderedDict()
        self.high = high
        self.low = low
        self.lock = threading.Lock()
        self.loop = loop = asyncio.get_event_loop()

    # Todo: this will be called from multiple threads, one per driver. Handle synchronization accordingly.
    # Waits to get all Synchronized controllers to go then coordinates their steps so that all sleep and step at once.
    def go(self, controller: MotorDriver, targetPosition, accelerationStrategy, fn, interruptPredicate):
        # Contiguous method calls in RPi are about 10uS apart. for a collision here it'd need to be 10K times faster.
        with self.lock:
            self.pulsingControllers[time.monotonic_ns()] = [SynchronizedNavigation.PulsingController(
                controller, controller.stepperMotor.minSleepTime, targetPosition, fn, interruptPredicate)]
        # this job is navigation level. there is a job per driver we need to complete independently.
        navJob = self.work([])
        return controller.currentJob.block

    def __doGo(self):
        while len(self.pulsingControllers) > 0:
            with self.lock:
                # waits till 1st pulse is due.
                pulseUpTime, pulsedPins, pulsedControllers = (
                    self.pulseControllerStartBatch(self.pulsingControllers, self.high))
                # Using Pulse HIGH time to perform computations (instead of waiting).
                self.removeReadyOrInterruptedControllers(pulsedControllers)
                self.updatePulsedControllersSleepTimes(pulseUpTime, pulsedControllers)

            pulseLowTime = self.pulseControllerStopBatch(pulsedPins, self.low)
            tprint(f"Controller pulseWith ns: {pulseLowTime - pulseUpTime}")

    @staticmethod
    def isInterruptible():
        return True

    def removeReadyOrInterruptedControllers(self, pulsedControllers):
        """
        interrupted controllers might skip a step, timewise?
        @return:
        """
        controllersToRemove = []
        for pulseTime, controllerList in self.pulsingControllers.items():
            for idx, pulsingController in enumerate(controllerList):
                if (pulsingController.controller.currentPosition == pulsingController.targetPosition
                        and pulsingController.controller.accelerationStrategy.canStop()):
                    controllersToRemove.append((pulseTime, idx))
                elif pulsingController.interruptPredicate():
                    tprint(f"Interrupting stepping job for {pulsingController.controller.workerName}.")
                    controllersToRemove.append((pulseTime, idx))

        pulseTimesToRemove = []
        for (pulseTime, idx) in controllersToRemove:
            pulsingC = self.pulsingControllers[pulseTime].pop(idx)
            # todo: check if still needed.
            pulsingC.controller.accelerationStrategy.done()
            # Todo: GPIO.LOW is not controller specific. fix with controller.setDirection(controller.defaultDirection)
            #  or similar
            pulsingC.controller.setDirection(GPIO.LOW)
            pulsingC.controller.currentJob.block.set_result(True)
            if pulsingC in pulsedControllers:
                pulsedControllers.remove(pulsingC)
            if len(self.pulsingControllers[pulseTime]) == 0:
                pulseTimesToRemove.append(pulseTime)

        for pulseTime in pulseTimesToRemove:
            self.pulsingControllers.pop(pulseTime)

    class PulsingController:
        def __init__(self, controller: MotorDriver, sleepTime, targetPosition, fn=None, interruptPredicate=None,
                     eventInAdvanceSteps=10, eventName="steppingComplete"):
            self.controller = controller
            self.sleepTime = sleepTime
            self.targetPosition = targetPosition
            self.fn = fn
            self.interruptPredicate = interruptPredicate
            self.eventInAdvanceSteps = eventInAdvanceSteps
            self.eventName = eventName

    def updatePulsedControllersSleepTimes(self, pulseTime, pulsedControllers):
        for pulsingController in pulsedControllers:
            pulsingController.controller.setCurrentPosition(
                (pulsingController.controller.accelerationStrategy.computeSleepTimeUs(
                    pulsingController.controller.currentPosition, pulsingController.targetPosition,
                    lambda d: pulsingController.controller.setDirection(d))))

            # tprint(f"Driver's self.currentPosition: {controller.currentPosition}. targetPosition: {targetPosition}")
            nextPulseNs = pulseTime + 1000 * pulsingController.controller.accelerationStrategy.getCurrentSleepUs()

            # controller.usleep(micros)
            # Called while Stepper is in flight to finish this step.
            # fn should not consume many CPU instructions to avoid delays between steps.
            pulsingController.fn(pulsingController.controller.currentPosition,
                                 pulsingController.targetPosition,
                                 pulsingController.controller.accelerationStrategy.realDirection)

            # Scheduling next pulse.
            if nextPulseNs in self.pulsingControllers:
                self.pulsingControllers[nextPulseNs].append(pulsingController)
            else:
                self.pulsingControllers[nextPulseNs] = [pulsingController]

    @staticmethod
    def pulseControllerStartBatch(pulsingControllers: OrderedDict, high=GPIO.HIGH):
        """
        LIMITATION: all controllers must use same value for start and stop, either HIGH or LOW.
        This method tries to merge pulses when acceptably close.
        @param high: value controllers use for starting a stepping pulse.
        @param pulsingControllers: controller ordered dict {startTimNs: [PulsingController, ...]}.
        """
        start = time.monotonic_ns()
        SynchronizedNavigation.attemptMergePulses(pulsingControllers)

        pulsedPins = []
        pulsedControllers = []
        pastPulses = []
        first = True
        now = time.monotonic_ns()
        hadToSleep = False
        for pulseTimeNs in pulsingControllers:
            # wait till 1st pulse is due
            if first and now >= pulseTimeNs:
                hadToSleep = now - pulseTimeNs
                MotorDriver.usleep(hadToSleep // 1000)
                first = False
                now = time.monotonic_ns()

            if now < pulseTimeNs:
                break
            pastPulses.append(pulseTimeNs)
            for pulsingController in pulsingControllers[pulseTimeNs]:
                pulsedPins.append(pulsingController.controller.stepGpioPin)
                pulsedControllers.append(pulsingController)

        for pastPulse in pastPulses:
            pulsingControllers.pop(pastPulse)
        end = time.monotonic_ns()

        tprint(f"Merging close pulses and collecting pulsingPins ({pulsedPins}) takes: {end-start} ns, and hadToSleep: {hadToSleep}")
        GPIO.output(pulsedPins, high)
        return time.monotonic_ns(), pulsedPins, pulsedControllers

    @staticmethod
    def attemptMergePulses(pulsingControllers: OrderedDict):
        # Attempt to merge with next.
        key_iterator = iter(pulsingControllers)
        emptiedTimeSlots = []
        currentTime = next(key_iterator, None)
        nextTime = next(key_iterator, None)
        while nextTime is not None:
            deltaTime = nextTime - currentTime
            for index, pulsingController in enumerate(pulsingControllers[nextTime]):
                # pulsingController: (controller, currentSleepTime)
                if deltaTime <= pulsingController.sleepTime / 10:
                    # Merge pulses
                    pulsingControllers[currentTime].append(pulsingController)
                    pulsingControllers[nextTime].pop(index)
                    if len(pulsingControllers[nextTime]) == 0:
                        emptiedTimeSlots.append(nextTime)
            currentTime = nextTime
            nextTime = next(key_iterator, None)

        if len(emptiedTimeSlots) > 0:
            for pulseTime in emptiedTimeSlots:
                pulsingControllers.pop(pulseTime)

    @staticmethod
    def pulseControllerStopBatch(pulsedPins: list, low=GPIO.LOW):
        """
        LIMITATION: all controllers must use same value for start and stop, either HIGH or LOW.
        @param pulsedPins: pulsed pins set.
        @param low: value controllers use for stopping a stepping pulse.
        """
        GPIO.output(pulsedPins, low)
        return time.monotonic_ns()