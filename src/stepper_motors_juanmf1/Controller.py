import ctypes
import threading
from abc import abstractmethod
import time
from typing import Callable

from RPi import GPIO

from stepper_motors_juanmf1.AccelerationStrategy import AccelerationStrategy
from stepper_motors_juanmf1.BlockingQueueWorker import BlockingQueueWorker
from stepper_motors_juanmf1.EventDispatcher import EventDispatcher
from stepper_motors_juanmf1.StepperMotor import StepperMotor
from stepper_motors_juanmf1.myMath import sign, cmp
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint


class NoDirectionPinDriver:
    pass


class ThirdPartyAdapter:
    """
    Marker interface to signal that this Controller's pins ot protected attributes can't be accessed safely.
    Use High level API methods instead.
    """

    @abstractmethod
    def pulseStart(self, stepRelativeToJobStart=None):
        pass

    @abstractmethod
    def pulseStop(self):
        pass

class DriverSharedPositionStruct(ctypes.Structure):
    """
    In Multiprocessing, position will be shared through these fields.
    """
    _fields_ = [("position", ctypes.c_int),
                ("direction", ctypes.c_int)]


# Todo: migrate to https://pypi.org/project/python-periphery/
class MotorDriver(BlockingQueueWorker):
    LOCK = threading.Lock()
    INSTANCES_COUNT = 0
    PULSE_TIME_MICROS = None
    INITIALIZED = False

    def __init__(self, *, stepperMotor,
                 directionGpioPin,
                 stepGpioPin,
                 accelerationStrategy,
                 navigation,
                 workerName,
                 jobQueueMaxSize=2,
                 jobQueue=None,
                 sharedMemory=None,
                 isProxy=False,
                 jobCompletionObserver=None):

        workerName = f"{self.__class__.__name__}_{MotorDriver.INSTANCES_COUNT}_" \
                     if workerName is None else workerName
        super().__init__(self._operateStepper,
                         jobQueueMaxSize=jobQueueMaxSize,
                         workerName=workerName,
                         jobQueue=jobQueue,
                         isProxy=isProxy,
                         jobCompletionObserver=jobCompletionObserver)

        MotorDriver.INSTANCES_COUNT += 1
        self.stepGpioPin = stepGpioPin
        self.directionGpioPin = directionGpioPin
        self.stepperMotor: StepperMotor = stepperMotor
        # Tracks current either by position for position based movement (DynamicNavigation, interruptible)
        # or by fixed number of steps (StaticNavigation, uninterruptible).
        self.navigation = navigation
        self.navigation.setDriverPulseTimeNs(driverPulseTimeUs=self.PULSE_TIME_MICROS)
        self.accelerationStrategy = accelerationStrategy
        if sharedMemory is not None:
            self.sharedLock = sharedMemory[0]
            self.sharedPosition = sharedMemory[1]
            self.multiprocessObserver = sharedMemory[2]
        else:
            self.multiprocessObserver = self.sharedPosition = self.sharedLock = None

    def _oneTimeInit(self):
        with self.LOCK:
            if MotorDriver.INITIALIZED:
                return

            MotorDriver.INITIALIZED = True
            GPIO.setmode(GPIO.BCM)

    @abstractmethod
    def _operateStepper(self, direction, steps):
        pass

    @abstractmethod
    def getCurrentPosition(self):
        pass

    @abstractmethod
    def setCurrentPosition(self, position):
        pass

    @abstractmethod
    def setDirection(self, directionState):
        pass

    @abstractmethod
    def pulseStart(self, stepRelativeToJobStart=None):
        """
        In most controllers this would mean set step pint to HIGH
        @param stepRelativeToJobStart: for each step job, starts from zero, not related to absolute position.
                                       Used to pick sequence in Unipolar Driers. Depending on Direction should increase
                                       or decrease. Can be negative.
        """
        pass

    @abstractmethod
    def pulseStop(self):
        """
        In most controllers this would mean set step pint to LOW
        @return:
        """
        pass

    @staticmethod
    def usleep(micros):
        """
        Sleep time strategies tested on RPI 4B
        """
        if micros < 1000:
            # Todo: if libc.usleep works, use active wait under 300uS
            # Active wait
            # Shows overhead of ~5-10 uS
            # Driver's pulse time is generally much shorter than step period, i.e. low duty-cycle.
            nanos = (micros * 1000)
            start = time.monotonic_ns()
            while True:
                end = time.monotonic_ns()
                if end - start >= nanos:
                    break
        # Todo: relying on libc.usleep is hanging up on my RPI, it should be a good fit for sleep times 200us < t < 2ms
        # elif micros < 2000:
        #     # Shows overhead of ~110 uS
        #     # self.libc.usleep(micros)
        #     time.sleep(micros / 1_000_000)
        else:
            # Accuracy is very poor bellow a few mS, significant overhead, test with `benchSleep.benchSleep()`
            # But the overhead is relatively consistent for each time range, so you can slee less, accounting for tested
            # overhead in your platform.
            time.sleep(micros / 1_000_000)

    def __str__(self):
        return (f"{type(self)} \n"
                f"Stepper Motor: {self.stepperMotor}\n"
                f"Direction GPIO Pin: {self.directionGpioPin}\n"
                f"Step GPIO Pin: {self.stepGpioPin}\n"
                f"Navigation Strategy: {self.navigation}\n"
                f"Acceleration Strategy: {self.accelerationStrategy}\n"
                f"MultiprocessObserver: {self.multiprocessObserver}\n"
                f"sharedPosition: {self.sharedPosition}\n"
                f"sharedLock: {self.sharedLock}\n"
                f"Worker details {super().__str__()}")


class BipolarStepperMotorDriver(MotorDriver):
    """
    Bipolar stepper motor driver abstract implementation.
    Uses a dedicated thread to handle pulses to driver hardware in a non-blocking fashion.
    Uses StepperMotor specs to keep timing within acceptable ranges and can be configured wit different
    acceleration strategies.
    Accepts Navigation modes to enable interruptions or keep head down until a stepping job is 100% done.
    Client code can send (non-blocking) instructions as :func:`~stepCounterClockWise` or
    :func:`~stepClockWise` passing a callable to update position as the motor moves.
    """

    CW = GPIO.HIGH  # Clockwise Rotation
    CCW = GPIO.LOW  # Counterclockwise Rotation
    CLOSEST = -1    # In absolute positioning `moveTo()` determine cheapest from CW vs CCW.

    # Mode pins are static, as they are shared among Turret motors.
    RESOLUTION = None

    RESOLUTION_MULTIPLE = {'Full':  1,
                           'Half':  1 / 2,
                           '1/2':   1 / 2,
                           '1/4':   1 / 4,
                           '1/8':   1 / 8,
                           '1/16':  1 / 16,
                           '1/32':  1 / 32,
                           '1/64':  1 / 64,
                           '1/128': 1 / 128,
                           '1/256': 1 / 256}

    DEFAULT_STEPPING_MODE = 'Full'

    PULSE_TIME_MICROS = None
    """
    Many sites will show 50% duty cycle from controller code. Drivers actually take short signal pulses.
    """

    PULSE_STATE = None
    """
    Only tested with DRV8825 but LOW pulse is also possible on HIGH background.
    """

    def __init__(self, *,
                 stepperMotor: StepperMotor,
                 accelerationStrategy: AccelerationStrategy,
                 directionGpioPin,
                 stepGpioPin,
                 navigation,
                 sleepGpioPin=None,
                 stepsMode=DEFAULT_STEPPING_MODE,
                 modeGpioPins=None,
                 enableGpioPin=None,
                 workerName=None,
                 useHoldingTorque=None,
                 jobQueue=None,
                 sharedMemory=None,
                 isProxy=False,
                 steppingCompleteEventName="steppingComplete",
                 jobCompletionObserver=None):
        """
        Multiple drivers could share the same mode pins (assuming current supply from pin is enough,
        and the drivers' mode pins are bridged same to same)
        Raspberry board pins layout:
        [GPIOn] n * * m [GPIOm]; n,m = physical board pin order. * = physical pin. [GPIOm],[GPIOm] = GPIO pin n,m

        [3v3 Power] 1  * *  2 [5v Power] // 3v3 Power to controller IC. To Reset and Sleep pins.
        [GPIO_2]    3  * *  4 [5v Power]
        [GPIO_3]    5  * *  6 [GND]
        [GPIO_4]    7  * *  8 [GPIO_14] // default modeGpioPins[0]
        [GND]       9  * * 10 [GPIO_15] // default modeGpioPins[1]
        [GPIO_17]   11 * * 12 [GPIO_18] // default modeGpioPins[2]
        [GPIO_27]   13 * * 14 [GND]
        [GPIO_22]   15 * * 16 [GPIO_23]
        [3v3 Power] 17 * * 18 [GPIO_24] // 3v3 Power to controller IC. To Reset and Sleep pins.
        [GPIO_10]   19 * * 20 [GND]
        [GPIO_9]    21 * * 22 [GPIO_25]
        [GPIO_11]   23 * * 24 [GPIO_8]
        [GND]       25 * * 26 [GPIO_7]
        [GPIO_0]    27 * * 28 [GPIO_1]
        [GPIO_5]    29 * * 30 [GND]
        [GPIO_6]    31 * * 32 [GPIO_12]
        [GPIO_13]   33 * * 34 [GND]
        [GPIO_19]   35 * * 36 [GPIO_16]
        [GPIO_26]   37 * * 38 [GPIO_20]
        [GND]       39 * * 40 [GPIO_21]
        """
        assert stepperMotor and accelerationStrategy and navigation
        assert (isinstance(self, ThirdPartyAdapter)
                or (isinstance(self, NoDirectionPinDriver) and stepGpioPin)
                or (directionGpioPin and stepGpioPin))
        super().__init__(stepperMotor=stepperMotor,
                         directionGpioPin=directionGpioPin,
                         stepGpioPin=stepGpioPin,
                         accelerationStrategy=accelerationStrategy,
                         navigation=navigation,
                         workerName=workerName,
                         jobQueue=jobQueue,
                         sharedMemory=sharedMemory,
                         isProxy=isProxy,
                         jobCompletionObserver=jobCompletionObserver)

        self._steppingCompleteEventName = steppingCompleteEventName
        self.enableGpioPin = enableGpioPin
        self.modeGpioPins = modeGpioPins  # Microstep Resolution GPIO Pins
        self.stepsMode = stepsMode
        self.sleepGpioPin = sleepGpioPin

        # Counts pulses, Clockwise adds, counterclockwise subtracts.
        self.currentPosition = 0
        # Corrects self.currentPosition for stepping mode.
        self.actualCurrentPosition = 0
        self.steppingModeMultiple = self.RESOLUTION_MULTIPLE[stepsMode]
        self.currentDirection = None

        self.isRunning = False

        # False means no current when no stepping. True sends current for holding Torque.
        # sleepGpioPin and enableGpioPin can be set by hardware, if provided a GPIO they are used to save power
        # unless overriden by useHoldingTorque
        # Never sleeps/disable
        self.useHoldingTorque = useHoldingTorque if useHoldingTorque is not None \
            else not (sleepGpioPin is None and enableGpioPin is None)

        self._initGpio(stepsMode)

        # Todo: libc.usleep fails on my RPI. see `self.usleep()`
        # self.libc = ctypes.CDLL('libc.so.6')

    def _initGpio(self, stepsMode):
        self._oneTimeInit()

        GPIO.setup(self.directionGpioPin, GPIO.OUT)
        self.setDirection(GPIO.LOW)

        GPIO.setup(self.stepGpioPin, GPIO.OUT)
        GPIO.output(self.stepGpioPin, GPIO.LOW)

        if self.enableGpioPin is not None:
            GPIO.setup(self.enableGpioPin, GPIO.OUT)
            self.setEnableMode()

        if self.sleepGpioPin is not None:
            GPIO.setup(self.sleepGpioPin, GPIO.OUT)
            self.setSleepMode()

        if self.modeGpioPins is not None:
            GPIO.setup(self.modeGpioPins, GPIO.OUT)
            GPIO.output(self.modeGpioPins, self.RESOLUTION[stepsMode])
            self.steppingModeMultiple = self.RESOLUTION_MULTIPLE[stepsMode]

    def _doOperateStepper(self, direction, steps, fn=None, jobCompleteEventNamePrefix="", eventInAdvanceSteps=10):
        """
        Positive change in position is in clockwise direction, negative is counter clock wise.
        callable signature is expected as fn(currentPosition, targetPosition, realDirection)
        (realDirection from AccelerationStrategy)
        interruptibility is determined by Navigation instance injected at construction time.
        :param direction: self.CW or self.CCW
        :param steps: Number of steps, as we rely on direction, an error is raised if negative values are sent.
                      steps is Actual Pulses, i.e. in micro stepping 1/8, steps = 8 would move the stepper motor 1
                      actual step. use self.steppingModeMultiple to correct for microstepping mode:
                          intendedMotorSteps = steps / self.steppingModeMultiple
                          actualSteps        = steps * self.steppingModeMultiple
        # Todo: client knows targetPosition, would only need currentPosition and realDirection, but not every step.
        :param fn: callable to execute after each step. Contract:
            fn(controller.getCurrentPosition(), targetPosition, accelerationStrategy.realDirection)
            Note it does not work with lambdas on multiprocess!
        """
        if steps < 0:
            raise RuntimeError("Can't handle negative steps. Use direction (self.CW or self.CCW) & steps > 0 properly.")

        position = self.getCurrentPosition()
        if direction == self.CLOSEST:
            # steps is absolute position.
            targetPosition = self.findShortestPathTargetPosition(position, steps, self.stepperMotor.getSpr())
        else:
            signedDirection = 1 if direction == self.CW else -1
            targetPosition = int(position + (signedDirection * steps))

        self.isRunning = True

        if not self.useHoldingTorque:
            self.powerOnCoils()

        eventName = jobCompleteEventNamePrefix + self._steppingCompleteEventName
        # Multiprocess event propagation is in the order of 0.003 seconds or ~1 step at 300 PPS.

        block = self.navigation.go(self, targetPosition, self.accelerationStrategy, fn, self.isInterrupted,
                                   eventInAdvanceSteps=eventInAdvanceSteps, eventName=eventName)
        block.result()
        if self.navigation.isInterruptible() and self.isInterrupted():
            # Navigation finished, but we don't deactivate coils, or declare not running. as momentum could be
            # useful for interrupting job.
            return

        if not self.useHoldingTorque:
            self.shutDownCoils()

        self.isRunning = False
        startTime = self.currentJob.startTime.result()
        EventDispatcher.instance().publishMainLoop(eventName + "FinalStep",
                                                   {'position': self.currentPosition,
                                                    'startTime': startTime,
                                                    'endTime': time.monotonic_ns()
                                                    })

    @staticmethod
    def findShortestPathTargetPosition(position, stepNumber, spr):
        """
        @param position: Driver's current position any int.
        @param stepNumber: A step number within one revolution.
        @param spr: Steps per revolution.
        @return:
        """
        halfWay = spr // 2
        distance = (stepNumber - position) % spr
        shortest = halfWay - abs(abs(distance) - halfWay)
        return position + (shortest if (spr + distance) % spr < halfWay else -shortest)

    def _operateStepper(self, direction, steps, fn=None, jobCompleteEventNamePrefix="", maxStepsPerSecondOverride=None,
                        eventInAdvanceSteps=10):
        """
        @param maxStepsPerSecondOverride: for this stepping job, override motor's steps per second.
                                          maxPps == maxStepsPerSecond on Full step mode only.
        @return:
        """
        if maxStepsPerSecondOverride:
            oldPps = self.accelerationStrategy.getMaxPPS()
            self.accelerationStrategy.setMaxPPS(maxStepsPerSecondOverride)
            self._doOperateStepper(direction, steps, fn, jobCompleteEventNamePrefix, eventInAdvanceSteps)
            self.accelerationStrategy.setMaxPPS(oldPps)
        else:
            self._doOperateStepper(direction, steps, fn, jobCompleteEventNamePrefix, eventInAdvanceSteps)

    def shutDownCoils(self):
        if self.sleepGpioPin:
            self.setSleepMode(sleepOn=True)
        elif self.enableGpioPin:
            self.setEnableMode(enableOn=False)

    def powerOnCoils(self):
        if self.sleepGpioPin:
            self.setSleepMode(sleepOn=False)
        elif self.enableGpioPin:
            self.setEnableMode(enableOn=True)

    def getActualCurrentPosition(self):
        self.actualCurrentPosition = self.steppingModeMultiple * self.getCurrentPosition()
        return self.actualCurrentPosition

    def setCurrentPosition(self, position):
        if self.isProxy:
            # Enable proxy (MainProcess counterpart of childProcess driver) to reset position only when not running.
            assert not self.isRunning, "Proxy Can't change position when stepper is running."
        self.currentPosition = position
        if self.sharedPosition is not None:
            with self.sharedLock:
                self.sharedPosition.position = position
                self.sharedPosition.direction = self.currentDirection

    def getCurrentPosition(self):
        if self.isProxy:
            with self.sharedLock:
                self.currentPosition = self.sharedPosition.position
                self.currentDirection = self.sharedPosition.direction
        elif self.sharedPosition is not None and self.currentPosition != self.sharedPosition.position:
            # Most likely proxyDriver in MainProcess changed position, potential usecase setting to 0 (Homing).
            assert not self.isRunning, "Illegal state: Position externally modified while running."
            with self.sharedLock:
                self.currentPosition = self.sharedPosition.position
                self.currentDirection = self.sharedPosition.direction
        return self.currentPosition

    def isInterrupted(self):
        return self.hasQueuedJobs()

    def signedSteps(self, steps, *, fn=None, jobCompleteEventNamePrefix="", maxPpsOverride=None,
                    eventInAdvanceSteps=10) -> BlockingQueueWorker.Job:
        """
        Accounts for current micro-stepping mode. Steps is the actual, real, motor steps to run for.
        this is equivalent to signedMicroSteps() only in Full step mode.
        @param steps: REAL motor steps number
        @return: a BlockingQueueWorker.Job representing this stepping job.
        """
        return self.signedMicroSteps(int(steps / self.steppingModeMultiple),
                                     fn=fn,
                                     jobCompleteEventNamePrefix=jobCompleteEventNamePrefix,
                                     maxPpsOverride=maxPpsOverride,
                                     eventInAdvanceSteps=eventInAdvanceSteps)

    def stepClockWise(self, steps, *, fn=None, jobCompleteEventNamePrefix="", maxPpsOverride=None,
                      eventInAdvanceSteps=10) -> BlockingQueueWorker.Job:
        """
        Accounts for current micro-stepping mode. Steps is the the actual, real, motor steps to run for.
        this is equivalent to microStepClockWise() only in Full step mode.
        @param steps: REAL motor steps number
        @return: a BlockingQueueWorker.Job representing this stepping job.
        """
        return self.microStepClockWise(int(steps / self.steppingModeMultiple),
                                       fn=fn,
                                       jobCompleteEventNamePrefix=jobCompleteEventNamePrefix,
                                       maxPpsOverride=maxPpsOverride,
                                       eventInAdvanceSteps=eventInAdvanceSteps)

    def stepCounterClockWise(self, steps, *, fn=None, jobCompleteEventNamePrefix="", maxPpsOverride=None,
                                  eventInAdvanceSteps=10) -> BlockingQueueWorker.Job:
        """
        Accounts for current micro-stepping mode. Steps is the the actual, real, motor steps to run for.
        this is equivalent to microStepCounterClockWise() only in Full step mode.
        @param steps: REAL motor steps number
        @return: a BlockingQueueWorker.Job representing this stepping job.
        """
        return self.microStepCounterClockWise(int(steps / self.steppingModeMultiple),
                                              fn=fn,
                                              jobCompleteEventNamePrefix=jobCompleteEventNamePrefix,
                                              maxPpsOverride=maxPpsOverride,
                                              eventInAdvanceSteps=eventInAdvanceSteps)

    def moveTo(self, step, *, fn=None, jobCompleteEventNamePrefix="", maxPpsOverride=None, eventInAdvanceSteps=10):
        """
        move the shaft to an absolute position. 0 <= step <= SPR
        @param step:
        @return:
        """
        return self.work(
            paramsList=[self.CLOSEST, int(step / self.steppingModeMultiple), fn, jobCompleteEventNamePrefix,
                        maxPpsOverride, eventInAdvanceSteps],
            block=True)

    def signedMicroSteps(self, steps, *, fn=None, jobCompleteEventNamePrefix="", maxPpsOverride=None,
                         eventInAdvanceSteps=10) -> BlockingQueueWorker.Job:
        """
        Creates stepping jobs handled by self._operateStepper.
        Steps is affected by microstepping mode. Effectively one step = one real microstep
        Here Steps = Pulses, matchong actual motor steps ONLY IN FULL STEP MODE. see signedMicroSteps()
        @param steps: signed steps, -100 is 100 steps counterclockwise, 100 is 100 steps clockwise.
        @param fn: Stepping callback. Will be called on each step with contract:
                   fn(controller.getCurrentPosition(), targetPosition, accelerationStrategy.realDirection,
                      controller.multiprocessObserver)
        @param jobCompleteEventNamePrefix: "<prefix>steppingComplete[Advance|FinalStep]" events will be published on
                                           each job. "<prefix>steppingCompleteAdvance" eventInAdvanceSteps before done.
        @param maxPpsOverride: Allows for motor to run at a speed different from StepperMotor.getMaxPps(), ideally lower
        @param eventInAdvanceSteps: How many steps before done should "<prefix>steppingCompleteAdvance" be published.
                                    Your app might wat to perform tasks in preparation for final step.
        @return: a BlockingQueueWorker.Job representing this stepping job.
        """
        StepsSign = sign(steps)
        if StepsSign > 0:
            return self.microStepClockWise(abs(steps),
                                           fn=fn,
                                           jobCompleteEventNamePrefix=jobCompleteEventNamePrefix,
                                           maxPpsOverride=maxPpsOverride,
                                           eventInAdvanceSteps=eventInAdvanceSteps)
        elif StepsSign < 0:
            return self.microStepCounterClockWise(abs(steps),
                                                  fn=fn,
                                                  jobCompleteEventNamePrefix=jobCompleteEventNamePrefix,
                                                  maxPpsOverride=maxPpsOverride,
                                                  eventInAdvanceSteps=eventInAdvanceSteps)

    def microStepClockWise(self, steps, *, fn=None, jobCompleteEventNamePrefix="", maxPpsOverride=None,
                           eventInAdvanceSteps=10) -> BlockingQueueWorker.Job:
        """
        See signedSteps()
        1 step = 1 pulse, does not correct for microStepping. See microStepClockWise()
        @return: a BlockingQueueWorker.Job representing this stepping job.
        """
        return self.work(
            paramsList=[self.CW, steps, fn, jobCompleteEventNamePrefix, maxPpsOverride, eventInAdvanceSteps],
            block=True)

    def microStepCounterClockWise(self, steps, *, fn=None, jobCompleteEventNamePrefix="", maxPpsOverride=None,
                                  eventInAdvanceSteps=10) -> BlockingQueueWorker.Job:
        """
        See signedSteps()
        1 step = 1 pulse, does not correct for microStepping. See microStepCounterClockWise()
        @return: a BlockingQueueWorker.Job representing this stepping job.
        """
        return self.work(
            paramsList=[self.CCW, steps, fn, jobCompleteEventNamePrefix, maxPpsOverride, eventInAdvanceSteps],
            block=True)

    def setDirection(self, directionState):
        # Todo: update sharedMemory
        # Translating potential negative values to GPIO.LOW
        directionState = GPIO.HIGH if directionState == GPIO.HIGH else GPIO.LOW
        GPIO.output(self.directionGpioPin, directionState)
        self.currentDirection = directionState

    def pulseStart(self, stepRelativeToJobStart=None):
        """
        In most controllers this would mean set step pint to HIGH
        """
        GPIO.output(self.stepGpioPin, GPIO.HIGH)

    def pulseStop(self):
        """
        In most controllers this would mean set step pint to HIGH
        """
        GPIO.output(self.stepGpioPin, GPIO.LOW)

    @abstractmethod
    def setSleepMode(self, sleepOn=False):
        pass

    @abstractmethod
    def setEnableMode(self, enableOn=True):
        pass


class DRV8825MotorDriver(BipolarStepperMotorDriver):
    """
    Current adjustment formula: (I = 2 * vRef)

    Tested with SongHe (ghost company?) & HiLetgo (here:http://www.hiletgo.com/ProductDetail/1952516.html) brands for
    DRV8825 board.
    Cheap DRV8825 Stepper Motor Controller IC https://www.rototron.info/wp-content/uploads/PiStepper_DRV8825.pdf

    Interesting reads:
    https://www.rototron.info/raspberry-pi-stepper-motor-tutorial/
    https://lastminuteengineers.com/drv8825-stepper-motor-driver-arduino-tutorial/
    """
    CW = GPIO.HIGH  # Clockwise Rotation
    CCW = GPIO.LOW  # Counterclockwise Rotation

    # Mode pins are static, as they are shared among Turret motors.
    RESOLUTION = {'Full': (0, 0, 0),
                  'Half': (1, 0, 0),
                  '1/4': (0, 1, 0),
                  '1/8': (1, 1, 0),
                  '1/16': (0, 0, 1),
                  '1/32': (1, 0, 1)}

    # DRV8825 uses min 10 microseconds HIGH for STEP pin.
    # Raspberry Pi sleeps (in % of desired time) increasingly longer the lower the time goes.
    # So in actuality this will sleep about 20uS
    PULSE_TIME_MICROS = 20

    # DRV8825 Uses HIGH pulse on LOW background for STEP signal.
    PULSE_STATE = GPIO.HIGH

    def __init__(self, *,
                 stepperMotor: StepperMotor,
                 accelerationStrategy: AccelerationStrategy,
                 directionGpioPin,
                 stepGpioPin,
                 navigation,
                 # LOW = sleep mode; HIGH = chip active
                 sleepGpioPin=None,
                 stepsMode=BipolarStepperMotorDriver.DEFAULT_STEPPING_MODE,
                 useHoldingTorque=None,
                 modeGpioPins=None,
                 # LOW = enabled; HIGH chip disabled
                 enableGpioPin=None,
                 jobQueue=None,
                 workerName=None,
                 sharedMemory=None,
                 isProxy=False,
                 steppingCompleteEventName="steppingComplete",
                 jobCompletionObserver=None):
        """
        DRIVER PINOUT: [EN??] & [FLT] unused

        [EN]     1  *   *  9 [VMOT] DANGER, VOLTAGE MOTOR KEEP AWAY FROM RaspberryPi // VMOT bridged with capacitor to GND
        [MODE_0] 2  *   * 10 [GND] // MODE_0 -> modeGpioPins[0]
        [MODE_1] 3  *   * 11 [B2] // MODE_1 -> modeGpioPins[1] // B2 & B1 to same coil in motor
        [MODE_2] 4  *   * 12 [B1] // MODE_2 -> modeGpioPins[2]
        [RST]    5  *   * 13 [A1] // A1 & A2 to same coil in motor  // RST & SLP to [3v3 Power] in Pi
        [SLP]    6  *   * 14 [A2]
        [STP]    7  *   * 15 [FLT] // STP -> stepGpioPin in Raspberry
        [DIR]    8  *   * 16 [GND] // DIR -> directionGpioPin in Raspberry // GND to GND in pi

        @param stepperMotor:
        @param accelerationStrategy:
        @param directionGpioPin:
        @param stepGpioPin:
        @param navigation:
        @param sleepGpioPin:
        @param stepsMode: [STP]
        @param modeGpioPins: [MODE_0..2]
        @param enableGpioPin: [EN] LOW = enabled; HIGH chip disabled
        """
        if stepsMode is None:
            # Builders might set this to None.
            stepsMode = BipolarStepperMotorDriver.DEFAULT_STEPPING_MODE,

        super().__init__(stepperMotor=stepperMotor,
                         accelerationStrategy=accelerationStrategy,
                         navigation=navigation,
                         directionGpioPin=directionGpioPin,
                         stepGpioPin=stepGpioPin,
                         sleepGpioPin=sleepGpioPin,
                         enableGpioPin=enableGpioPin,
                         useHoldingTorque=useHoldingTorque,
                         stepsMode=stepsMode,
                         modeGpioPins=modeGpioPins,
                         steppingCompleteEventName=steppingCompleteEventName,
                         jobQueue=jobQueue,
                         workerName=workerName,
                         sharedMemory=sharedMemory,
                         isProxy=isProxy,
                         jobCompletionObserver=jobCompletionObserver)

        if not(stepsMode == self.DEFAULT_STEPPING_MODE or modeGpioPins):
            tprint(f"Warning: {workerName} needs modeGpioPins set unless it's set by hardware."
                   "stepsMode does not match default stepping Mode")

    def setSleepMode(self, sleepOn=False):
        """
        Active LOW, so if sleepOn True, pin is LOW. if sleepOn False, pin is HIGH.
        @param sleepOn: True puts chip to sleep. reducing power consumption to a minimum. You can use this to save
        power, especially when the motor is not in use.
        """
        if self.sleepGpioPin is None:
            return
        state = GPIO.LOW if sleepOn else GPIO.HIGH
        GPIO.output(self.sleepGpioPin, state)

    def setEnableMode(self, enableOn=True):
        """
        Active LOW, so if enableOn True, pin is LOW. if enableOn False, pin is HIGH.
        @param enableOn: enable or disable the driver chip.  This pin is particularly useful when implementing an
        emergency stop or shutdown system. Although some integrated circuits like RPI Hats will use this for sleep.
        """
        if self.enableGpioPin is None:
            return
        state = GPIO.LOW if enableOn else GPIO.HIGH
        GPIO.output(self.enableGpioPin, state)


# class TB6560(BipolarStepperMotorDriver):
#     pass


class TMC2209StandaloneMotorDriver(BipolarStepperMotorDriver):
    """
    When a GPIO pin is set to LOW, it typically sinks current, effectively providing a path to ground. That's why EN_PIN
    which is active LOW needs to be connected to GPIO or to GND.
    Should work with drop-in replacements as well:
    TMC2208 / TMC2209 / TMC2226
    No UART support
    Current adjustment formula: (I(rms) = Vref/sqrt(2))
    Peak Current: I(peak) = I(rms)/sqrt(2) = Vref
    Vref range：0.2V – 2.2V
    Factory default Vref(same as Peak Current): 1.2V ±0.1V
    Factory default RMS current：~0.9A

    DRIVER PINOUT: [EN??] & [FLT] unused

    [EN]     1  *   *  9 [VMOT] [EN] Active LOW.
                                DANGER, VOLTAGE MOTOR KEEP AWAY FROM RaspberryPi // VMOT bridged with capacitor to GND
    [MODE_1] 2  *   * 10 [GND] // MODE_0 -> modeGpioPins[0]
    [MODE_2] 3  *   * 11 [A2]  // MODE_1 -> modeGpioPins[1]
    [PDN]    4  *   * 12 [A1]  // A1 & A2 to same coil in motor
    [PDN]    5  *   * 13 [B1]  // B2 & B1 to same coil in motor
    [CLK]    6  *   * 14 [B2]
    [STP]    7  *   * 15 [VDD] // STP -> stepGpioPin in Raspberry
    [DIR]    8  *   * 16 [GND] // DIR -> directionGpioPin in Raspberry // GND to GND in pi

    https://www.youtube.com/watch?v=VcyGzXIZm58
    Readings:
        https://bttwiki.com/TMC2209.html#trinamic-driver-current-formula
        https://klipper.discourse.group/t/tmc-uart-wiring-and-pin-variations/11391
        https://forum.arduino.cc/t/tmcstepper-arduino-tmc2209/956036
        https://www.digikey.cz/htmldatasheets/production/2042923/0/0/1/mikroe-2624.html#pf3c
        [Not Supported]:
            UART https://forum.arduino.cc/t/using-a-tmc2209-silent-stepper-motor-driver-with-an-arduino/666992/104?page=6
            https://www.elinux.org/Serial_port_programming

    Tested on BigTreeTech TMC2209 v1.3 board:
    BigTreeTechnologies UART (“BTT”) found here:
        https://www.aliexpress.com/item/1005004656945214.html (Not sure this one is v1.3)
        https://www.amazon.com/gp/product/B07ZPYKL46

    https://learn.watterott.com/silentstepstick/pinconfig/tmc2209/#step-configuration
    4 Microstepping modes, in either of stealthChop, spreadCycle (more torque at high speed).
    Micro stepping pins:
        MS1,MS2,SPREAD/MS3, StepMode, Mode
        0   0   0            1/8        stealthChop
        1   0   0            1/2        stealthChop
        0   1   0            1/4        stealthChop
        1   1   0            1/16       stealthChop
        0   0   1            1/8        spreadCycle
        1   0   1            1/2        spreadCycle
        0   1   1            1/4        spreadCycle
        1   1   1            1/16       spreadCycle

    """

    CW = GPIO.HIGH  # Clockwise Rotation
    CCW = GPIO.LOW  # Counterclockwise Rotation

    # Mode pins are static, as they are shared among Turret motors.
    # TODO: what combo is Full? if 0 0 => 1/8? is it that not connected != LOW as it happens with EN_PIN?
    # TODO: super uses 3 pins. make compatible.
    RESOLUTION = {'1/8':  (0, 0),
                  '1/32':  (1, 0),
                  '1/64':  (0, 1),
                  '1/16':  (1, 1)}

    DEFAULT_STEPPING_MODE = '1/8'

    # DRV8825 uses min 10 microseconds HIGH for STEP pin.
    # Raspberry Pi sleeps (in % of desired time) increasingly longer the lower the time goes.
    # So in actuality this will sleep about 20uS
    PULSE_TIME_MICROS = 20

    # DRV8825 Uses HIGH pulse on LOW background for STEP signal.
    PULSE_STATE = GPIO.HIGH

    CHOP_MODE_STEALTH = GPIO.LOW
    CHOP_MODE_SPREAD_CYCLE = GPIO.HIGH

    def __init__(self, *,
                 stepperMotor: StepperMotor,
                 accelerationStrategy: AccelerationStrategy,
                 directionGpioPin,
                 stepGpioPin,
                 navigation,
                 # LOW = enabled; HIGH chip disabled
                 enableGpioPin=None,
                 useHoldingTorque=None,
                 # LOW = sleep mode; HIGH = chip active
                 stepsMode=DEFAULT_STEPPING_MODE,
                 modeGpioPins=None,
                 spreadGpioPin=None,
                 diagGPIOPin=None,
                 autoResetOnError=False,
                 homeDirection=-1,
                 indexGPIOPin=None,
                 # Vref range：0.2V – 2.2V
                 vRefAnalogPin=None,
                 stepsPerIndexPulse=4,
                 jobQueue=None,
                 workerName=None,
                 sharedMemory=None,
                 isProxy=False,
                 steppingCompleteEventName="steppingComplete",
                 jobCompletionObserver=None):

        if stepsMode is None:
            # Builders might set this to None.
            stepsMode = self.DEFAULT_STEPPING_MODE,
        elif not (stepsMode == self.DEFAULT_STEPPING_MODE or modeGpioPins):
            tprint(f"Warning: {workerName} needs modeGpioPins set unless it's set by hardware."
                   "stepsMode does not match default stepping Mode")

        super().__init__(stepperMotor=stepperMotor,
                         accelerationStrategy=accelerationStrategy,
                         navigation=navigation,
                         directionGpioPin=directionGpioPin,
                         stepGpioPin=stepGpioPin,
                         enableGpioPin=enableGpioPin,
                         useHoldingTorque=useHoldingTorque,
                         stepsMode=stepsMode,
                         modeGpioPins=modeGpioPins,
                         steppingCompleteEventName=steppingCompleteEventName,
                         jobQueue=jobQueue,
                         workerName=workerName,
                         sharedMemory=sharedMemory,
                         isProxy=isProxy,
                         jobCompletionObserver=jobCompletionObserver)

        self.spreadGpioPin = spreadGpioPin

        # Not so Soon: https://swharden.com/blog/2016-09-28-generating-analog-voltages-with-the-raspberry-pi/
        # https://www.youtube.com/watch?v=iwzXh2V1SP4&t=20s
        self.vRefAnalogPin = vRefAnalogPin

        # Inputs
        # Diagnostic and StallGuard output. High level upon stall detection or driver error. Reset error condition by
        # ENN=high.
        self.diagGPIOPin = diagGPIOPin
        self.isInErrorState = False
        self.autoResetOnError = autoResetOnError
        self.homeDirection = homeDirection

        # Configurable index output. Provides index pulse. one pulse each 4 Full steps by default.
        self.indexGPIOPin = indexGPIOPin
        self.indexReportedPosition = 0
        self.stepsPerIndexPulse = stepsPerIndexPulse

        self.init()

    def init(self):
        if self.spreadGpioPin:
            GPIO.setup(self.spreadGpioPin, GPIO.OUT)
            self.setChopper(self.CHOP_MODE_STEALTH)


        if self.diagGPIOPin:
            GPIO.setup(self.diagGPIOPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.add_event_detect(self.diagGPIOPin,
                                  GPIO.FALLING,
                                  callback=self.readDiagnosticPin,
                                  bouncetime=30)
        if self.indexGPIOPin:
            GPIO.setup(self.indexGPIOPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.add_event_detect(self.indexGPIOPin,
                                  GPIO.FALLING,
                                  callback=self.readIndexPin,
                                  bouncetime=30)

        # Default to stealthChop

    def readDiagnosticPin(self):
        """
        Chip error Or (most likely stall detected, Assuming homing when resetErrorState() set position to zero.)
        @return:
        """
        self.isInErrorState = True
        tprint(f"{self.workerName} Reported error or Stalling!")
        if self.autoResetOnError and self.currentDirection == self.homeDirection:
            # Vague attempt at sensorless-homing.
            self.resetErrorState()

    def resetErrorState(self):
        direction = self.homeDirection
        steps = 1
        fn = None
        jobCompleteEventNamePrefix = "ResettingErrorState"
        job = self.work([direction, steps, fn, jobCompleteEventNamePrefix])
        tprint(f"Sensorless homing {self.workerName}")

        # Block until done.
        job.result()
        self.isInErrorState = False
        self.indexReportedPosition = 0
        self.currentPosition = 0
        self.toggleEnabled()

    def toggleEnabled(self):
        # EN is active LOW
        # This should reset index internal count too.
        GPIO.output(self.enableGpioPin, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(self.enableGpioPin, GPIO.LOW)

    def readIndexPin(self):
        self.indexReportedPosition += int(self.currentDirection * self.stepsPerIndexPulse)

    def setPeakCurrent(self, current):
        assert current < 2.2 and current > 0.2
        # https://swharden.com/blog/2016-09-28-generating-analog-voltages-with-the-raspberry-pi/
        pass

    def setRmsCurrent(self, current):
        # Theoretical max: 2.2 / math.sqrt(2) => 1.5556349186104046
        assert current < 1.551 and current > 0.2
        # https://swharden.com/blog/2016-09-28-generating-analog-voltages-with-the-raspberry-pi/
        pass

    def setChopper(self, state):
        assert self.spreadGpioPin and state in (self.CHOP_MODE_STEALTH, self.CHOP_MODE_SPREAD_CYCLE)
        GPIO.output(self.spreadGpioPin, state)

    def setSleepMode(self, sleepOn=False):
        raise RuntimeError("TMC2209 does not hae a sleep Pin, Use EN_PIN `enableGpioPin`")

    def setEnableMode(self, enableOn=True):
        """
        Active LOW, so if enableOn True, pin is LOW. if enableOn False, pin is HIGH.
        @param enableOn: enable or disable the driver chip.  This pin is particularly useful when implementing an
        emergency stop or shutdown system. Although some integrated circuits like RPI Hats will use this for sleep.
        """
        if self.enableGpioPin is None:
            return
        state = GPIO.LOW if enableOn else GPIO.HIGH
        GPIO.output(self.enableGpioPin, state)

    def _operateStepper(self, direction, steps, fn=None, jobCompleteEventNamePrefix="", maxStepsPerSecondOverride=None,
                        eventInAdvanceSteps=10):
        super()._operateStepper(direction, steps, fn, jobCompleteEventNamePrefix, maxStepsPerSecondOverride, eventInAdvanceSteps)
        self.checkPositionVsIndexReportedPosition()

    def checkPositionVsIndexReportedPosition(self):
        if self.indexGPIOPin and abs(self.currentPosition - self.indexReportedPosition) > 3:
            # Todo: This will fail unless we correct by microsteps as currentPosition counts int regardless of mode
            raise RuntimeError("Lost track of position, chip index and currentPosition dicerged")
