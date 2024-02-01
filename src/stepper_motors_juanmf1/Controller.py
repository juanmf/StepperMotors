from abc import abstractmethod
from concurrent.futures import Future
import time

from RPi import GPIO
import pigpio

from stepper_motors_juanmf1.AccelerationStrategy import AccelerationStrategy
from stepper_motors_juanmf1.StepperMotor import StepperMotor
from stepper_motors_juanmf1.BlockingQueueWorker import BlockingQueueWorker
from stepper_motors_juanmf1.myMath import sign
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint


# Todo: migrate to https://pypi.org/project/python-periphery/
class BipolarStepperMotorDriver(BlockingQueueWorker):
    INSTANCES_COUNT = 0
    """
    Bipolar stepper motor driver abstract implementation.
    Uses a dedicated thread to handle pulses to driver hardware in a non-blocking fashion.
    Uses StepperMotor specs to keep timing within acceptable ranges and can be configured wit different
    acceleration strategies.
    Accepts Navigation modes to enable interruptions or keep head down until a stepping job is 100% done.
    Client code can send (non-blocking) instructions as :func:`~stepCounterClockWise` or
    :func:`~stepClockWise` passing a callable to update position as the motor moves.
    """
    INITIALIZED = False
    CW = None  # Clockwise Rotation
    CCW = None  # Counterclockwise Rotation
    # Mode pins are static, as they are shared among Turret motors.
    RESOLUTION = None

    PULSE_TIME_MICROS = None
    """
    Many sites will show 50% duty cycle from controller code. Drivers actually take short signal pulses.
    """

    SIGNED_STEPS_CALLABLES = {}
    """
    steps in direction aware functions like :func:`~stepClockWise` and :func:`~stepCounterClockWise` must be positive.
    To alleviate client code burden, subclasses must define an association between signed steps and CW vs CCW.
    :func:`~signedSteps` will pick the right callable (lambda to either :func:`~stepClockWise` or 
    :func:`~stepCounterClockWise`) and send `abs(steps)` to it, consuming sign in the process.
    see :func:`DRV8825MotorDriver.SIGNED_STEPS_CALLABLES` for an example.
    Basically it should be 
    `{-1: lambda _steps, _fn: self.stepCounterClockWise(_steps, _fn),
      1: lambda _steps, _fn: self.stepClockWise(_steps, _fn)}` or otherwise. 
    """

    PULSE_STATE = None
    """
    Only tested with DRV8825 but LOW pulse is also possible on HIGH background.
    """

    pi = pigpio.pi()
    """
    Connect to pigpiod daemon
    # Todo: check that "-s" still applies. think it was important for PWM(), not used now.
    Important: use -s option to mode 10 $> sudo pigpiod -s 10 -t 0 (or find the right -s <number>)
    Go back to steps per ramp level waveforms if steps are missed.
    "if you can control multiple stepper motors.The short answer is yes but it depends on the type of
    precision and timing you need.For example CNC or 3D printing would probably require a dedicated controller.
    Please take a look at PyCNC. Otherwise, you are only limited by the number of available GPIO pins.
    """

    def __init__(self,
                 stepperMotor: StepperMotor,
                 accelerationStrategy: AccelerationStrategy,
                 directionGpioPin,
                 stepGpioPin,
                 navigation,
                 sleepGpioPin=None,
                 stepsMode="Full",
                 modeGpioPins=None,
                 enableGpioPin=None,
                 workerName=None,
                 useHoldingTorque=None):
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
        super().__init__(self._operateStepper, jobQueueMaxSize=2,
                         workerName=f"{self.__class__.__name__}_{BipolarStepperMotorDriver.INSTANCES_COUNT}_"
                         if workerName is None else workerName)
        BipolarStepperMotorDriver.INSTANCES_COUNT += 1
        self.stepperMotor = stepperMotor
        self.accelerationStrategy = accelerationStrategy
        self.enableGpioPin = enableGpioPin
        self.modeGpioPins = modeGpioPins  # Microstep Resolution GPIO Pins
        self.stepsMode = stepsMode
        self.sleepGpioPin = sleepGpioPin
        self.stepGpioPin = stepGpioPin
        self.directionGpioPin = directionGpioPin
        # Tracks current either by position for position based movement (DynamicNavigation, interruptible)
        # or by fixed number of steps (StaticNavigation, uninterruptible).
        self.navigation = navigation
        self.currentPosition = 0
        self.currentDirection = None

        self.isRunning = False

        # False means no current when no stepping. True sends current for holding Torque.
        # sleepGpioPin and enableGpioPin can be set by hardware, if provided a GPIO they are used to save power
        # unless overriden by useHoldingTorque
        self.useHoldingTorque = useHoldingTorque if useHoldingTorque is not None else sleepGpioPin or enableGpioPin

        self._initGpio(stepsMode)

        # Todo: libc.usleep fails on my RPI. see `self.usleep()`
        # self.libc = ctypes.CDLL('libc.so.6')

    def _initGpio(self, stepsMode):
        self._oneTimeInit(stepsMode)

        GPIO.setup(self.directionGpioPin, GPIO.OUT)
        self.setDirection(GPIO.LOW)

        GPIO.setup(self.stepGpioPin, GPIO.OUT)
        GPIO.output(self.stepGpioPin, GPIO.LOW)

        if self.enableGpioPin is not None:
            GPIO.setup(self.enableGpioPin, GPIO.OUT)
            self.setEnableMode()

    def _oneTimeInit(self, stepsMode):
        if not BipolarStepperMotorDriver.INITIALIZED:
            GPIO.setmode(GPIO.BCM)

        # Todo: check we can call setup before GPIO.setmode(GPIO.BCM)
        if self.modeGpioPins is not None:
            GPIO.setup(self.modeGpioPins, GPIO.OUT)

        if BipolarStepperMotorDriver.INITIALIZED:
            return

        BipolarStepperMotorDriver.INITIALIZED = True
        if self.modeGpioPins is None:
            return

        for i in range(3):
            self.pi.write(self.modeGpioPins[i], self.RESOLUTION[stepsMode][i])

    def _operateStepper(self, direction, steps, fn):
        """
        Positive change in position is in clockwise direction, negative is counter clock wise.
        callable signature is expected as fn(currentPosition, targetPosition, realDirection)
        (realDirection from AccelerationStrategy)
        interruptibility is determined by Navigation instance injected at construction time.
        :param direction: self.CW or self.CCW
        :param steps: Number of steps, as we rely on direction, an error is raised if negative values are sent.
        :param fn: callable to execute after each step. Contract:
            fn(controller.currentPosition, targetPosition, accelerationStrategy.realDirection)
        """
        if steps < 0:
            raise RuntimeError("Can't handle negative steps. Use direction (self.CW or self.CCW) & steps > 0 properly.")
        signedDirection = 1 if direction == self.CW else -1
        targetPosition = self.currentPosition + (signedDirection * steps)
        self.isRunning = True

        if not self.useHoldingTorque:
            self.setSleepMode(sleepOn=False)
            self.setEnableMode(enableOn=True)

        self.navigation.go(self, targetPosition, self.accelerationStrategy, fn, self.isInterrupted)
        if self.navigation.isInterruptible() and self.isInterrupted():
            return

        if not self.useHoldingTorque:
            self.setSleepMode(sleepOn=True)
            self.setEnableMode(enableOn=False)

        self.isRunning = False

    def isInterrupted(self):
        return self.hasQueuedJobs()

    def signedSteps(self, steps, fn):
        return self.SIGNED_STEPS_CALLABLES.get(sign(steps), lambda _steps, _fn: None)(abs(steps), fn)

    def stepClockWise(self, steps, fn):
        return self.work([self.CW, steps, fn], block=True)

    def stepCounterClockWise(self, steps, fn):
        return self.work([self.CCW, steps, fn], block=True)

    def setDirection(self, directionState):
        # Translating potential negative values to GPIO.LOW
        directionState = GPIO.HIGH if directionState == GPIO.HIGH else GPIO.LOW
        tprint(f"Setting direction pin {self.directionGpioPin} {directionState}.")
        GPIO.output(self.directionGpioPin, directionState)
        self.currentDirection = directionState

    @staticmethod
    def usleep(micros):
        """
        Sleep time strategies tested on RPI 4B
        """
        if micros < 2000:
            # Todo: if libc.usleep works, use active wait under 300uS
            # Active wait
            # Shows overhead of ~5-10 uS
            # Driver's pulse time is generally much shorter than step period, i.e. low duty-cycle.
            nanos = (micros * 1000)
            start = time.time_ns()
            while True:
                end = time.time_ns()
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

    @abstractmethod
    def setSleepMode(self, sleepOn=False):
        pass

    @abstractmethod
    def setEnableMode(self, enableOn=True):
        pass


class DRV8825MotorDriver(BipolarStepperMotorDriver):
    """
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
    PULSE_TIME_MICROS = 50
    SIGNED_STEPS_CALLABLES = {}
    # DRV8825 Uses HIGH pulse on LOW background for STEP signal.
    PULSE_STATE = GPIO.HIGH


    def __init__(self,
                 stepperMotor: StepperMotor,
                 accelerationStrategy: AccelerationStrategy,
                 directionGpioPin,
                 stepGpioPin,
                 navigation,
                 # LOW = sleep mode; HIGH = chip active
                 sleepGpioPin=None,
                 stepsMode="Full",
                 modeGpioPins=None,
                 # LOW = enabled; HIGH chip disabled
                 enableGpioPin=None):
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
        super().__init__(stepperMotor, accelerationStrategy, directionGpioPin, stepGpioPin, navigation,
                         sleepGpioPin=sleepGpioPin, stepsMode=stepsMode, modeGpioPins=modeGpioPins,
                         enableGpioPin=enableGpioPin)
        self.SIGNED_STEPS_CALLABLES = {-1: lambda _steps, _fn: self.stepCounterClockWise(_steps, _fn),
                                        1: lambda _steps, _fn: self.stepClockWise(_steps, _fn)}
        tprint(f"sleepPin: {self.sleepGpioPin}.")
        tprint(f"useHOldingToruqe: {self.useHoldingTorque}")

    def setSleepMode(self, sleepOn=False):
        """
        Active LOW, so if sleepOn True, pin is LOW. if sleepOn False, pin is HIGH.
        @param sleepOn: True puts chip to sleep. reducing power consumption to a minimum. You can use this to save
        power, especially when the motor is not in use.
        """
        tprint(f"self.sleepGpioPin is None {self.sleepGpioPin is None}")
        if self.sleepGpioPin is None:
            return
        state = GPIO.LOW if sleepOn else GPIO.HIGH
        tprint(f"Setting Sleep pin {self.sleepGpioPin} to {state}")
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
        tprint(f"Setting Enabled pin {self.enableGpioPin} to {state}")
        GPIO.output(self.enableGpioPin, state)
