import queue

from stepper_motors_juanmf1.AccelerationStrategy import AccelerationStrategy
from stepper_motors_juanmf1.StepperMotor import StepperMotor
from stepper_motors_juanmf1.BlockingQueueWorker import BlockingQueueWorker
from RPi import GPIO
import pigpio
import time


# Todo: migrate to https://pypi.org/project/python-periphery/
class BipolarStepperMotorDriver(BlockingQueueWorker):
    INITIALIZED = False
    CW = None  # Clockwise Rotation
    CCW = None  # Counterclockwise Rotation
    # Mode pins are static, as they are shared among Turret motors.
    RESOLUTION = None

    # Many sites will show 50% duty cycle from controller code. Drivers actually take short signal pulses.
    PULSE_TIME_MICROS = None

    # Only tested with DRV8825 but LOW pulse is also possible on HIGH background.
    PULSE_STATE = None

    # Connect to pigpiod daemon
    # TODO: use -s option to mode 10 $> sudo pigpiod -s 10 -t 0
    #    Go back to steps per ramp level waveforms if steps are missed.
    #    "if you can control multiple stepper motors.The short answer is yes but it depends on the type of
    #    precision and timing you need.For example CNC or 3D printing would probably require a dedicated controller.
    #    Please take a look at PyCNC.Otherwise, you are only limited by the number of available GPIO pins.
    pi = pigpio.pi()

    # Multiple drivers could share the same mode pins (assuming current supply from pin is enough,
    # and the drivers' mode pins are bridged same to same)
    # Raspberry board pins layout:
    # [GPIOn] n * * m [GPIOm]; n,m = physical board pin order. * = physical pin. [GPIOm],[GPIOm] = GPIO pin n,m
    #
    # [3v3 Power] 1  * *  2 [5v Power] // 3v3 Power to controller IC. To Reset and Sleep pins.
    # [GPIO_2]    3  * *  4 [5v Power]
    # [GPIO_3]    5  * *  6 [GND]
    # [GPIO_4]    7  * *  8 [GPIO_14] // default modeGpioPins[0]
    # [GND]       9  * * 10 [GPIO_15] // default modeGpioPins[1]
    # [GPIO_17]   11 * * 12 [GPIO_18] // default modeGpioPins[2]
    # [GPIO_27]   13 * * 14 [GND]
    # [GPIO_22]   15 * * 16 [GPIO_23]
    # [3v3 Power] 17 * * 18 [GPIO_24] // 3v3 Power to controller IC. To Reset and Sleep pins.
    # [GPIO_10]   19 * * 20 [GND]
    # [GPIO_9]    21 * * 22 [GPIO_25]
    # [GPIO_11]   23 * * 24 [GPIO_8]
    # [GND]       25 * * 26 [GPIO_7]
    # [GPIO_0]    27 * * 28 [GPIO_1]
    # [GPIO_5]    29 * * 30 [GND]
    # [GPIO_6]    31 * * 32 [GPIO_12]
    # [GPIO_13]   33 * * 34 [GND]
    # [GPIO_19]   35 * * 36 [GPIO_16]
    # [GPIO_26]   37 * * 38 [GPIO_20]
    # [GND]       39 * * 40 [GPIO_21]
    #
    def __init__(self,
                 stepperMotor: StepperMotor,
                 accelerationStrategy: AccelerationStrategy,
                 directionGpioPin,
                 stepGpioPin,
                 navigation,
                 sleepGpioPin=None,
                 stepsMode="Full",
                 modeGpioPins=None,
                 emergencyStopGpioPin=None):
        super().__init__(self._operateStepper)
        # Tracks current position for position based movement instead of fixed number of steps.
        self.navigation = navigation
        self.currentPosition = 0
        # Inverse of Speed, it will be somewhere between sleepTime and sleepTime / 10 and exponentially reach sleepTime
        # in 5 steps ((sleepTime / 10) * 1.6 ^ 5).
        self.accelerationStrategy = accelerationStrategy
        # Todo: libc.usleep fails.
        # self.libc = ctypes.CDLL('libc.so.6')
        self.emergencyStopGpioPin = emergencyStopGpioPin
        self.modeGpioPins = modeGpioPins  # Microstep Resolution GPIO Pins
        self.stepsMode = stepsMode
        self.sleepGpioPin = sleepGpioPin
        self.stepGpioPin = stepGpioPin
        self.directionGpioPin = directionGpioPin
        self.currentDirection = GPIO.LOW
        self.stepperMotor = stepperMotor

        self.isRunning = False
        # False means no current when no stepping. True sends current for holding Torque.
        self.useHoldingTorque = True  # sleepGpioPin is None

        self.emergencyStopGpioPin = emergencyStopGpioPin
        self._initGpio(stepsMode)

        # Dedicated thread will move the turret, allowing for changes in direction between every step
        self.preemptionEnabled = True

    def _initGpio(self, stepsMode):
        self._oneTimeInit(stepsMode)

        GPIO.setup(self.directionGpioPin, GPIO.OUT)
        self.setDirection(GPIO.LOW)

        GPIO.setup(self.stepGpioPin, GPIO.OUT)
        GPIO.output(self.stepGpioPin, GPIO.LOW)

        if self.emergencyStopGpioPin is not None:
            GPIO.setup(self.emergencyStopGpioPin, GPIO.OUT)
            GPIO.output(self.emergencyStopGpioPin, GPIO.LOW)

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

    # Positive change in position is in clockwise direction. (double check)
    # fn() gets passed current position, targetPosition and realDirection (from Accelerator)
    def _operateStepper(self, direction, steps, fn):
        signedDirection = 1 if direction == self.CW else -1
        targetPosition = self.currentPosition + (signedDirection * steps)
        self.isRunning = True
        self.navigation.go(self, targetPosition, self.accelerationStrategy, fn, self.isInterrupted)

        if self.navigation.isInterruptible() and self.isInterrupted():
            return

        if not self.useHoldingTorque:
            print(f"Setting Sleep pin {self.sleepGpioPin} LOW (sleep).")
            GPIO.output(self.sleepGpioPin, GPIO.LOW)
        self.isRunning = False

    def isInterrupted(self):
        return self.preemptionEnabled and self.jobQueue.qsize() > 0

    def stepClockWise(self, steps, fn):
        self.jobQueue.put([self.CW, steps, fn], block=True)

    def stepCounterClockWise(self, steps, fn):
        self.jobQueue.put([self.CCW, steps, fn], block=True)

    def setDirection(self, directionState):
        # Translating potential negative values to GPIO.LOW
        directionState = GPIO.HIGH if directionState == GPIO.HIGH else GPIO.LOW
        print(f"Setting direction pin {self.directionGpioPin} {directionState}.")
        GPIO.output(self.directionGpioPin, directionState)
        self.currentDirection = directionState

    # Tested on Raspberry Pi 4B
    def usleep(self, micros):
        if micros < 2000:  # 300:
            # Shows overhead of ~5-10 uS
            # Driver's pulse time is generally much shorter than step period, i.e. low duty-cycle.
            nanos = (micros * 1000)
            start = time.time_ns()
            # count = 0
            while True:
                # count += 1
                end = time.time_ns()
                # print(f"elapsed time {end - start} in {count} iterations.")
                if end - start >= nanos:
                    break
        # elif micros < 2000:
        #     # Shows overhead of ~110 uS
        #     # self.libc.usleep(micros)  # Todo: is hanging up.
        #     time.sleep(micros / 1_000_000)

        else:
            time.sleep(micros / 1_000_000)


# Tested with Pololu DRV8825
class DRV8825MotorDriver(BipolarStepperMotorDriver):
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

    # DRV8825 Uses HIGH pulse on LOW background for STEP signal.
    PULSE_STATE = GPIO.HIGH

    # DRIVER PINOUT: [EN??] & [FLT] unused
    #
    # [EN??]   1  *   *  9 [VMOT] DANGER, VOLTAGE MOTOR KEEP AWAY FROM RaspberryPi // VMOT bridged with capacitor to 10 GND
    # [MODE_0] 2  *   * 10 [GND] // MODE_0 -> modeGpioPins[0]
    # [MODE_1] 3  *   * 11 [B2] // MODE_1 -> modeGpioPins[1] // B2 & B1 to same coil in motor
    # [MODE_2] 4  *   * 12 [B1] // MODE_2 -> modeGpioPins[2]
    # [RST]    5  *   * 13 [A1] // A1 & A2 to same coil in motor  // RST & SLP to [3v3 Power] in Pi
    # [SLP]    6  *   * 14 [A2]
    # [STP]    7  *   * 15 [FLT] // STP -> stepGpioPin in Raspberry
    # [DIR]    8  *   * 16 [GND] // DIR -> directionGpioPin in Raspberry // GND to GND in pi
    def __init__(self,
                 stepperMotor: StepperMotor,
                 accelerationStrategy: AccelerationStrategy,
                 directionGpioPin,
                 stepGpioPin,
                 navigation,
                 sleepGpioPin=None,
                 stepsMode="Full",
                 modeGpioPins=None,
                 emergencyStopGpioPin=None):
        super().__init__(stepperMotor, accelerationStrategy, directionGpioPin, stepGpioPin, navigation,
                         sleepGpioPin=sleepGpioPin, stepsMode=stepsMode, modeGpioPins=modeGpioPins,
                         emergencyStopGpioPin=emergencyStopGpioPin)
