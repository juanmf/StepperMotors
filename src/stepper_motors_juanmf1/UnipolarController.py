from stepper_motors_juanmf1.AccelerationStrategy import AccelerationStrategy
from stepper_motors_juanmf1.Controller import BipolarStepperMotorDriver
from RPi import GPIO

from stepper_motors_juanmf1.StepperMotor import StepperMotor


class UnipolarMotorDriver(BipolarStepperMotorDriver):

    def __init__(self, *,
                 stepperMotor: StepperMotor,
                 accelerationStrategy: AccelerationStrategy,
                 directionGpioPin,
                 # For Unipolar drivers it's a list of 2-4 pins.
                 stepGpioPin,
                 navigation,
                 # LOW = enabled; HIGH chip disabled
                 enableGpioPin=None,
                 # LOW = sleep mode; HIGH = chip active, Works for StandBy in TB6612 for instance
                 sleepGpioPin=None,
                 useHoldingTorque=None,
                 stepsMode=BipolarStepperMotorDriver.DEFAULT_STEPPING_MODE,
                 jobQueue=None,
                 sharedMemory=None,
                 isProxy=False,
                 steppingCompleteEventName="steppingComplete",
                 jobCompletionObserver=None,
                 workerName=None):

        if stepsMode is None:
            # Builders might set this to None.
            stepsMode = self.DEFAULT_STEPPING_MODE,
        else:
            assert stepsMode in UnipolarMotorDriver.Sequence.SUPPORTED_MICROSTEPS
        super().__init__(stepperMotor=stepperMotor,
                         accelerationStrategy=accelerationStrategy,
                         navigation=navigation,
                         directionGpioPin=directionGpioPin,
                         stepGpioPin=stepGpioPin,
                         sleepGpioPin=sleepGpioPin,
                         enableGpioPin=enableGpioPin,
                         useHoldingTorque=useHoldingTorque,
                         stepsMode=stepsMode,
                         steppingCompleteEventName=steppingCompleteEventName,
                         jobQueue=jobQueue,
                         workerName=workerName,
                         sharedMemory=sharedMemory,
                         isProxy=isProxy,
                         jobCompletionObserver=jobCompletionObserver)

        self.sequence = UnipolarMotorDriver.Sequence(UnipolarMotorDriver.Sequence.FULL)

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

    def pulseStart(self, stepRelativeToJobStart=None):
        GPIO.output(
            self.stepGpioPin,
            self.sequence.getStepSequence(stepRelativeToJobStart))

    def pulseStop(self):
        # Not in Unipolar
        pass

    class Sequence:

        SUPPORTED_MICROSTEPS = ["Full", "Half", "1/2"]

        FULL = "4p-Full"
        HALF = "4p-Half"
        FULL_2PIN = "2p-Full"
        FULL_3PIN = "3p-Full"
        HALF_3PIN = "3p-Half"

        SUPPORTED_TYPES = [FULL, FULL_2PIN, HALF, FULL_3PIN, HALF_3PIN]

        STEP_SEQUENCE = {
            FULL_2PIN: [(GPIO.LOW, GPIO.HIGH),   # 01
                        (GPIO.HIGH, GPIO.HIGH),  # 11
                        (GPIO.HIGH, GPIO.LOW)],  # 10

            FULL:  [(GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW),  # 1010
                    (GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW),  # 0110
                    (GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH),  # 0101
                    (GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH)], # 1001

            HALF: [(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.LOW),     # 1000
                   (GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW),   # 1010
                   (GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.LOW),    # 0010
                   (GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW),   # 0110
                   (GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.LOW),    # 0100
                   (GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH),   # 0101
                   (GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.HIGH),    # 0001
                   (GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH)],  # 1001

            FULL_3PIN: [(GPIO.HIGH, GPIO.LOW, GPIO.LOW),   # 100
                        (GPIO.LOW, GPIO.LOW, GPIO.HIGH),   # 001
                        (GPIO.LOW, GPIO.HIGH, GPIO.LOW)],  # 010

            HALF_3PIN: [(GPIO.HIGH, GPIO.LOW, GPIO.LOW),    # 100
                        (GPIO.HIGH, GPIO.LOW, GPIO.HIGH),   # 101
                        (GPIO.LOW, GPIO.LOW, GPIO.HIGH),    # 001
                        (GPIO.LOW, GPIO.HIGH, GPIO.HIGH),   # 011
                        (GPIO.LOW, GPIO.HIGH, GPIO.LOW),    # 010
                        (GPIO.HIGH, GPIO.HIGH, GPIO.LOW)]}  # 110

        # https://github.com/d235j/AccelStepper/blob/master/AccelStepper/AccelStepper.cpp
        def __init__(self, sequenceType):
            assert sequenceType in self.SUPPORTED_TYPES
            self.type = sequenceType

        def getStepSequence(self, stepRelativeToJobStart):
            return self.STEP_SEQUENCE[self.type][stepRelativeToJobStart % len(self.STEP_SEQUENCE[self.type])]


class ULN2003MotorDriver(UnipolarMotorDriver):
    pass


class TB6612MotorDriver(UnipolarMotorDriver):
    pass
