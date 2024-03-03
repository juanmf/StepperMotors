from RPi import GPIO

from stepper_motors_juanmf1.AccelerationStrategy import AccelerationStrategy
from stepper_motors_juanmf1.Controller import BipolarStepperMotorDriver, ThirdPartyAdapter
from stepper_motors_juanmf1.StepperMotor import StepperMotor
from stepper_motors_juanmf1.Controller import NoDirectionPinDriver
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint


class UnipolarMotorDriver(BipolarStepperMotorDriver, NoDirectionPinDriver):
    PULSE_TIME_MICROS = 2
    """
    Unipolar don't use pulse time as bipolar, 2 represents a small enough pulse to prevent wait.
    """

    DISABLE_STATE = (GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.LOW)
    """
    When useHoldingTorque if False shitting down coils after stepping.
    """

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

        pinsCount = len(stepGpioPin)
        if stepsMode is None:
            # Todo: 5 pin sequences?
            assert pinsCount in [2, 3, 4]
            # Builders might set this to None.
            stepsMode = self.DEFAULT_STEPPING_MODE
        else:
            assert ((stepsMode in UnipolarMotorDriver.Sequence.SUPPORTED_MICROSTEPS) or
                    (stepsMode in UnipolarMotorDriver.Sequence.SUPPORTED_TYPES
                     and stepsMode in UnipolarMotorDriver.Sequence.PINS_USED_2_TYPES_MAP[pinsCount]))

        if stepsMode in UnipolarMotorDriver.Sequence.SUPPORTED_TYPES:
            sequence = stepsMode
            stepsMode = UnipolarMotorDriver.Sequence.SEQUENCE_2_MICROSTEPPING_MODE_MAP[stepsMode]
        else:
            sequenceKey = f"{stepsMode}-{pinsCount}"
            sequence = UnipolarMotorDriver.Sequence.MICROSTEPPING_MODE_2_SEQUENCE_MAP[sequenceKey]

        self.sequence = UnipolarMotorDriver.Sequence(sequence)

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

    def shutDownCoils(self):
        self.setEnableMode(False)

    def powerOnCoils(self):
        self.setEnableMode(True)

    def _initGpio(self, stepsMode):
        self._oneTimeInit()

        GPIO.setup(self.stepGpioPin, GPIO.OUT)
        GPIO.output(self.stepGpioPin, GPIO.LOW)

    def setDirection(self, directionState):
        # Todo: update sharedMemory
        # Translating potential negative values to GPIO.LOW
        directionState = GPIO.HIGH if directionState == GPIO.HIGH else GPIO.LOW
        self.currentDirection = directionState

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
        Achieve Enable/Disable by shutting down coils
        """
        state = self.sequence.getCurrent() if enableOn else self.DISABLE_STATE
        GPIO.output(self.stepGpioPin, state)

    def pulseStart(self, stepRelativeToJobStart=None):
        GPIO.output(
            self.stepGpioPin,
            self.sequence.getStepSequence(stepRelativeToJobStart))

    def pulseStop(self):
        # Not in Unipolar
        pass

    class Sequence:

        FULL = "4p-Full"
        FULL_LOW_POWER = "4p-Full-low"
        HALF = "4p-Half"
        FULL_2PIN = "2p-Full"
        FULL_3PIN = "3p-Full"
        HALF_3PIN = "3p-Half"

        SUPPORTED_TYPES = [FULL, FULL_LOW_POWER, FULL_2PIN, HALF, FULL_3PIN, HALF_3PIN]
        SUPPORTED_MICROSTEPS = ["Full", "Half", "1/2"]

        PINS_USED_2_TYPES_MAP = {
                4: [FULL, FULL_LOW_POWER, HALF],
                3: [FULL_3PIN, HALF_3PIN],
                2: [FULL_2PIN]}

        MICROSTEPPING_MODE_2_SEQUENCE_MAP = {
                "Full-4": FULL,
                "Full-3": FULL_3PIN,
                "Full-2": FULL_2PIN,
                "Half-4": HALF,
                "Half-3": HALF_3PIN,
                "1/2-4": HALF,
                "1/2-3": HALF_3PIN}

        SEQUENCE_2_MICROSTEPPING_MODE_MAP = {
                FULL: "Full",
                FULL_LOW_POWER: "Full",
                HALF: "Half",
                FULL_2PIN: "Full",
                FULL_3PIN: "Full",
                HALF_3PIN: "Half"}

        STEP_SEQUENCE = {
            FULL_2PIN: [(GPIO.LOW, GPIO.HIGH),   # 01
                        (GPIO.HIGH, GPIO.HIGH),  # 11
                        (GPIO.HIGH, GPIO.LOW)],  # 10

            FULL:  [(GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW),   # 1010
                    (GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW),   # 0110
                    (GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH),   # 0101
                    (GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH)],  # 1001

            FULL_LOW_POWER:  [(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.LOW),   # 1000
                              (GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.LOW),   # 0010
                              (GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.LOW),   # 0100
                              (GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.HIGH)],  # 0001

            HALF: [(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.LOW),    # 1000
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
            self.current = self.STEP_SEQUENCE[self.type][0]
            self.steppingMod = len(self.STEP_SEQUENCE[self.type])

        def getStepSequence(self, stepRelativeToJobStart):
            # Todo: Not sure if starting from zero on every stepping job is the right call.
            #   What happens when a job ends at a different phase and then the next starts at phase 0?
            self.current = self.STEP_SEQUENCE[self.type][stepRelativeToJobStart % self.steppingMod]
            return self.current

        def getCurrent(self):
            return self.current


class ULN2003MotorDriver(UnipolarMotorDriver):
    pass


class TB6612MotorDriver(UnipolarMotorDriver):
    pass
