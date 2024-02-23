from RPi import GPIO

import adafruit_motor.stepper as stepper
from adafruit_motor.stepper import StepperMotor as AdafruitStepperDriver
from stepper_motors_juanmf1.AccelerationStrategy import AccelerationStrategy

from stepper_motors_juanmf1.StepperMotor import StepperMotor
from stepper_motors_juanmf1.Controller import BipolarStepperMotorDriver, ThirdPartyAdapter


class AdafruitStepperAdapter(BipolarStepperMotorDriver, ThirdPartyAdapter):

    CW = GPIO.HIGH  # Clockwise Rotation
    CCW = GPIO.LOW  # Counterclockwise Rotation

    ADAFRUIT_STYLES = [stepper.SINGLE, stepper.DOUBLE, stepper.MICROSTEP, stepper.INTERLEAVE]

    # Mode pins are static, as they are shared among Turret motors.
    RESOLUTION = {'Full': 1,
                  'Half': 2,
                  '1/4': 4,
                  '1/8': 8,
                  '1/16': 16,  # Default when style is microstep
                  '1/32': 32,
                  '1/64': 64,
                  '1/128': 128,
                  }

    # DRV8825 uses min 10 microseconds HIGH for STEP pin.
    # Raspberry Pi sleeps (in % of desired time) increasingly longer the lower the time goes.
    # So in actuality this will sleep about 20uS
    PULSE_TIME_MICROS = 20

    # DRV8825 Uses HIGH pulse on LOW background for STEP signal.
    PULSE_STATE = GPIO.HIGH

    def __init__(self, *,
                 adafruitDriver: AdafruitStepperDriver,
                 stepperMotor: StepperMotor,
                 accelerationStrategy: AccelerationStrategy,
                 navigation,
                 stepsMode="Full",
                 useHoldingTorque=True,
                 jobQueue=None,
                 sharedMemory=None,
                 isProxy=False,
                 steppingCompleteEventName="steppingComplete",
                 jobCompletionObserver=None):

        # Adafruit allows for any pair > 2 microsteps, Here it's limited to 2**n <= 128
        assert not adafruitDriver._microsteps or adafruitDriver._microsteps == self.RESOLUTION[stepsMode], \
            "When configuring microsteps in Adafruit, need to make is consistent with stepsMode provided."

        super().__init__(stepperMotor=stepperMotor,
                         accelerationStrategy=accelerationStrategy,
                         navigation=navigation,
                         directionGpioPin=None,
                         stepGpioPin=None,
                         useHoldingTorque=useHoldingTorque,
                         stepsMode=stepsMode,
                         steppingCompleteEventName=steppingCompleteEventName,
                         jobQueue=jobQueue,
                         sharedMemory=sharedMemory,
                         isProxy=isProxy,
                         jobCompletionObserver=jobCompletionObserver)

        # Use self.useHoldingTorque to toggle (or not) enabled before and after stepping.
        self.enableGpioPin = True
        self.adafruitDriver = adafruitDriver
        self.adafruitStyle = self.determineSteppingStyle(stepsMode)

    def _initGpio(self, stepsMode):
        pass

    def pulseStart(self, stepRelativeToJobStart=None):
        self.adafruitDriver.onestep(direction=self.currentDirection, style=self.adafruitStyle)

    def pulseStop(self):
        pass

    def setDirection(self, directionState):
        # Todo: update sharedMemory
        # Translating potential negative values to GPIO.LOW
        directionState = GPIO.HIGH if directionState == GPIO.HIGH else GPIO.LOW
        self.currentDirection = directionState

    def setSleepMode(self, sleepOn=False):
        raise RuntimeError("Adafruit HAT does not have sleep pin.")

    def setEnableMode(self, enableOn=True):
        if not enableOn:
            self.adafruitDriver.release()

    def determineSteppingStyle(self, stepsMode):
        assert stepsMode in self.ADAFRUIT_STYLES or stepsMode in self.RESOLUTION
        if stepsMode in self.RESOLUTION:
            return stepper.MICROSTEP
        return stepsMode
