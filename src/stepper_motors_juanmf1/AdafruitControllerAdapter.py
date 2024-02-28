from RPi import GPIO

import adafruit_motor.stepper as stepper
from adafruit_motor.stepper import StepperMotor as AdafruitStepperDriver
from adafruit_motor.stepper import FORWARD, SINGLE, MICROSTEP, INTERLEAVE, DOUBLE

from stepper_motors_juanmf1.AccelerationStrategy import AccelerationStrategy

from stepper_motors_juanmf1.StepperMotor import StepperMotor
from stepper_motors_juanmf1.Controller import BipolarStepperMotorDriver, ThirdPartyAdapter


class AdafruitStepperDriverAdapter(BipolarStepperMotorDriver, ThirdPartyAdapter):

    CW = GPIO.HIGH  # Clockwise Rotation
    CCW = GPIO.LOW  # Counterclockwise Rotation

    ADAFRUIT_STYLES_TO_OURS = {SINGLE: 'Full', DOUBLE: 'Full', MICROSTEP: None, INTERLEAVE: 'Half'}
    OUR_STYLES_TO_ADAFRUIT = {'Full': SINGLE, 'Half': INTERLEAVE, '1/2': INTERLEAVE}

    # Mode pins are static, as they are shared among Turret motors.
    RESOLUTION = {'Full': 1,
                  'Half': 2,
                  '1/2': 2,
                  '1/4': 4,
                  '1/8': 8,
                  '1/16': 16,  # Default when style is microstep
                  '1/32': 32,
                  '1/64': 64,
                  '1/128': 128,
                  }

    MICROSTEP_MAP = {
        2: 'Half',
        4: '1/4',
        8: '1/8',
        16: '1/16',
        32: '1/32',
        64: '1/64',
        128: '1/128',
    }

    # Even though the default is 16 micro-steps per steps, Adafruit does that internally, one step is always one step.
    DEFAULT_STEPPING_MODE = 'Full'

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
                 stepsMode=DEFAULT_STEPPING_MODE,
                 useHoldingTorque=True,
                 jobQueue=None,
                 workerName=None,
                 sharedMemory=None,
                 isProxy=False,
                 steppingCompleteEventName="steppingComplete",
                 jobCompletionObserver=None):

        self.adafruitDriver: AdafruitStepperDriver = adafruitDriver
        self.stepsMode = stepsMode
        self.adafruitStyle = self.determineSteppingStyle(stepsMode)
        super().__init__(stepperMotor=stepperMotor,
                         accelerationStrategy=accelerationStrategy,
                         navigation=navigation,
                         directionGpioPin=None,
                         stepGpioPin=None,
                         useHoldingTorque=useHoldingTorque,
                         stepsMode=self.stepsMode,
                         steppingCompleteEventName=steppingCompleteEventName,
                         jobQueue=jobQueue,
                         workerName=workerName,
                         sharedMemory=sharedMemory,
                         isProxy=isProxy,
                         jobCompletionObserver=jobCompletionObserver)

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
        """
        Adafruit drier allows for:
          implicit microstepping: call oneStep() for one actual motor step, but does n _microsteps transparently.
          explicit microstepping: call oneStep(style=MICROSTEP) once every micro step, e.g. 16 times for one motor step.
          full or half steps: <style>:[stepper.SINGLE, stepper.DOUBLE, stepper.MICROSTEP, stepper.INTERLEAVE] call
            oneStep(style=<style>) for each:
               * full step: [stepper.SINGLE, stepper.DOUBLE] (i.e. configures microsteps) (DOUBLE is strength, not steps)
               * half step: stepper.INTERLEAVE (i.e. configured microsteps // 2)

        @param stepsMode:
        @return:
        """
        assert stepsMode in self.ADAFRUIT_STYLES_TO_OURS or stepsMode in self.RESOLUTION
        assert self.adafruitDriver._microsteps is None or self.adafruitDriver._microsteps in self.MICROSTEP_MAP

        if stepsMode == MICROSTEP:
            self.stepsMode = self.MICROSTEP_MAP[self.adafruitDriver._microsteps]
        elif stepsMode in self.ADAFRUIT_STYLES_TO_OURS:
            # convert Adafruit's to our values.
            self.stepsMode = self.ADAFRUIT_STYLES_TO_OURS[stepsMode]
        elif stepsMode in self.OUR_STYLES_TO_ADAFRUIT:
            stepsMode = self.OUR_STYLES_TO_ADAFRUIT[stepsMode]
        else:
            stepsMode = MICROSTEP
        # Full steps from our side either 'Full', stepper.SINGLE or stepper.DOUBLE.
        return stepsMode

    """
    proxy methods
    """

    def onestep(self, *, direction: int = FORWARD, style: int = SINGLE) -> None or int:
        """
        Actually returns current microstep
        @return: current microstep
        """
        return self.adafruitDriver.onestep(direction=direction, style=style)

    def release(self) -> None:
        self.adafruitDriver.release()
