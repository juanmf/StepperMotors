"""
Demonstrate how to build and use Adafruit drivers, wrapped with a BipolarStepperMotorDriver
and all it's acceleration strategies.
"""
import board
from adafruit_motor import stepper
from adafruit_motorkit import MotorKit
from stepper_motors_juanmf1.AdafruitControllerAdapter import AdafruitStepperAdapter
from stepper_motors_juanmf1.Controller import BipolarStepperMotorDriver
from stepper_motors_juanmf1.ControllerFactory import DynamicControllerFactory, StaticControllerFactory, \
    SynchronizedControllerFactory, ControllerBuilder
from stepper_motors_juanmf1.StepperMotor import Nema17_42Ncm_17HS4401

from src.stepper_motors_juanmf1.Navigation import BasicSynchronizedNavigation

SUPPORTED_STEPPING_STYLES_OR_MODES = AdafruitStepperAdapter.ADAFRUIT_STYLES_TO_OURS + AdafruitStepperAdapter.RESOLUTION.keys()

# Threads in same process (inefficient)
# factory = StaticControllerFactory()        # Non-interruptible stepping jobs
# factory = SynchronizedControllerFactory()  # Interruptible stepping jobs; managed in a centralized way,
#   no competing stepping threads.
factory = DynamicControllerFactory()  # Interruptible stepping jobs

kit = MotorKit(i2c=board.I2C())
adafruitDriver1 = kit.stepper1
adafruitDriver2 = kit.stepper2
stepMode = SUPPORTED_STEPPING_STYLES_OR_MODES[SUPPORTED_STEPPING_STYLES_OR_MODES.index[stepper.MICROSTEP]]

motor = Nema17_42Ncm_17HS4401(loaded=True)
motor2 = Nema17_42Ncm_17HS4401(loaded=True)

adafruitAdapter1 = factory.getExponentialAdafruitStepperWith(stepperMotor=motor,
                                                             adafruitDriver=adafruitDriver1,
                                                             stepsMode=stepMode)

adafruitAdapter2 = (ControllerBuilder()
                    .getBasicBuilder(motor2, directionGpioPin=None, stepGpioPin=None, stepsMode="1/8")
                    .withAdafruitDriver(adafruitDriver2)
                    .withLinearAcceleration()
                    .buildAdafruitStepperDriverAdapter())

adafruitAdapter1.signedMicroSteps(steps=-800)
adafruitAdapter2.signedMicroSteps(steps=400)
