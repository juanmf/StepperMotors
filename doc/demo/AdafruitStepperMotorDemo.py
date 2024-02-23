"""
Demonstrate how to build and use Adafruit drivers, wrapped with a BipolarStepperMotorDriver
and all it's acceleration strategies.
"""
import board
from adafruit_motor import stepper
from adafruit_motorkit import MotorKit
from stepper_motors_juanmf1.AdafruitMotorAdapter import AdafruitStepperAdapter
from stepper_motors_juanmf1.Controller import BipolarStepperMotorDriver
from stepper_motors_juanmf1.ControllerFactory import DynamicControllerFactory, StaticControllerFactory, \
    SynchronizedControllerFactory
from stepper_motors_juanmf1.StepperMotor import Nema17_42Ncm_17HS4401

from src.stepper_motors_juanmf1.Navigation import BasicSynchronizedNavigation

SUPPORTED_STEPPING_STYLES_OR_MODES = AdafruitStepperAdapter.ADAFRUIT_STYLES + AdafruitStepperAdapter.RESOLUTION.keys()

# Threads in same process (inefficient)
# factory = StaticControllerFactory()        # Non-interruptible stepping jobs
# factory = SynchronizedControllerFactory()  # Interruptible stepping jobs; managed in a centralized way,
                                             #   no competing stepping threads.
factory = DynamicControllerFactory()         # Interruptible stepping jobs

kit = MotorKit(i2c=board.I2C())
adafruitDriver = kit.stepper1
stepMode = SUPPORTED_STEPPING_STYLES_OR_MODES[SUPPORTED_STEPPING_STYLES_OR_MODES.index[stepper.MICROSTEP]]

motor = Nema17_42Ncm_17HS4401(loaded=True)
adafruitAdapter = factory.getExponentialAdafruitStepperWith(stepperMotor=motor,
                                                            adafruitDriver=adafruitDriver,
                                                            stepsMode=stepMode)

adafruitAdapter.signedMicroSteps(steps=-800)
