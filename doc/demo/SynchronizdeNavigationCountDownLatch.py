import time

from stepper_motors_juanmf1.ControllerFactory import ControllerBuilder
from stepper_motors_juanmf1.Navigation import BasicSynchronizedNavigation
from stepper_motors_juanmf1.StepperMotor import Nema17_42Ncm_17HS4401, PG35S_D48_HHC2

motor1 = PG35S_D48_HHC2(loaded=True)
motor2 = Nema17_42Ncm_17HS4401(loaded=True)

controller1 = (ControllerBuilder.getBasicBuilder(motor1, directionGpioPin=13, stepGpioPin=19, sleepGpioPin=12,
                                                 stepsMode="Full", modeGpioPins=None)
                                .withNavigationStyleSynchronized()  # Nav style HAS TO be defined before Acceleration
                                .withCustomTorqueCurveAccelerationAcceleration()
                                .buildDRV8825Driver())

controller2 = (ControllerBuilder.getBasicBuilder(motor2, directionGpioPin=8, stepGpioPin=25, enableGpioPin=1,
                                                 stepsMode="1/16", modeGpioPins=None)
                                .withNavigationStyleSynchronized()
                                .withExponentialAcceleration()
                                .buildTMC2209StandaloneDriver())

# Singleton, constructor returns only instance
navigation: BasicSynchronizedNavigation = BasicSynchronizedNavigation()  # Equivalent to `controller2.navigation`
navigation.setCountDown(2)

# Both motors should start together
controller1.signedSteps(100)
time.sleep(0.5)
controller2.signedSteps(1600)
