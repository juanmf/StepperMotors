import time

from stepper_motors_juanmf1.ControllerFactory import ControllerBuilder, MultiProcessingControllerFactory
from stepper_motors_juanmf1.Navigation import BasicSynchronizedNavigation
from stepper_motors_juanmf1.StepperMotor import Nema17_42Ncm_17HS4401, PG35S_D48_HHC2, GenericStepper

motor1 = PG35S_D48_HHC2(loaded=True)
motor2 = Nema17_42Ncm_17HS4401(loaded=True)


controller1 = (ControllerBuilder.getBasicBuilder(motor1, directionGpioPin=13, stepGpioPin=19, sleepGpioPin=12,
                                                 stepsMode="Full", modeGpioPins=None)
                                .withNavigationStyleSynchronized()  # Nav style HAS TO be defined before Acceleration
                                .withCustomTorqueCurveAccelerationAcceleration()
                                .buildDRV8825Driver())
builder2 = (ControllerBuilder.getBasicBuilder(motor2, directionGpioPin=8, stepGpioPin=25, enableGpioPin=1,
                                              stepsMode="1/16", modeGpioPins=None)
                             .withNavigationStyleSynchronized()
                             .withExponentialAcceleration())
controller2 = builder2.buildTMC2209StandaloneDriver()

# Singleton, constructor returns only instance
navigation: BasicSynchronizedNavigation = BasicSynchronizedNavigation.getInstance()  # Equivalent to `controller2.navigation`
navigation.setCountDown(2)

# Both motors should start together
controller1.signedSteps(100)
time.sleep(0.5)
controller2.signedSteps(1600)

"""
Multiprocess presents some challenges.
You'll need one BasicSynchronizedNavigation instance in Main process per each child process.
Each child process will hae only one singleton.
This is worked around by making BasicSynchronizedNavigation a keyed multiton.
"""

nav1 = BasicSynchronizedNavigation.getInstance(newMultitonKey=0)
nav2 = BasicSynchronizedNavigation.getInstance(newMultitonKey=1)

motor1_1 = Nema17_42Ncm_17HS4401(loaded=True)
motor1_2 = GenericStepper.Builder().copy(motor1)
motor2_1 = Nema17_42Ncm_17HS4401(loaded=True)
motor2_2 = GenericStepper(minPps=200, maxPps=4000, minSleepTime=1/4000, maxSleepTime=1/200)

controllerFactory = MultiProcessingControllerFactory()
# Child Process 1
driver1_1, driver1_2 = (controllerFactory.setUpProcess()
                        # Todo: Make sure multiProcessObserver can be None
                        .withDriver(factoryFnReference=controllerFactory.getMpCustomTorqueCharacteristicsDRV8825With,
                                    stepperMotor=motor1_1, directionGpioPin=13, stepGpioPin=19, sleepGpioPin=12)
                        .withDriver(factoryFnReference=controllerFactory.getMpCustomTorqueCharacteristicsDRV8825With,
                                    stepperMotor=motor1_1, directionGpioPin=8, stepGpioPin=25, sleepGpioPin=1)
                        .spawn(withBasicSynchronizedNavigationMultitonKey=0))  # Navigation to use


builder2 = (ControllerBuilder.getBasicBuilder(motor2, directionGpioPin=8, stepGpioPin=25, enableGpioPin=1,
                                              stepsMode="1/16", modeGpioPins=None)
                             .withNavigationStyleSynchronized()
                             .withExponentialAcceleration())
# Child Process 2
# Didn't check pins bellow
driver2_1, driver2_2 = (controllerFactory.setUpProcess()
                        # Todo: Make sure multiProcessObserver can be None
                        .withDriver(factoryFnReference=controllerFactory.getMpCustomTorqueCharacteristicsDRV8825With,
                                    stepperMotor=motor1_1, directionGpioPin=14, stepGpioPin=20, sleepGpioPin=15)
                        .withDriverBuilder(builder=builder2, builderMethodRef=builder2.buildDRV8825Driver)
                        .spawn(withBasicSynchronizedNavigationMultitonKey=1))  # Navigation to use

nav1.setCountDown(2)
driver1_1.signedSteps(100)
driver2_1.signedSteps(200)  # should run immediately

time.sleep(0.5)
driver1_2.signedSteps(100)  # starts driver1_1 & driver1_2 together, resets counter.

nav1.setCountDown(2)
driver1_1.signedSteps(100)
nav1.setCountDown(2)  # raise Error, re-setting countDown when it's in use.
time.sleep(0.5)
driver1_2.signedSteps(100)  # starts driver1_1 & driver1_2 together, resets counter.
