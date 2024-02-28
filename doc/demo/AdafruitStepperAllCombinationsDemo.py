import itertools
import board
from adafruit_motorkit import MotorKit
from stepper_motors_juanmf1.ControllerFactory import ControllerBuilder
from stepper_motors_juanmf1.StepperMotor import GenericStepper
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint, flush_streams
from adafruit_motor.stepper import FORWARD, SINGLE, MICROSTEP, INTERLEAVE, DOUBLE


def run(accCallable, navCallable, stepMode, kit, nema):
    print(accCallable[1], navCallable[1], stepMode, kit, nema)
    builder: ControllerBuilder = (ControllerBuilder()
                          .getBasicBuilder(nema, directionGpioPin=None, stepGpioPin=None)
                          .withAdafruitDriver(kit.stepper1)
                          .withHoldingTorqueEnabled(False))
    navCallable[0](builder)
    accCallable[0](builder)
    builder.withsteppingMode(stepMode)
    driver = builder.buildAdafruitStepperDriverAdapter()
    return driver

def key(accCallable, navCallable, stepMode):
    """
    Convenience method to make compound keys for a dict of drivers.
    :param accCallable: a string representation of acceleration setting method in builder
    :param navCallable: a string representation of navigationStyle setting method in builder
    :param stepMode: the step mode used to build the driver adapter.
    :return: string key
    """
    return f"{accCallable[1]}-{navCallable[1]}-{stepMode}"

def main():
    maxPPS = 200
    minPPS = 50
    nema = GenericStepper(minPps=minPPS, maxPps=maxPPS, minSleepTime=1 / maxPPS, maxSleepTime=1 / minPPS, spr=200)
    kit = MotorKit(i2c=board.I2C(), steppers_microsteps=2)
    accs = [(lambda b: b.withLinearAcceleration(), 'withLinearAcceleration'),
            (lambda b: b.withExponentialAcceleration(), 'withExponentialAcceleration')]
    navs = [(lambda b: b.withNavigationStyleStatic(), 'withNavigationStyleStatic'),
            (lambda b: b.withNavigationStyleDynamic(), 'withNavigationStyleDynamic'),
            (lambda b: b.withNavigationStyleSynchronized(), 'withNavigationStyleSynchronized')]
    steppingModes = ['Full', 'Half', '1/8', '1/16']
    steppingStyles = [SINGLE, DOUBLE, INTERLEAVE, MICROSTEP]
    allSteppingModes = steppingModes + steppingStyles
    drivers={}
    for tup in itertools.product(accs, navs, allSteppingModes):
        drivers[key(*tup)] = run(*tup, kit, nema)
    kit.stepper1.release()
    return drivers

drivers = main()
last = None
for d in drivers.values():
    d.signedSteps(d.stepperMotor.getSpr()).result()
    last = d

last.release()


"""
To copy paste in python console and test, individually, manually 
"""
maxPPS = 200
minPPS = 50
nema = GenericStepper(minPps=minPPS, maxPps=maxPPS, minSleepTime=1 / maxPPS, maxSleepTime=1 / minPPS, spr=200)
kit = MotorKit(i2c=board.I2C(), steppers_microsteps=2)
accs = [(lambda b: b.withLinearAcceleration(), 'withLinearAcceleration'),
        (lambda b: b.withExponentialAcceleration(), 'withExponentialAcceleration')]
navs = [(lambda b: b.withNavigationStyleStatic(), 'withNavigationStyleStatic'),
        (lambda b: b.withNavigationStyleDynamic(), 'withNavigationStyleDynamic'),
        (lambda b: b.withNavigationStyleSynchronized(), 'withNavigationStyleSynchronized')]
d = run(accs[1], navs[1], 'Full', kit, nema)

# 1 revolution
steps = nema.getSpr()
d.signedSteps(steps)
# Not needed if driver.useHoldingTorque is False
d.release()
flush_streams()
