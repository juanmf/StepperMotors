import itertools
from stepper_motors_juanmf1.ControllerFactory import ControllerBuilder
from stepper_motors_juanmf1.StepperMotor import Stepper_28BYJ_48
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint, flush_streams
from stepper_motors_juanmf1.UnipolarController import UnipolarMotorDriver


def run(accCallable, navCallable, stepMode, stepper):
    print(accCallable[1], navCallable[1], stepMode, stepper)
    builder: ControllerBuilder = (ControllerBuilder()
                                  # Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
                                  .getBasicBuilder(stepper, directionGpioPin=None, stepGpioPin=(6, 19, 13, 26))
                                  .withHoldingTorqueEnabled(True))
    navCallable[0](builder)
    accCallable[0](builder)
    builder.withsteppingMode(stepMode)
    driver = builder.buildUNL2003Driver()
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
    stepper = Stepper_28BYJ_48()
    accs = [(lambda b: b.withLinearAcceleration(), 'withLinearAcceleration'),
            (lambda b: b.withExponentialAcceleration(), 'withExponentialAcceleration')]
    navs = [(lambda b: b.withNavigationStyleStatic(), 'withNavigationStyleStatic'),
            (lambda b: b.withNavigationStyleDynamic(), 'withNavigationStyleDynamic'),
            (lambda b: b.withNavigationStyleSynchronized(), 'withNavigationStyleSynchronized')]
    steppingModes = ['Full', 'Half']
    # Types for 4 pins
    sequenceTypes = UnipolarMotorDriver.Sequence.PINS_USED_2_TYPES_MAP[4]
    drivers={}
    for tup in itertools.product(accs, navs, steppingModes + sequenceTypes):
        drivers[key(*tup)] = run(*tup, stepper)
    return drivers


drivers = main()

dir = 1
last1 = None
for k, d in drivers.items():
    print(f"Starting round: {k}")
    d.signedSteps(dir * d.stepperMotor.getSpr()).result()
    last1 = d
    dir *= -1

last1.setEnableMode(enableOn=False)


for k, d in drivers.items():
    print(f"Starting round: {k}")
    print(d.steppingModeMultiple)