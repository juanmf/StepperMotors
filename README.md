from src.stepper_motors_juanmf1.Navigation import DynamicNavigation

# Intro

Although Python and PC in general (RPI in particular) are not optimal for accurate timing of stepper motor pulses
(SO scheduler + Python sleep inaccuracies and, ATM, Global Interpreter Lock [GIL]), this library aims at providing
a versatile tool for managing a set of stepper motors (through their drivers) in several ways that might fit specific 
scenarios.

A few distinct concepts have been implemented:
* Driver (or Controller, used interchangeably), each instantiated driver will behave as a dedicated single thread worker
  (see `BlockingQueueWorker`) which receives steps jobs through a shared queue, in an attempt decouple steps timing 
  from the rest of the system, (here the GIL imposes some challenged in theory).
* Stepper Motor, encapsulates the motor characteristics, like min and max PPS (pulses per second), and instantaneous  
  torque. A Generic motor is implemented that can be constructed with specifics in case of lacking implementation of  
  your motor (you are welcome to add it).
* Navigation, The driver can navigate from A to B, setting direction & sending pulses to the motor, statically (as in 
  a 3D printer scenario where planning is made up front) or dynamically (for interactive or event based systems that
  need to quickly respond to unplanned speed and direction changes).
* Acceleration strategies or profiles, handle how to reach max speeds for the motor. Linear, Exponential & Custom (which
  takes motor's instantaneous torque or a list of transformations as input to max out your motor capabilities). Custom
  acceleration strategy has a pre-requisite that you use the Benchmark module (see bellow) to find optimal  
  transformations for your motor, in a production setup (proper load applied). All changes are effected as a function of
  current motor' speed in PPS. (other systems use curves as function of time. I found that impractical)
  * DelayPlanners (in tandem with Navigation modes) enable Drivers to handle inertia gracefully either in a static or 
    dynamic context. DelayPlanner implementations determine if it's time to start breaking, speeding up or stay steady.
    Acceleration strategies effect proper changes to speed. 
* Benchmark, a stress test module to find your motor's (under current load), min & max speeds, and instantaneous torque 
  characteristics, all in terms of PPS. For instantaneous torque characteristics the output (with format
  `[(minPPS, incrementPPS_1), (minPPS + incrementPPS_1, incrementPPS_2), ..., (maxPPS, 0)]`) can be
  used as `YourStepperMotorSubClass.TORQUE_CHARACTERISTICS` or as an input to `CustomAccelerationPerPps` acceleration 
  strategy's `transformations` constructor argument. This enables yor motor to reach max speed in the least amount of 
  steps possible while keeping synch (useful when speed matters).
  * Currently tested on (see https://www.reddit.com/r/robotics/comments/18ukw4p/benchmarking_stepper_motor/):  
    * Raspberry Pi 4B with,
    * DRV8825 driver board
    * PG35S_D48_HHC2 stepper motor 
 

![doc/collab.png](./doc/collab.png):

## Install

`pip install -i https://test.pypi.org/simple/ stepper-motors-juanmf1==0.0.2`

## Usage

### Acceleration Strategies

Factory methods from `ControllerFactory` ([🔗](./src/stepper_motors_juanmf1/ControllerFactory.py)) provide easy access to
well constructed Drivers with specific acceleration profiles.


```Python3
from stepper_motors_juanmf1 import (GenericStepper, 
                                    DRV8825MotorDriver, 
                                    ExponentialAcceleration, 
                                    DynamicDelayPlanner, 
                                    DynamicNavigation) 

class MyRoboticArm:
  """
  Example class using multiple driver instances (many motors).
  """
  
  def __init__(self):
    """
    Assuming you connected this motor driver's direction and step pins accordingly. 
    Step modes can also be set for micro-stepping. Tho ControllerFactory methods use only full step mode ATM.
    Sleep pin can also be set for DRV8825MotorDriver when no holding torque neeed.  
    """
    self.elbow =    MyRoboticArm.setupDriver(directionPin=23, StepPin=24) 
    self.shoulder = MyRoboticArm.setupDriver(directionPin=14, StepPin=15) 
    self.hand =     MyRoboticArm.setupDriver(directionPin=25, StepPin=8) 
    
  @staticmethod
  def setupDriver(*, directionPin, StepPin):
    stepperMotor = GenericStepper(maxPps=2000, minPps=150)
    delayPlanner = DynamicDelayPlanner()
    navigation = DynamicNavigation()
    
    acceleration = ExponentialAcceleration(stepperMotor, delayPlanner)
    # Important to set this reference once you have acceleration instance! 
    delayPlanner.setAccelerationStrategy(acceleration)
    return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, StepPin, navigation)
  

```

In this example we use `ExponentialAcceleration`, which exponentially decreases increments as PPS goes up in a `RampingUp` 
state behaves as follows ([see this to play around](https://www.desmos.com/calculator/luvnt6dtae)):

Current speed is `a`, marked by `x=a` fn for visual aid. Intersection between Identity fn `x=x` and `x=a` shows how 
large is the jump in PPS to next speed, either when `RampingUp`(Fn1, in red) or `RampingDown`(Fn2, in black).

In the following desktop test, black dot shows `(currentSpeed, nextSpeed)` or `F(currentPPS) -> nextPPS`. 

I try to feed F1 with nextPPS iteratively to simulate acceleration process that takes place at run time.
In this case with `minSpeed=200 PPS` and `maxSpeed=900 PPS` it'd take **6 steps to reach max speed** using 
`ExponentialAcceleration` with initial acceleration factor of (`b=2`).
 
<img src="./doc/photo1704731655.jpeg" alt="RampingUp starting at 200 PPS" width="400"/>

> Starting at minPPS of 200, ramping Up, uses Fn1 200 PPS -> 345 PPS
> So next speed will be 345 PPS...

<img src="./doc/photo1704731655_2.jpeg" alt="RampingUp next speed" width="400"/>

> (approximating 345 with 350) 
> 350 PPS -> 538 PPS 

<img src="./doc/photo1704731655_3.jpeg" alt="RampingUp next speed" width="400"/>

> (approximating 538 with 500, yes, 550 was closer...) 
> 500 PPS -> 685 PPS

<img src="./doc/photo1704731655_4.jpeg" alt="RampingUp next speed" width="400"/>

> (approximating 685 with 700) 
> 700 PPS -> 822 PPS 

<img src="./doc/photo1704731655_5.jpeg" alt="RampingUp next speed" width="400"/>

> 850 PPS -> 889 PPS

<img src="./doc/photo1704731655_6.jpeg" alt="RampingUp next speed" width="400"/>

> 900 PPS -> 906 PPS
> ExponentialAcceleration limits speeds to maxPPS so this would get stuck at 900 PPS
 
### Benchmark
for CLI Benchmark, you need download sources and cd to `src/stepper_motors_juanmf1/`
start gpio demon on your Raspberry Pi and start benchmark passing step and direction pins per your setup.
```
$ cd src/stepper_motors_juanmf1
$ sudo pigpiod -s 10 -t 0
$ python3 python3 Benchmark.py 23 24

# you can use `netstat netstat -tulpn` to check pigpiod demon 
#  is listening, in my case on `tcp6       0      0 :::8888` 
```
Annotated (`# <== `) output:
```
Benchmarking azimuth Motor
Setting direction pin 23 0.



Process has 4 steps, needs human feedback. Make sure you apply relevant load to the motor.
1) Find min PPS at which the motor show continuity between steps.
2) Find max PPS motor acn keep up with, this by using modest acceleration so might be slow.
3) Starting from minPPS will try to jump to greatest next PPS that you observe not to fail with.
4) after repeating 3 until maxPPS is reached, dump the data in a format compatible with stepperMotors.CustomAccelerationPerPps.transformations

picked['picked']: False
decreaseSleepTime: pps: 150.
decreaseSleepTime: TargetSleep: 0; currentSleepTime: 6666.666666666667.
CurrentPPS: 155.0                                    # <== initial PPS incremented by minimum delta.
decreaseSleepTime: pps: 155.0.
decreaseSleepTime: TargetSleep: 0; currentSleepTime: 6451.612903225807.
CurrentPPS: 160.0                                    # 'up pressed '<== increments by minimum delta.
decreaseSleepTime: pps: 160.0.
Enter pressed, setting picked True!!!                # < === picked min speed by pressing enter.
# < === <ommited a few lines>
Successfully stopped motor.
Successfully stopped motor.                          # < === <ommited a few lines>  

Setting direction pin 23 0. 
SEARCHING FOR MAX PPS.                               # < === search for max speed starts, wait till motor fails and 
                                                     # press enter ASAP, (have about 0.5 seconds before it keeps increasing speed)                               
decreaseSleepTime: pps: 150.
decreaseSleepTime: TargetSleep: 0; currentSleepTime: 6666.666666666667.
CurrentPPS: 155.0
decreaseSleepTime: pps: 155.0.
decreaseSleepTime: TargetSleep: 0; currentSleepTime: 6451.612903225807.
CurrentPPS: 160.0
decreaseSleepTime: pps: 160.0.
decreaseSleepTime: TargetSleep: 0; currentSleepTime: 6250.0.
CurrentPPS: 165.0
decreaseSleepTime: pps: 165.0.
decreaseSleepTime: TargetSleep: 0; currentSleepTime: 6060.606060606061.
CurrentPPS: 170.0
...
Enter pressed, setting picked True!!!               # < === picked maxPPS (system uses previous PPS to falure, you might 
                                                    # want to give it a margin.)
# < === <ommited a few lines>

Successfully stopped motor.
Successfully stopped motor.

speedBoosts: [(165.0, 5)]

Setting direction pin 23 1.
State Rest -> RampingUp
State RampingUp -> RampingUp
decreaseSleepTime: TargetSleep: 5882.35294117647; currentSleepTime: 6060.606060606061.
pos: 0; self.transformations [(165.0, 5)]
pos: 1; self.transformations [(165.0, 5)]
State RampingUp -> Steady
PASSED speed boost: cPPs: 170 last Speed: 165.0
Restarting cycle.
Doubling Delta: new delta: 10

FINDING SPEED BOOSTS FOR MOTOR.                     # < === from here press 'y' or 'n' & 'enter' to pick speed increments
                                                    # from minPPS to maxPPS

State Steady -> RampingDown
increaseSleepTime: TargetSleep: 6060.606060606061; currentSleepTime: 5882.35294117647.
pos: 1; self.transformations [(165.0, 10)]
resetStoppingFlag: currentPosition: 3526, targetPosition: 3526
Successfully stopped motor.
Successfully stopped motor.

Setting direction pin 23 0.

speedBoosts: [(165.0, 10)]

Setting direction pin 23 1.
State Rest -> RampingUp
State RampingUp -> RampingUp
decreaseSleepTime: TargetSleep: 5714.285714285715; currentSleepTime: 6060.606060606061.
pos: 0; self.transformations [(165.0, 10)]
pos: 1; self.transformations [(165.0, 10)]
State RampingUp -> Steady
PASSED speed boost: cPPs: 175 last Speed: 165.0
Restarting cycle.
Doubling Delta: new delta: 20

FINDING SPEED BOOSTS FOR MOTOR.

resetStoppingFlag: currentPosition: 3666, targetPosition: 3670
resetStoppingFlag: currentPosition: 3667, targetPosition: 3670
resetStoppingFlag: currentPosition: 3668, targetPosition: 3670
resetStoppingFlag: currentPosition: 3669, targetPosition: 3670
State Steady -> RampingDown
increaseSleepTime: TargetSleep: 6060.606060606061; currentSleepTime: 5714.285714285715.
pos: 1; self.transformations [(165.0, 20)]
resetStoppingFlag: currentPosition: 3670, targetPosition: 3670
Successfully stopped motor.
Successfully stopped motor.

Setting direction pin 23 0.

speedBoosts: [(165.0, 20)]
...
Ending the Search. max speed reached. currentPps: 185; maxPps: 185.0accelerationStrategy: CustomAccelerationPerPps

Found next speed boost of 15, for pps 165.0
speedBoosts: [(165.0, 15), (180.0, 0)]


pos: 1; self.transformations [(165.0, 15), (180.0, 0)]
transformations exact match: 180.0
State RampingUp -> Steady
resetStoppingFlag: currentPosition: 3673, targetPosition: 3677
Results===========================================================================
MinSpeed PPS: 165.0; MaxSpeed PPS: 180.00000000000003
optimal transformations for your motor with current load:
[(165.0, 15), (180.0, 0)]                                    # < === use this list as input to CustomAccelerationPerPps 

Same data to play in a spreadsheet
PPS	SpeedDelta PPS
165.0	15
180.0	0
```
Will clean up output shortly.
