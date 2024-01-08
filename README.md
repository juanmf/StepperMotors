# Intro

Although Python and PC in general (RPI in particular) are not optimal for accurate timing of stepper motor pulses
(SO scheduler + Python sleep inaccuracies and, ATM, Global Interpreter Lock [GIL]), this library aims at providing
a versatile tool for managing a set of stepper motors (through their drivers) in several ways that might fit specific 
scenarios.

A few distinct concepts have been implemented; see [doc/collab.png](./doc/collab.png):
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

## Install

`pip install -i https://test.pypi.org/simple/ stepper-motors-juanmf1==0.0.2`

## Usage

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
