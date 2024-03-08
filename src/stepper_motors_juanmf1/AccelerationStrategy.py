from bisect import bisect_left

from stepper_motors_juanmf1.myMath import cmp, sign
from stepper_motors_juanmf1.StepperMotor import StepperMotor
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint


class AccelerationStrategy:
    """
    Base Strategy class implements flat (min) speed.
    """

    def __init__(self, stepperMotor: StepperMotor, delayPlanner, steppingModeMultiple, rampSteps=None):
        tprint(f"stepperMotor: {stepperMotor}, steppingModeMultiple {steppingModeMultiple}")
        """
        :param stepperMotor: Stepper to take parameters from, like [min/max]PPS and so on.
        :param delayPlanner: delayPlanner to use to compute sleep time between steps as motor ramps up/down & while
                             steady
        :param rampSteps: How many steps it takes to ramp the motor. It assumes same number of steps to ramp up or down
        """
        self.realDirection = 0
        # Todo: evaluate having rampUpSteps & rampDownSteps as it might differ.
        self.rampSteps = rampSteps

        self.maxPps = int(stepperMotor.getMaxPps() / steppingModeMultiple)
        self.minPps = int(stepperMotor.getMinPps() / steppingModeMultiple)

        # Keeps track of speed, in terms of pps, and direction (-, +) useful for momentum considerations.
        self.currentPps = self.minPps
        minSleepTimeSeconds = 1 / self.maxPps
        maxSleepTimeSeconds = 1 / self.minPps
        self.minSleepTimeUs = int(minSleepTimeSeconds * 1_000_000)
        self.maxSleepTimeUs = int(maxSleepTimeSeconds * 1_000_000)

        # Should start at max sleep time, override in subclasses.
        self.currentSleepTimeUs = self.maxSleepTimeUs
        self.shouldBreakCache = {}

        self.delayPlanner = delayPlanner
        self.steppingModeMultiple = steppingModeMultiple
        delayPlanner.setAccelerationStrategy(self)

    def setMaxPPS(self, maxPps):
        self.maxPps = int(maxPps / self.steppingModeMultiple)
        self.minSleepTimeUs = int(1_000_000 / self.maxPps)

    def getMaxPPS(self):
        return self.maxPps

    def getCurrentSleepUs(self):
        return self.currentSleepTimeUs

    def done(self):
        """
        Signal from client code motor is done working. Reset to initial values.
        """
        self.delayPlanner.done()
        self.realDirection = 0
        # Keeps track of speed, in terms of pps, and direction (-, +) useful for momentum considerations.
        self.currentPps = self.minPps
        # Should start at max sleep time, override in subclasses.
        self.currentSleepTimeUs = self.maxSleepTimeUs

    # Speeds up if targetSleepTime isn't reached yet
    def decreaseSleepTime(self, targetSleepTime):
        """
        Speeds up if targetSleepTime isn't reached yet. By setting self.currentSleepTimeUs
        :param targetSleepTime: min sleep time attainable.
        """
        pass

    def increaseSleepTime(self, targetSleepTime):
        """
        Slows down if targetSleepTime isn't reached yet. By setting self.currentSleepTimeUs
        In position based slowdowns, it might be necessary to perform direction reversals and speedups.
        :param targetSleepTime: max sleep time attainable.
        """
        pass

    def shouldBreak(self, pendingSteps):
        """
        Given self.currentPps (speed) determine if should start breaking to stop in pendingSteps.
        Assumes motor moving towards targetPosition, within self.rampSteps window, but potentially not at max speed
        This opens the possibility if still needing to accelerate to a certain point and then break.

        :param pendingSteps: steps to reach target position or total number of steps.
        """
        pass

    #
    def canStop(self):
        """
        Only can stop if is stopped or at min speed.
        :return: boolean indicating motor can stop.
        """
        return self.currentPps is None or abs(self.currentPps) <= self.minPps + 1

    def isMaxSpeed(self):
        return self.maxPps <= abs(self.currentPps) + 1

    def isRightDirection(self, currentPosition, targetPosition):
        return cmp(targetPosition, currentPosition) == self.realDirection

    def computeSleepTimeUs(self, current, target, directionChangeListener=None):
        """
        Increase or decrease sleep time, conversely affecting speed in PPS.
        :param current:
        :param target:
        :param directionChangeListener: callee to call if and when direction changes
        :return: 1 + currentStep
        """
        return self.delayPlanner.computeDelay(current, target, directionChangeListener)


class LinearAcceleration(AccelerationStrategy):
    """
    Constant decrease in sleep time between steps, attempts to cause linear acceleration.
    The Speed delta must be less than the max speed jump your motor can take at a speed with its
    minimum torque (wasteful).
    """

    def __init__(self, stepperMotor: StepperMotor, delayPlanner, steppingModeMultiple, rampSteps=25):
        super().__init__(stepperMotor, delayPlanner, steppingModeMultiple, rampSteps)
        self.sleepDeltaUs = (self.maxSleepTimeUs - self.minSleepTimeUs) / rampSteps
        self.currentSleepTimeUs = self.maxSleepTimeUs

    def decreaseSleepTime(self, targetSleepTime):
        nextTime = self.currentSleepTimeUs - self.sleepDeltaUs
        self.currentSleepTimeUs = nextTime if targetSleepTime < nextTime else targetSleepTime
        self.currentPps = int(self.realDirection * (1_000_000 / self.currentSleepTimeUs))

    #
    def increaseSleepTime(self, targetSleepTime):
        nextTime = self.currentSleepTimeUs + self.sleepDeltaUs
        self.currentSleepTimeUs = nextTime if targetSleepTime > nextTime else targetSleepTime
        self.currentPps = int(self.realDirection * (1_000_000 / self.currentSleepTimeUs))

    def shouldBreak(self, pendingSteps):
        return self.maxSleepTimeUs >= self.currentSleepTimeUs + (self.sleepDeltaUs * pendingSteps)


class ExponentialAcceleration(AccelerationStrategy):
    """
    An exponential growth is a terrible fit for stepper motors, with more Torque at low PPS, So this will implement an
    exponential decrease in the PPS increment. As it's feasible for steppers (e.g. with min speed 200 and max 2000) to
    jump from 200 PPS to 400 PPS, but not likely to make it from 1000 to 2000, or even 1200 (same increment as minPPS).
    Base SpeedUp formula: pps * b^(1.01 - (pps/maxPps)) =>
       nextPps = pps * b^(1) when pps = minPps
       nextPps = pps * b^(0) when pps = maxPps
    Base SpeedDown formula: pps / b^(1-(pps/maxPps)) =>
       nextPps = pps / b^(1) when pps = minPps
       nextPps = pps / b^(0) when pps = maxPps
    b is initialIncrementFactor=2
    For simulations see https://www.desmos.com/calculator/luvnt6dtae where:
    Fn7: x = a shows currentPPS (intersection with Fn3: Y = x; shows how slowDown curve breaks and speedUp curve
      accelerates)
    Fn1: is speedUp curve: a=currentPPS; b=initialIncrementFactor: c=maxPPS of your motor
    Fn2: is slowDown curve: a=currentPPS; b=initialIncrementFactor: c=maxPPS of your motor
    When you move "a" bar to change currentPPS the speedUp nd speedDown curves converge to maxPPS when a -> maxPPS meaning
      no acceleration.
    """
    def __init__(self, stepperMotor: StepperMotor, delayPlanner, steppingModeMultiple, initialIncrementFactor=2):
        super().__init__(stepperMotor=stepperMotor, delayPlanner=delayPlanner,
                         steppingModeMultiple=steppingModeMultiple, rampSteps=None)
        pps = self.maxPps
        self.stepsToBreakCache = [pps]
        # With this formula, breaking takes more steps than accelerating. using breaking to determine rampSteps
        while pps > self.minPps:
            pps /= initialIncrementFactor ** (1.01 - (pps / self.maxPps))
            self.stepsToBreakCache = [pps] + self.stepsToBreakCache

        self.rampSteps = len(self.stepsToBreakCache)
        self.initialIncrementFactor = initialIncrementFactor

    def decreaseSleepTime(self, targetSleepTime):
        nextPps = int(self.currentPps * self.initialIncrementFactor ** (1.01 - (abs(self.currentPps) / self.maxPps)))
        if abs(nextPps) > self.maxPps:
            nextPps = int(sign(self.currentPps) * self.maxPps)

        self.currentPps = nextPps
        self.currentSleepTimeUs = 1_000_000 / nextPps

    def increaseSleepTime(self, targetSleepTime):
        nextPps = int(self.currentPps / self.initialIncrementFactor ** (1.01 - (abs(self.currentPps) / self.maxPps)))
        if abs(nextPps) < self.minPps:
            nextPps = int(sign(self.currentPps) * self.minPps)

        self.currentPps = nextPps
        self.currentSleepTimeUs = 1_000_000 / nextPps

    def shouldBreak(self, pendingSteps):
        return (self.rampSteps >= pendingSteps
                and bisect_left(self.stepsToBreakCache, self.currentPps)) >= pendingSteps


class CustomAccelerationPerPps(AccelerationStrategy):
    """
    Inspired in research https://www.atlantis-press.com/proceedings/ammee-17/25878485 :
    Abstract
    In order to solve the problem that the stepping motor in the manipulator is frequently stopped and the conventional
    index algorithm can not make full use of the output torque under the high speed heavy load,the two-stage exponential
    acceleration algorithm for stepper motor is proposed.Based on the critical load curve, the two-stage exponential
    type and the conventional exponential acceleration scheme are analyzed and compared, which proves that the
    former has better fastness...
    It can be found that the higher the motor speed, the smaller the  acceleration value can be obtained at this moment.
    --

    But not quite. I decided to gather torque characteristics by benchmarking the motor {@see Benchmark} class
    so that the maximum speed boost at each "current speed" is recorded into the motor class's TORQUE_CURVE.
    Alternatively accepts an explicit set of multipliers to apply to current PPS as a proxy to instantaneous torque
    see `transformations`.
    """
    def __init__(self, stepperMotor: StepperMotor, delayPlanner, steppingModeMultiple, transformations=None):
        super().__init__(stepperMotor=stepperMotor, delayPlanner=delayPlanner,
                         steppingModeMultiple=steppingModeMultiple)
        """
        Curve of optimal acceleration.
        array of tuples with max speed boost at given PPS starting from stepperMotor.MIN_PPS 
        [(minPPS, speedIncrement_1), (minPPS * speedIncrement_1, speedIncrement_2), ..., (maxPPS, 0)]
        """
        self.transformations = list(transformations) if transformations is not None else list(stepperMotor.TORQUE_CURVE)
        if steppingModeMultiple != 1:
            correctedTransformations = []
            for pps, increment in self.transformations:
                correctedTransformations.append((pps / steppingModeMultiple, increment / steppingModeMultiple))
            self.transformations = correctedTransformations

        self.transformationsPPS = [item[0] for item in self.transformations]
        self.rampSteps = len(self.transformationsPPS)
        self.minPps = self.transformationsPPS[0]
        self.maxPps = self.transformationsPPS[-1]
        self.currentPps = self.minPps
        self.maxSleepTimeUs = 1_000_000 / self.minPps
        self.currentSleepTimeUs = 1_000_000 / self.currentPps
        self.minSleepTimeUs = 1_000_000 / self.maxPps
        self.lastSpeedDelta = None
        # Force canStop() logic, see Benchmark
        self.willStop = False

    def canStop(self):
        return self.willStop or super().canStop()

    def takeClosestSpeedTransformation(self, pps, *, lookBehind=False):
        """
        Searches in array of tuples [(pps1, accelerationFactor), ...]
        :return: linear interpolation if not exact hit, first or last increments if out of range.
        """
        pos = bisect_left(self.transformationsPPS, pps)

        if pos == len(self.transformations):
            pos = pos - 2 if pos > 1 and lookBehind or self.delayPlanner.isSlowingDown() else pos - 1
            return self.transformations[pos][1]
        elif pos == 0:
            return self.transformations[0][1]
        elif self.transformations[pos][0] == pps:
            # When slowing down, use previous transformation.
            pos = pos - 1 if lookBehind or self.delayPlanner.isSlowingDown() else pos
            return self.transformations[pos][1]

        beforePps = self.transformationsPPS[pos - 1]
        afterPps = self.transformationsPPS[pos]
        before = self.transformations[pos - 1][1]
        after = self.transformations[pos][1]
        transformation = before + ((after - before) / (afterPps - beforePps)) * (pps - beforePps)
        return transformation

    def decreaseSleepTime(self, targetSleepTime):
        nextPps = abs(self.currentPps) + self.takeClosestSpeedTransformation(abs(self.currentPps))
        nextTime = 1_000_000 / nextPps
        self.currentSleepTimeUs = nextTime if targetSleepTime < nextTime else targetSleepTime
        self.currentPps = round(self.realDirection * 1_000_000 / self.currentSleepTimeUs)

    def increaseSleepTime(self, targetSleepTime):
        nextPps = abs(self.currentPps) - self.takeClosestSpeedTransformation(abs(self.currentPps))
        nextTime = 1_000_000 / nextPps
        self.currentSleepTimeUs = nextTime if targetSleepTime > nextTime else targetSleepTime
        self.currentPps = round(self.realDirection * 1_000_000 / self.currentSleepTimeUs)

    def shouldBreak(self, pendingSteps):
        cPps = abs(self.currentPps)
        minPps = self.minPps
        nextPps = cPps
        stepsToBreak = 0
        if self.currentPps in self.shouldBreakCache:
            stepsToBreak = self.shouldBreakCache[self.currentPps]
        else:
            while minPps < nextPps:
                nextPps -= self.takeClosestSpeedTransformation(nextPps, lookBehind=True)
                stepsToBreak += 1
            self.shouldBreakCache[self.currentPps] = stepsToBreak
        return pendingSteps <= stepsToBreak

    """
    Benchmark related methods bellow
    """

    def setMinPps(self, pps):
        pps = int(pps / self.steppingModeMultiple)
        self.minPps = pps
        self.maxSleepTimeUs = 1_000_000 / self.minPps
        if self.currentPps < self.minPps:
            self.currentPps = self.minPps
            self.currentSleepTimeUs = 1_000_000 / self.minPps

    def setMaxPPS(self, maxPps):
        raise RuntimeError("Cant set max PPS for Custom Acceleration, it's determined by torque curve.")

    def setMaxPpsForBench(self, pps):
        """
        To be used by benchmark module only
        """
        pps = int(pps / self.steppingModeMultiple)
        self.maxPps = pps
        self.minSleepTimeUs = 1_000_000 / pps
        if self.currentPps > pps:
            self.currentPps = pps
            self.currentSleepTimeUs = 1_000_000 / pps

    def doubleSpeedDelta(self):
        """
        Doubles last speed increment.
        """
        self.lastSpeedDelta = self.transformations[-1][1]
        self.transformations[-1] = (self.transformations[-1][0], int(2 * self.transformations[-1][1]))

    def getSpeedDelta(self):
        return self.transformations[-1][1]

    def setSpeedDelta(self, speedDelta, overrideLastSpeed=False):
        """
        Manually sets speed increment. See Benchmark.
        :param speedDelta: New increment
        :param overrideLastSpeed: if set, prevents storing last increment, use this instead.
        """
        self.lastSpeedDelta = self.transformations[-1][1] \
                              if not overrideLastSpeed \
                              else overrideLastSpeed / self.steppingModeMultiple
        self.transformations[-1] = (self.transformations[-1][0], int(speedDelta / self.steppingModeMultiple))
        self.inferMaxPps()

    def resetMaxPps(self):
        """
        Adds a new speed boost to self.transformations. new PPS will be former last + its current increment.
        New increment is set to zero. Last increment must be zero to prevent faster than max speeds.
        """
        self.transformations.append((self.transformations[-1][0] + self.transformations[-1][1], 0))
        self.transformationsPPS = [item[0] for item in self.transformations]
        self.lastSpeedDelta = 0

        self.setMaxPpsForBench(self.transformationsPPS[-1])

    def inferMaxPps(self):
        """
        Resets max Speed using last speed Boost + its increment. Which should be zero unless benchmarking.
        """
        self.setMaxPpsForBench(self.transformations[-1][0] + self.transformations[-1][1])

    @classmethod
    def constructFrom(cls, controller, speedBoosts):
        """
        Constructs a new CustomAccelerationPerPps using current state of controller's accelerationStrategy.
        swap delayPlanner to newly created CustomAccelerationPerPps (breaks controller's accelerationStrategy!!)
        :param controller: Controller containing needed AccelerationStrategy.
        :param speedBoosts: speed transformations to use. [(minPPS, increment_1), ..., (maxPPS, 0)]
        :return: the new CustomAccelerationPerPps
        """
        out = CustomAccelerationPerPps(controller.stepperMotor,
                                       delayPlanner=controller.accelerationStrategy.delayPlanner,
                                       steppingModeMultiple=controller.steppingModeMultiple,
                                       transformations=speedBoosts)
        out.done()
        # Removing reference to delayPlanner from original accelerationStrategy
        controller.accelerationStrategy.delayPlanner = None
        return out


class InteractiveAcceleration(AccelerationStrategy):
    """
    AccelerationStrategy that enables manually setting speed (PPS) used for stress tests in
    :func:`~ Benchmark.benchmarkMotor`.
    """
    def __init__(self, stepperMotor, delayPlanner, steppingModeMultiple, minSpeedDelta, minPPS):
        super().__init__(stepperMotor=stepperMotor, delayPlanner=delayPlanner,
                         steppingModeMultiple=steppingModeMultiple, rampSteps=25)
        self.minSpeedDelta = int(minSpeedDelta / steppingModeMultiple)
        self.speedDelta = self.minSpeedDelta
        self.minPps = int(minPPS / steppingModeMultiple)
        self.maxPps = None
        self.minSleepTimeSeconds = 0
        self.minSleepTimeUs = 1_000_000 * self.minSleepTimeSeconds
        self.rampTokens = 0
        self.currentPps = self.minPps
        self.willStop = False
        self.lastSpeedDelta = None
        self.realDirection = 1

    def canStop(self):
        """
        :return: Value of manually set self.willStop
        """
        return self.willStop

    def decreaseSleepTime(self, targetSleepTime):
        nextPps = abs(self.currentPps) + self.speedDelta
        nextTime = 1_000_000 / nextPps
        self.currentSleepTimeUs = nextTime if targetSleepTime < nextTime else targetSleepTime
        self.currentPps = int(self.realDirection * 1_000_000 / self.currentSleepTimeUs)

    def increaseSleepTime(self, targetSleepTime):
        nextPps = abs(self.currentPps) - self.speedDelta
        nextTime = 1_000_000 / nextPps
        self.currentSleepTimeUs = nextTime if targetSleepTime > nextTime else targetSleepTime
        self.currentPps = int(self.realDirection * 1_000_000 / self.currentSleepTimeUs)

    def setSpeedDelta(self, speedDelta, overrideLastSpeed=False):
        self.lastSpeedDelta = self.speedDelta \
                              if not overrideLastSpeed \
                              else overrideLastSpeed / self.steppingModeMultiple
        self.speedDelta = int(speedDelta / self.steppingModeMultiple)

    def getSpeedDelta(self):
        return self.speedDelta

    def reverseSpeedDelta(self):
        self.setSpeedDelta(-self.speedDelta)

    def setMinPps(self, minPPS):
        minPPS = int(minPPS / self.steppingModeMultiple)
        self.maxSleepTimeUs = 1_000_000 / minPPS
        self.minPps = minPPS

    def setMaxPps(self, maxPPS):
        self.maxPps = int(maxPPS / self.steppingModeMultiple)
        self.minSleepTimeSeconds = 1 / self.maxPps
        self.minSleepTimeUs = 1_000_000 / self.maxPps

    def speedUp(self):
        self.rampTokens += 1

    def slowDown(self):
        self.rampTokens -= 1

    def computeSleepTimeUs(self, currentStep, steps, directionChangeListener=None):
        """
        Consumes self.rampTokens in order to increase or decrease sleep time, conversely affecting speed in PPS.
        :param currentStep: used to increment (regardless of direction) controller's currentStep.
        :param steps: Ignored
        :param directionChangeListener: Ignored
        :return: 1 + currentStep
        """
        if self.rampTokens > 0:
            self.rampTokens -= 1
            self.decreaseSleepTime(self.minSleepTimeUs)
        elif self.rampTokens < 0:
            self.rampTokens += 1
            self.increaseSleepTime(self.maxSleepTimeUs)
        return currentStep + 1

    def done(self, resetDelta=False):
        self.currentPps = self.minPps
        self.currentSleepTimeUs = self.maxSleepTimeUs
        self.speedDelta = self.minSpeedDelta if resetDelta else self.speedDelta


class DelayPlanner:
    """
    Handles changes in sleep time (i.e. speed in PPS).
    Modifies linked AccelerationStrategy's state.
    Determines if motor is ramping up or down, is in rest or steady (max) speed.
    """

    def __init__(self):
        self.accelerationStrategy = None

    def setAccelerationStrategy(self, accelerationStrategy):
        """
        Important to keep in mind to link to containing AccelerationStrategy, as there is a circular dependency here.
        and This instance is passed to AccelerationStrategy constructor, thus created before without passing an
        instance of AccelerationStrategy.
        :param accelerationStrategy:
        :return:
        """
        self.accelerationStrategy = accelerationStrategy

    def computeDelay(self, current, target, directionChangeListener=None):
        """
        Called every step to determine sleep time.
        Determines if sleep time should be changes at all, if so, incremented (rampingUp) or decremented (rampingDown)
        :param current: Current step or position
        :param target: Target step or position
        :param directionChangeListener: Controller provided callable in case of direction change
        :return: Updated current (-+1) depending on direction.
        """
        pass

    def done(self):
        """
        Controller decided to stop, signals us to go back to initial state.
        """
        pass

    def isSlowingDown(self):
        """
        Useful for determining speed Deltas as increment/decrement.
        :return: True if slowing down.
        """
        pass


class StaticDelayPlanner(DelayPlanner):
    """
    Computes state statically, depending on current step vs remaining or target steps.
    Used in scenarios where motor goes from A to B without unplanned changes.
    """

    def __init__(self):
        super().__init__()
        self.steps = None
        self.currentStep = None

    def computeDelay(self, currentStep, steps, directionChangeListener=None):
        # Smooth (de-)acceleration
        self.currentStep, self.steps = currentStep, steps
        if currentStep == 0:
            # Starts with max sleep time unchanged.
            pass
        elif currentStep <= self.accelerationStrategy.rampSteps:
            # Todo: find midpoint when steps < 2 * rampSteps
            self.accelerationStrategy.decreaseSleepTime(self.accelerationStrategy.minSleepTimeUs)
        elif currentStep >= steps - self.accelerationStrategy.rampSteps:
            self.accelerationStrategy.increaseSleepTime(self.accelerationStrategy.maxSleepTimeUs)
        return currentStep + self.accelerationStrategy.realDirection

    def isSlowingDown(self):
        return self.currentStep >= self.steps - self.accelerationStrategy.rampSteps


class DynamicDelayPlanner(DelayPlanner):
    """
    Stateful delay planner, uses a state machine (see State class) to assess, given (unexpectedly) updated target
    position and current state (acceleration state + current position + current direction), next state in terms of
    RampingUp, RampingDown, Steady, Rest.
    Use when motor destination step can change on the fly. If your application always goes from A to B without
    interruptions or sudden changes, use StaticDelayPlanner
    """

    def __init__(self):
        super().__init__()
        self.currentState = DynamicDelayPlanner.Rest()

    def computeDelay(self, currentPosition, targetPosition, directionChangeListener=None):
        """
        Called every step to determine sleep time.
        Given current state (inertia in terms of signed cPps), driver's currentPosition & target position
        determine next sleep time and cPps.
        Returns updated value of Driver's current position which can change in a direction independent of Driver's
        direction due to momentum of the motor and load.
        Positions increase clockwise. When self.currentPps is positive we are going clockwise.
        directionChangeListener takes Direction (-1, 0, 1) as parameter.

        :param currentPosition: Controller keeps track of motor position in terms of steps given since 0.
        currentPosition helps determine, in relation to targetPosition and State what the next State should be.
        :param targetPosition: updated target position, could have changed from last step to this one.
        :param directionChangeListener: Controller provided callable in case of direction change
        :return: Updated current (-+1) depending on direction.
        """
        if currentPosition == targetPosition and self.accelerationStrategy.canStop():
            raise RuntimeError("Already at target position.")

        self.currentState = self.currentState.transition(
            currentPosition, targetPosition, self.accelerationStrategy, directionChangeListener)

        self.currentState.effectSpeed(self.accelerationStrategy)
        return currentPosition + self.accelerationStrategy.realDirection

    def isSlowingDown(self):
        return isinstance(self.currentState, DynamicDelayPlanner.RampingDown)

    def done(self):
        self.currentState = DynamicDelayPlanner.Rest()

    # State machine to represent transitions between [rest, steady, `rampingUp, rampingDown] in dynamic
    # (position based) acceleration.
    class State:
        """
        State machine design pattern.
        Used to track current motor acceleration state and assess next step when given updated
        currentPosition & targetPosition
        """

        def transition(self, currentPosition, targetPosition, accelerationStrategy, directionChangeListener):
            """
            Determine which State comes next.
            Valid transitions:
              Rest -> Rest
              Rest -> RampingUp
              RampingUp -> RampingUp
              RampingUp -> RampingDown
              RampingUp -> Steady
              Steady -> Steady
              Steady -> RampingDown
              RampingDown -> RampingDown
              RampingDown -> RampingUp
              RampingDown -> Rest
            :param currentPosition: Controller's motor's currentPosition, from initially being 0 when operation started.
            :param targetPosition: Controller's motor's updated target position. Could change between steps.
            :param accelerationStrategy: AccelerationStrategy to base decision on it's state (currentPPS & Direction).
            :param directionChangeListener:
            :return: State, next state (RampingUp, RampingDown, Rest, Steady)
            """
            pass

        def effectSpeed(self, accelerationStrategy):
            """
            Each state knows what to do with current speed. Either increase, decrease, or leave it alone.
            :param accelerationStrategy: AccelerationStrategy instance to change Speed to. See
            :func:`stepperMotors.AccelerationStrategy.decreaseSleepTime` &
            :func:`stepperMotors.AccelerationStrategy.increaseSleepTime`
            """
            pass

    class Rest(State):
        def __new__(cls, *args, **kwargs):
            # Todo: use same approach for Singleton as EventDispatcher.
            if not getattr(cls, '_instance', None):
                cls._instance = super().__new__(cls)
            return cls._instance

        def transition(self, currentPosition, targetPosition, accelerationStrategy, directionChangeListener):
            if (currentPosition == targetPosition) or targetPosition is None or currentPosition is None:
                return self
            accelerationStrategy.realDirection = cmp(targetPosition, currentPosition)
            directionChangeListener(accelerationStrategy.realDirection)
            out = DynamicDelayPlanner.RampingUp()
            out.fromRest = True
            return out

    class RampingUp(State):
        def __new__(cls, *args, **kwargs):
            if not getattr(cls, '_instance', None):
                cls._instance = super().__new__(cls)
            return cls._instance

        def __init__(self):
            self.fromRest = True

        def transition(self, currentPosition, targetPosition, accelerationStrategy, directionChangeListener):
            pendingSteps = abs(currentPosition - targetPosition)
            isRightDir = accelerationStrategy.isRightDirection(currentPosition, targetPosition)
            shouldBreak = accelerationStrategy.shouldBreak(pendingSteps)
            if isRightDir and accelerationStrategy.isMaxSpeed() and not shouldBreak:
                return DynamicDelayPlanner.Steady()
            elif not isRightDir or shouldBreak:
                # Todo: pendingSteps == 0 and pps == minPPS we should be able to stop without rampDown.
                return DynamicDelayPlanner.RampingDown()
            return self

        def effectSpeed(self, accelerationStrategy):
            if self.fromRest:
                # Prevents skipping minPps
                self.fromRest = False
                return
            accelerationStrategy.decreaseSleepTime(accelerationStrategy.minSleepTimeUs)

    class RampingDown(State):
        def __new__(cls, *args, **kwargs):
            if not getattr(cls, '_instance', None):
                cls._instance = super().__new__(cls)
            return cls._instance

        def transition(self, currentPosition, targetPosition, accelerationStrategy, directionChangeListener):
            pendingSteps = abs(currentPosition - targetPosition)
            isRightDir = accelerationStrategy.isRightDirection(currentPosition, targetPosition)
            if pendingSteps == 0 and accelerationStrategy.canStop():
                return DynamicDelayPlanner.Rest()
            elif isRightDir and not accelerationStrategy.shouldBreak(pendingSteps):
                return DynamicDelayPlanner.RampingUp()
            elif pendingSteps > 0 and not isRightDir and accelerationStrategy.canStop():
                # Reversal
                accelerationStrategy.realDirection = cmp(targetPosition, currentPosition)
                directionChangeListener(accelerationStrategy.realDirection)
                return DynamicDelayPlanner.RampingUp()
            return self

        def effectSpeed(self, accelerationStrategy):
            accelerationStrategy.increaseSleepTime(accelerationStrategy.maxSleepTimeUs)

    class Steady(State):
        def __new__(cls, *args, **kwargs):
            if not getattr(cls, '_instance', None):
                cls._instance = super().__new__(cls)
            return cls._instance

        def transition(self, currentPosition, targetPosition, accelerationStrategy, directionChangeListener):
            pendingSteps = abs(currentPosition - targetPosition)
            if (not accelerationStrategy.isRightDirection(currentPosition, targetPosition)
                    or accelerationStrategy.shouldBreak(pendingSteps)):
                return DynamicDelayPlanner.RampingDown()
            return self
