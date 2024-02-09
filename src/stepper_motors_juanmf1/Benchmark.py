import threading
import time

import sys
from sshkeyboard import listen_keyboard, stop_listening

from stepper_motors_juanmf1.AccelerationStrategy import CustomAccelerationPerPps
from stepper_motors_juanmf1.Controller import BipolarStepperMotorDriver
from stepper_motors_juanmf1.ControllerFactory import DynamicControllerFactory
from stepper_motors_juanmf1.StepperMotor import GenericStepper
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint, flush_current_thread_only, flush_current_thread_only

class Benchmark:
    """
    By stress testing your motor, with human feedback, find min speed (min speed you feel comfortable with, when motor
    is not choppy anymore and there is continuity between steps); max speed (max rate of steps in PPS, Pulses Per
    Second your motor can take before getting stuck); and between former two values, max speed deltas from min to max.
    This last search is done by repeatedly restarting the motor with ever-increasing speed gaps between last known
    speed, starting at minPPS, and the next speed, to failure. When the user tells the system, the motor failed, search
    shifts to binary search between last known successful speed jump and failed speed jump. Process repeats until maxPPS
    is reached.
    The output is compatible with :func:`~StepperMotor.TORQUE_CHARACTERISTICS`, used by
    :func:`~CustomAccelerationPerPps` for acceleration profile, squeezing your motor's instantaneous torque to the
    limit. Alternatively, output can be passed to stepperMotors.CustomAccelerationPerPps.transformations in constructor
    overriding StepperMotor values.
    """
    MIN_PPS_DELTA = 5
    ABSOLUTE_MIN_PPS = 150

    def __init__(self):
        self.maxPpsFound = None
        self.speedDeltasSearchEnded = False
        self.isStoppingMotor = False
        self.lock = threading.Lock()
        self.minSpeedDelta = self.MIN_PPS_DELTA
        self.findSpeedJumpUpperLimit = None

    def benchmarkMotor(self, controller, minPps=None, maxPps=None):
        """
        Process has 4 steps, needs human feedback. Make sure you apply relevant load to the motor.
        1) Find min PPS
        2) Find max PPS
        3) Find max attainable  PPS increments from minPPS to maxPPS (i.e. minimum number of steps to reach maxPPS)
        :param controller: Controller to stress test.
        :param minPps: If provided, skips minPPS discovery
        :param maxPps: If provided, skips maxPPS discovery
        :return: list of max attainable PPS increments from minPPS to maxPPS as [(minPPS, inc1), ..., (maxPPS, 0)]
        """
        tprint("\n\n")
        tprint("Process has 4 steps, needs human feedback. Make sure you apply relevant load to the motor.")
        tprint("1) Find min PPS at which the motor show continuity between steps.")
        tprint("2) Find max PPS motor acn keep up with, this by using modest acceleration so might be slow.")
        tprint("3) Starting from minPPS will try to jump to greatest next PPS that you observe not to fail with.")
        tprint("4) after repeating 3 until maxPPS is reached, dump the data in a format compatible with "
              "stepperMotors.CustomAccelerationPerPps.transformations")
        tprint("\n\n")

        stop_listening()
        flush_current_thread_only()
        minPps = self.findMinPPS(controller) if minPps is None else minPps
        stop_listening()
        flush_current_thread_only()

        maxPps = self.findMaxPPS(controller) if maxPps is None else maxPps
        stop_listening()
        flush_current_thread_only()
        self.maxPpsFound = maxPps

        speedBoosts = self.findTransformations(controller, minPps, maxPps)
        listen_keyboard(
            on_press=lambda k: self.findMaxDeltaPerPPSControls(k, controller, speedBoosts),
            sequential=True,
            until='esc')
        self.printResults(minPps, maxPps, speedBoosts)
        return speedBoosts

    @staticmethod
    def printResults(minPps, maxPps, speedBoosts):
        tprint(f""
              f"Results"
              f"===========================================================================")
        tprint(f"MinSpeed PPS: {minPps}; MaxSpeed PPS: {maxPps}")
        tprint("optimal transformations for your motor with current load:")
        tprint(speedBoosts)
        tprint(f"")
        tprint("Same data to play in a spreadsheet")
        tprint("PPS\tSpeedDelta PPS")
        for pps, booster in speedBoosts:
            tprint(f"{pps}\t{booster}")
        tprint(f"")
        flush_current_thread_only()

    """
    Find min speed specific methods
    """
    def findMinPPS(self, controller: BipolarStepperMotorDriver):
        picked = {'picked': False, 'pps': 0}
        fn = lambda c, t, realDirection: self.keepMotorGoing(controller, c, t, picked)
        controller.stepClockWise(10_000, fn)
        while not picked['picked']:

            tprint(f"picked['picked']: {picked['picked']}")
            listen_keyboard(
                on_press=lambda k: self.findMinSpeedControls(k, controller, lambda: self.setPicked(controller, picked)),
                sequential=True,
                until='esc')
        minPps = picked['pps']
        controller.accelerationStrategy.setMinPps(minPps)
        return minPps

    def keepMotorGoing(self, controller, currentPosition, targetPosition, picked):
        if not picked['picked'] and currentPosition % 1000 == 999:
            fn = lambda c, t, realDirection: self.keepMotorGoing(controller, c, t, picked)
            controller.stepClockWise(10_000, fn)

    """
    Find max speed specific methods
    """

    def findMaxPPS(self, controller):
        tprint("SEARCHING FOR MAX PPS.")
        tprint("")
        picked = {'picked': False, 'pps': 0}
        fn = lambda c, t, realDirection: self.keepMotorGoingUp(controller, c, controller.getCurrentPosition(), picked)
        controller.stepClockWise(10_000, fn)
        flush_current_thread_only()
        listen_keyboard(
            on_press=lambda k: self.findMaxSpeedControls(k, controller, lambda: self.setPicked(controller, picked)),
            sequential=True,
            until='esc')

        maxPps = int(picked['pps'] - controller.accelerationStrategy.getSpeedDelta())
        controller.accelerationStrategy.setMaxPps(maxPps)
        tprint(f"Max Speed: {maxPps} <<===")
        return maxPps

    def keepMotorGoingUp(self, controller, currentPosition, lastChangedPosition, picked):
        if (not picked['picked']
                and (currentPosition - lastChangedPosition) % int(controller.accelerationStrategy.currentPps / 2)
                == 0):
            fn = lambda c, t, realDirection: self.keepMotorGoingUp(controller, c, currentPosition, picked)
            controller.accelerationStrategy.speedUp()
            controller.stepClockWise(10_000, fn)

    """
    Find speed boost specific methods
    """

    def findTransformations(self, controller, minPps, maxPps, speedBoosts=None):
        tprint("")
        tprint("FINDING SPEED BOOSTS FOR MOTOR.")
        tprint("")
        # Stop
        self.stopMotor(controller)

        if speedBoosts is None:
            minPps = int(minPps)
            speedBoosts = [(minPps, self.minSpeedDelta)]
            controller.accelerationStrategy = CustomAccelerationPerPps.constructFrom(controller, speedBoosts)
            controller.accelerationStrategy.setMinPps(minPps)
            controller.accelerationStrategy.setMaxPps(minPps + self.minSpeedDelta)

        controller.accelerationStrategy.done()

        tprint("")
        tprint(f"speedBoosts: {speedBoosts}")
        tprint("")
        flush_current_thread_only()
        controller.stepClockWise(10_000, lambda c, t, d: self.keepMotorSpeedingUp(controller, c, t, speedBoosts))

        return speedBoosts

    def keepMotorSpeedingUp(self, controller, currentPosition, targetPosition, speedBoosts):
        if self.stoppingMotor() or self.speedDeltasSearchEnded:
            return

        if controller.accelerationStrategy.currentPps >= self.maxPpsFound:
            tprint(f"Ending the Search. max speed reached. "
                  f"currentPps: {controller.accelerationStrategy.currentPps}; maxPps: {controller.accelerationStrategy.maxPps}"
                  f"accelerationStrategy: {type(controller.accelerationStrategy).__name__}")
            self.speedDeltasSearchEnded = True
            lastBoost = controller.accelerationStrategy.transformations[-1]
            controller.accelerationStrategy.setSpeedDelta(self.maxPpsFound - lastBoost[0])
            self.forcePickSpeedBoost(controller, speedBoosts, continueLooping=False)
            self.stopMotor(controller)
            stop_listening()

    def passedSpeedBoost(self, controller, speedBoosts):
        tprint(f"PASSED speed boost: cPPs: {controller.accelerationStrategy.currentPps} last Speed: {speedBoosts[-1][0]}")
        tprint("Restarting cycle.")
        if self.findSpeedJumpUpperLimit is None:
            controller.accelerationStrategy.doubleSpeedDelta()
            controller.accelerationStrategy.inferMaxPps()
            tprint(f"Doubling Delta: new delta: {controller.accelerationStrategy.getSpeedDelta()}")
        else:
            deltaDelta = (self.findSpeedJumpUpperLimit - controller.accelerationStrategy.getSpeedDelta()) / 2
            delta = round(controller.accelerationStrategy.getSpeedDelta() + deltaDelta)
            tprint(f"New Delta: {delta}")
            if deltaDelta >= self.minSpeedDelta:
                controller.accelerationStrategy.setSpeedDelta(delta)
            else:
                self.forcePickSpeedBoost(controller, speedBoosts)
                return
        self.findTransformations(
            controller, controller.accelerationStrategy.minPps, controller.accelerationStrategy.maxPps, speedBoosts)

    def failedSpeedBoost(self, controller, speedBoosts):
        tprint(f"FAILED speed boost: cPPs: {controller.accelerationStrategy.currentPps} last Speed: {speedBoosts[-1][0]}")
        tprint("Restarting cycle.")
        lastHealthySpeedDelta = controller.accelerationStrategy.lastSpeedDelta
        self.findSpeedJumpUpperLimit = controller.accelerationStrategy.getSpeedDelta()
        delta = round(lastHealthySpeedDelta + (self.findSpeedJumpUpperLimit - lastHealthySpeedDelta) / 2)
        tprint(f"new Limit: {self.findSpeedJumpUpperLimit}; newDelta: {delta}; prevDelta: {lastHealthySpeedDelta}")
        controller.accelerationStrategy.setSpeedDelta(delta, lastHealthySpeedDelta)

        self.findTransformations(
            controller, controller.accelerationStrategy.minPps, controller.accelerationStrategy.maxPps, speedBoosts)

    def repeatSpeedBoost(self, controller, speedBoosts):
        self.findTransformations(
            controller, controller.accelerationStrategy.minPps, controller.accelerationStrategy.maxPps, speedBoosts)

    def forcePickSpeedBoost(self, controller, speedBoosts, continueLooping=True):
        # Done
        self.findSpeedJumpUpperLimit = None
        tprint("")
        # Truncating to 6 decimals
        boost = round(controller.accelerationStrategy.getSpeedDelta())
        controller.accelerationStrategy.resetMaxPps()
        if not self.speedDeltasSearchEnded:
            controller.accelerationStrategy.setSpeedDelta(self.minSpeedDelta, self.minSpeedDelta)
        speedBoosts = controller.accelerationStrategy.transformations
        tprint(f"Found next speed boost of {boost}, for pps {speedBoosts[-2][0]}")
        tprint(f"speedBoosts: {speedBoosts}")
        tprint("")
        tprint("")

        if continueLooping:
            self.findTransformations(
                controller, controller.accelerationStrategy.minPps, controller.accelerationStrategy.maxPps, speedBoosts)

    """
    Utility methods follow
    """

    def setPicked(self, controller, picked):
        tprint("Enter pressed, setting picked True!!!")
        picked['picked'] = True
        picked['pps'] = controller.accelerationStrategy.currentPps
        self.stopMotor(controller)
        stop_listening()

    def stopMotor(self, controller):
        if self.stoppingMotor():
            tprint("skipping stopping motor a already stopping.")
            return

        self.stoppingMotor(True)
        controller.accelerationStrategy.willStop = True
        controller.stepClockWise(5, lambda a, b, c: self.resetStoppingFlag(a, b))
        time.sleep(0.5)
        # Safety measure.
        self.stoppingMotor(False)

    def stoppingMotor(self, value=None):
        if value is None:
            return self.isStoppingMotor
        with self.lock:
            self.isStoppingMotor = value

    def resetStoppingFlag(self, currentPosition, targetPosition):
        tprint(f"resetStoppingFlag: currentPosition: {currentPosition}, targetPosition: {targetPosition}")
        if currentPosition == targetPosition:
            tprint("Successfully stopped motor.")
            tprint("Successfully stopped motor.")
            tprint("")
            self.stoppingMotor(False)

    """
    Keyboard hot keys follow
    """

    @staticmethod
    def findMinSpeedControls(key, controller, picked):
        hotKeys = {
            'up': lambda: controller.accelerationStrategy.speedUp(),
            'down': lambda: controller.accelerationStrategy.slowDown(),
            'enter': picked,
        }
        method = hotKeys.get(key, lambda: False)
        method()

    @staticmethod
    def findMaxSpeedControls(key, controller, picked):
        hotKeys = {
            'up': lambda: controller.accelerationStrategy.reverseSpeedDelta(),
            'down': lambda: controller.accelerationStrategy.reverseSpeedDelta(),
            'enter': picked,
        }
        method = hotKeys.get(key, lambda: False)
        method()

    def findMaxDeltaPerPPSControls(self, key, controller, speedBoosts):
        hotKeys = {
            'y': lambda: self.passedSpeedBoost(controller, speedBoosts),
            'enter': lambda: self.forcePickSpeedBoost(controller, speedBoosts),
            'n': lambda: self.failedSpeedBoost(controller, speedBoosts),
            'r': lambda: self.repeatSpeedBoost(controller, speedBoosts),
            # Todo: 'u': lambda: self.undoSpeedBoost(controller, speedBoosts),
        }
        method = hotKeys.get(key, lambda: False)
        method()

    @staticmethod
    def initBenchmark(stepperMotor, directionPin, stepPin, *, sleepGpioPin=None, minPps=None, maxPps=None, minPpsDelta=None, absoluteMinPps=None):
        minPpsDelta = minPpsDelta if minPpsDelta is not None else Benchmark.MIN_PPS_DELTA
        absoluteMinPps = absoluteMinPps if absoluteMinPps is not None else Benchmark.ABSOLUTE_MIN_PPS
        controllerFactory = DynamicControllerFactory()

        driver = controllerFactory.getInteractiveDRV8825With(
            stepperMotor, directionPin, stepPin, minPpsDelta, absoluteMinPps, sleepGpioPin=sleepGpioPin)

        bench = Benchmark()
        # You can add your motor mi and max speed to skip their discovery phases.
        bench.benchmarkMotor(driver, minPps, maxPps)
        driver.killWorker()
        driver.executor.shutdown(wait=False, cancel_futures=True)

    @classmethod
    def main(cls):
        # Todo: test after library extraction.
        args = sys.argv[1:]
        if len(args) < 2 or not (int(args[0]) < 45 and int(args[1]) < 45):
            raise RuntimeError("Need directionPin, StepPin to init Controller.")

        motor = GenericStepper(maxPps=Benchmark.ABSOLUTE_MIN_PPS, minPps=Benchmark.ABSOLUTE_MIN_PPS)
        tprint("Benchmarking azimuth Motor")
        Benchmark.initBenchmark(motor, int(args[0]), int(args[1]))


if __name__ == '__main__':
    Benchmark.main()
