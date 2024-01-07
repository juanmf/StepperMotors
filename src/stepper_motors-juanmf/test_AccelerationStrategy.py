from unittest import TestCase

from AccelerationStrategy import TwoStageExponentialAcceleration, CustomAccelerationPerPps, \
    LinearAcceleration
from StepperMotor import PG35S_D48_HHC2, GenericStepper
from unittest.mock import MagicMock

class TestLinearAcceleration(TestCase):
    def test_decrease_sleep_time(self):
        ts = LinearAcceleration(GenericStepper(pps=200, minPps=100, maxTorquePps=15), 30)
        # fn = lambda:
        # ts.computePositionBasedSleepTimeUs(0, 30, fn)

        self.fail()

    def test_increase_sleep_time(self):
        self.fail()

    def test_should_break(self):
        self.fail()


class TestTwoStageExponentialAcceleration(TestCase):
    def test_decrease_sleep_time(self):
        ts = TwoStageExponentialAcceleration(PG35S_D48_HHC2(True))
        nextPps = ts.stepperMotor.MIN_PPS
        maxPps = ts.maxPps - 10
        stepsToBreak = 0
        while maxPps > nextPps:
            lastPps = nextPps
            nextPps = ts.rampUp(nextPps)
            stepsToBreak += 1
            print(f"{lastPps}\t{nextPps}")
        print(f"steps: {stepsToBreak}")

        self.fail()

    def test_increase_sleep_time(self):
        ts = TwoStageExponentialAcceleration(PG35S_D48_HHC2(True))
        nextPps = ts.maxPps
        minPps = ts.stepperMotor.MIN_PPS + 10
        stepsToBreak = 0
        out = ""
        while minPps < nextPps:
            lastPps = nextPps
            nextPps = ts.rampDown(nextPps)
            stepsToBreak += 1
            out = f"{lastPps}\t{nextPps}\n" + out

        print(out)
        print(f"steps: {stepsToBreak}")
        self.fail()

    def test_should_break(self):
        self.fail()


class TestCustomAccelerationPerPps(TestCase):
    def test_take_closest_speed_transformation(self):
        transformations = [
            (200, 3.40625),
            (681.25, 1.1663990825688073),
            (794.609375, 1.10549601809065),
            (878.4375, 1.0534062611170403),
            (925.3515625000001, 1.049452013592807),
            (971.1120605468752, 1.0287096105136795),
            (998.9923095703127, 1.0234338007843597),
            (1022.4024963378909, 1.0212576881409023),
            (1044.1364097595217, 1)]
        ts = CustomAccelerationPerPps(PG35S_D48_HHC2(True), transformations)
        self.assertEquals(ts.takeClosestSpeedTransformation(681.25), 1.1663990825688073)
        print(ts.takeClosestSpeedTransformation(300))

