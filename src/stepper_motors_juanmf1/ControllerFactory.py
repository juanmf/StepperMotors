from stepper_motors_juanmf1.AccelerationStrategy import (LinearAcceleration, AccelerationStrategy,
                                                         ExponentialAcceleration,
                                                         CustomAccelerationPerPps, DynamicDelayPlanner,
                                                         StaticDelayPlanner,
                                                         InteractiveAcceleration, DelayPlanner)
from stepper_motors_juanmf1.Controller import DRV8825MotorDriver
from stepper_motors_juanmf1.Navigation import DynamicNavigation, StaticNavigation, Navigation


class ControllerFactory:
    def getDelayPlanner(self) -> DelayPlanner:
        pass

    def getNavigation(self) -> Navigation:
        pass

    # Returns a controller that can't (de)accelerate.
    def getFlatDRV8825With(self, stepperMotor, directionPin, stepPin):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = AccelerationStrategy(stepperMotor, delayPlanner)
        delayPlanner.setAccelerationStrategy(acceleration)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation)

    def getLinearDRV8825With(self, stepperMotor, directionPin, stepPin):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = LinearAcceleration(stepperMotor, delayPlanner)
        delayPlanner.setAccelerationStrategy(acceleration)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation)

    def getExponentialDRV8825With(self, stepperMotor, directionPin, stepPin):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = ExponentialAcceleration(stepperMotor, delayPlanner)
        delayPlanner.setAccelerationStrategy(acceleration)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation)

    def getCustomTorqueCharacteristicsDRV8825With(self, stepperMotor, directionPin, stepPin, transformations=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = CustomAccelerationPerPps(stepperMotor, delayPlanner, transformations=transformations)
        delayPlanner.setAccelerationStrategy(acceleration)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation)

    def getInteractiveDRV8825With(self, stepperMotor, directionPin, stepPin, minSpeedDelta, minPps):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = InteractiveAcceleration(stepperMotor, delayPlanner, minSpeedDelta, minPps)
        delayPlanner.setAccelerationStrategy(acceleration)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation)


class StaticControllerFactory(ControllerFactory):
    def getDelayPlanner(self):
        return StaticDelayPlanner()

    def getNavigation(self):
        return StaticNavigation()


class DynamicControllerFactory(ControllerFactory):
    def getDelayPlanner(self):
        return DynamicDelayPlanner()

    def getNavigation(self):
        return DynamicNavigation()
