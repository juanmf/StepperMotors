from AccelerationStrategy import (LinearAcceleration, AccelerationStrategy, ExponentialAcceleration,
                                  CustomAccelerationPerPps, DynamicDelayPlanner, StaticDelayPlanner,
                                  InteractiveAcceleration, DelayPlanner)
from Controller import DRV8825MotorDriver
from Navigation import DynamicNavigation, StaticNavigation, Navigation


class ControllerFactory:
    def getDelayPlanner(self) -> DelayPlanner:
        pass

    def getNavigation(self) -> Navigation:
        pass

    # Returns a controller that can't (de)accelerate.
    def getFlatDRV8825With(self, stepperMotor, directionPin, StepPin):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = AccelerationStrategy(stepperMotor, delayPlanner)
        delayPlanner.setAccelerationStrategy(acceleration)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, StepPin, navigation)

    def getLinearDRV8825With(self, stepperMotor, directionPin, StepPin):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = LinearAcceleration(stepperMotor, delayPlanner)
        delayPlanner.setAccelerationStrategy(acceleration)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, StepPin, navigation)

    def getExponentialDRV8825With(self, stepperMotor, directionPin, StepPin):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = ExponentialAcceleration(stepperMotor, delayPlanner)
        delayPlanner.setAccelerationStrategy(acceleration)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, StepPin, navigation)

    def getCustomTorqueCharacteristicsDRV8825With(self, stepperMotor, directionPin, StepPin, transformations=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = CustomAccelerationPerPps(stepperMotor, delayPlanner, transformations=transformations)
        delayPlanner.setAccelerationStrategy(acceleration)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, StepPin, navigation)

    def getInteractiveDRV8825With(self, stepperMotor, directionPin, StepPin, minSpeedDelta, minPps):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = InteractiveAcceleration(stepperMotor, delayPlanner, minSpeedDelta, minPps)
        delayPlanner.setAccelerationStrategy(acceleration)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, StepPin, navigation)


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
