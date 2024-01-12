# Static or responsive/dynamic navigation implementations.
from RPi import GPIO
from stepper_motors_juanmf1.myMath import cmp
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint


class Navigation:
    def go(self, controller, targetPosition, accelerationStrategy, fn, interruptPredicate):
        pass

    @staticmethod
    def pulseController(controller):
        # tprint(f"Setting step pin {controller.stepGpioPin} HIGH.")
        GPIO.output(controller.stepGpioPin, GPIO.HIGH)
        controller.usleep(controller.PULSE_TIME_MICROS)
        # tprint(f"Setting step pin {controller.stepGpioPin} LOW.")
        GPIO.output(controller.stepGpioPin, GPIO.LOW)

    @staticmethod
    def isInterruptible():
        pass


class StaticNavigation(Navigation):

    def go(self, controller, targetPosition, accelerationStrategy, fn, interruptPredicate):
        steps = abs(controller.currentPosition - targetPosition)
        # Todo: find out  if -1 works as LOW (normally set to 0) for direction pin.
        controller.setDirection(cmp(targetPosition, controller.currentPosition))
        for i in range(steps):
            self.pulseController(controller)

            accelerationStrategy.computeSleepTimeUs(i, steps)
            # tprint(f"Sleeping for {accelerationStrategy.currentSleepTimeUs} uS.")
            controller.usleep(accelerationStrategy.currentSleepTimeUs)
            # fn should not consume many CPU instructions to avoid delays between steps.
            fn(controller.currentPosition, targetPosition, accelerationStrategy.realDirection)

        accelerationStrategy.done()
        controller.setDirection(GPIO.LOW)

    @staticmethod
    def isInterruptible():
        return False


class DynamicNavigation(Navigation):

    def go(self, controller, targetPosition, accelerationStrategy, fn, interruptPredicate):
        # Can cross targetPosition with some speed > 0, that's not a final state.
        while not (controller.currentPosition == targetPosition and accelerationStrategy.canStop()):
            if interruptPredicate():
                # Intentionally leave isRunning True
                # tprint("Interrupting stepping job.")
                return
            # Direction is set in position based acceleration' state machine.
            controller.currentPosition = accelerationStrategy.computeSleepTimeUs(
                controller.currentPosition, targetPosition, lambda d: controller.setDirection(d))

            self.pulseController(controller)

            # tprint(f"Driver's self.currentPosition: {controller.currentPosition}. targetPosition: {targetPosition}")
            micros = accelerationStrategy.getCurrentSleepUs()

            controller.usleep(micros)
            # fn should not consume many CPU instructions to avoid delays between steps.
            fn(controller.currentPosition, targetPosition, accelerationStrategy.realDirection)
            # tprint("")

        # todo: check if still needed.
        accelerationStrategy.done()
        controller.setDirection(GPIO.LOW)

    @staticmethod
    def isInterruptible():
        return True


# Todo: In applications where several motors need to work together in close coordination, like 3D printing, we can't
#   just have a set of independent drivers timing their pulses to their convenience. This Navigation ensures steps are
#   locked into a rithm.
class SynchronizedNavigation(Navigation):
    def __init__(self):
        self.synchronizedControllers = []

    def addControllerToSynchronize(self, controller):
        self.synchronizedControllers.append(controller)

    @staticmethod
    def pulseController(controller):
        # pulse all controllers at once.
        pass

    # Todo: this will be called from multiple threads, one per driver. Handle synchronization accordingly.
    # Waits to get all Synchronized controllers to go then coordinates their steps so that all sleep and step at once.
    def go(self, controller, targetPosition, accelerationStrategy, fn, interruptPredicate):
        # Have targetPosition be an array of targets, one per registered Driver.
        # step with all remaining motors at once. When target position is met at different times, remove done drivers
        # from loop.
        pass

    @staticmethod
    def isInterruptible():
        return False
