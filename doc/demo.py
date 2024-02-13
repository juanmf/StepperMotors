import ctypes
from functools import partial
from multiprocess import Value, Event

from src.stepper_motors_juanmf1.Controller import DRV8825MotorDriver
from src.stepper_motors_juanmf1.ControllerFactory import MultiProcessingControllerFactory, ControllerFactory
from src.stepper_motors_juanmf1.EventDispatcher import EventDispatcher, MultiprocessObserver
from src.stepper_motors_juanmf1.StepperMotor import PG35S_D48_HHC2


class PolarCoordinatesSample:

    """
    Initialization:
      Register even Listeners.
      Start motor driver(s) in child process.
      Setup MultiprocessObserver with publisher/observer callback pairs.
    """
    def __init__(self, controllerFactory: MultiProcessingControllerFactory):
        self.fireReadyEventsAwaited = [
            "azimuthAimingCompletesteppingCompleteAdvance",
            "azimuthAimingCompletesteppingCompleteFinalStep"
        ]
        self.azimuthDriver = None
        self.initDrivers(controllerFactory)
        EventDispatcher.instance().register(self.fireReadyEventsAwaited, self.fireReadyEventHandler)

    def initDrivers(self, controllerFactory):
        if isinstance(controllerFactory, MultiProcessingControllerFactory):
            self._initMpDrivers(controllerFactory)
        else:
            # Not used in this example.
            self._initLocalDrivers(controllerFactory)

    def _initMpDrivers(self, controllerFactory: MultiProcessingControllerFactory):
        azimuthDriverShared = Value(ctypes.c_int, 0)

        azimuthObserver = MultiprocessObserver(eventObserver=partial(self.childProcessEventObserver, "azimuth"),
                                               eventPublisher=self.sharedDataProcessing,
                                               sharedMemory=azimuthDriverShared)

        self.azimuthDriver = (controllerFactory
                .setUpProcess()
                .withDriver(multiprocessObserver=azimuthObserver,
                            factoryFnReference=controllerFactory.getMpCustomTorqueCharacteristicsDRV8825With,
                            stepperMotor=PG35S_D48_HHC2(True), directionPin=13,
                            stepPin=19, sleepGpioPin=12)
                # Can put orders to build more drivers here .withDriver(...).withDriver(...)...
                .spawn())

    def childProcessEventObserver(self, name, sharedInt):
        """
        Multiprocess event observer. Runs in parent process in dedicated Worker thread. Blocks until Events fire.
        """
        print(f"Parent process notified every {name}'s 100th step. Step: {sharedInt.value}")

    @staticmethod
    def sharedDataProcessing(sharedMemory, currentPosition):
        # Client process knows what's in sharedMemory, not Drivers.
        # Prints every 100 steps.
        print(f"Running callback from Navigation stepping in child process \n "
              f"{currentPosition, sharedMemory}")

    def _initLocalDrivers(self, controllerFactory: ControllerFactory):
        """
        Not used in multiprocess scenario. Regular Factory, simplest approach for comparison with MP approach.
        """
        self.azimuthDriver: DRV8825MotorDriver = controllerFactory.getCustomTorqueCharacteristicsDRV8825With(
            PG35S_D48_HHC2(True), directionPin=13, stepPin=19, sleepGpioPin=12)

    """
    Setup done, usage follows
    """

    def operateDrivers(self, azimuthDelta, elevationDelta):
        eventNamePrefix = "AimingComplete"
        # Events will result in 4 combinations: "[azimuth|elevation]AimingCompletesteppingComplete[Advance|FinalStep]"
        if azimuthDelta != 0:
            azimuthJob = self.azimuthDriver.signedSteps(azimuthDelta, self.steppingCallback,
                                                        jobCompleteEventNamePrefix="azimuth" + eventNamePrefix)
            self.awaitAzimuthReadyEvents()

    @staticmethod
    def steppingCallback(currentPosition, targetPosition, direction,
                         multiprocessObserver: MultiprocessObserver = None):
        """
        Called each step of the motor.
        Uses :func:`~MultiprocessObserver.eventPublisher` to notify MainProcess when custom conditions met.
        Contract:
          fn(pulsingController.controller.getCurrentPosition(),
             pulsingController.targetPosition,
             pulsingController.controller.accelerationStrategy.realDirection,
             pulsingController.controller.multiprocessObserver)  # Only provided in multiprocessing scenario. Method
                                                                 # runs in child process.
        """
        if multiprocessObserver:
            # Important, as in local process scenario sharedMemory is None. see :func:`~self._initLocalDrivers()`
            multiprocessObserver._sharedMemory.value += 1
            if multiprocessObserver._sharedMemory.value % 100 == 0:
                # every 100th step, signals Parent process to read multiprocessObserver._sharedMemory
                # eventPublisher runs `self.childProcessEventObserver()` provided callback with a lock and clears Event
                MultiprocessObserver.eventPublisher(multiprocessObserver, currentPosition)

    def awaitAzimuthReadyEvents(self):
        if not "azimuthAimingCompletesteppingComplete" in self.fireReadyEventsAwaited:
            self.fireReadyEventsAwaited.append("azimuthAimingCompletesteppingCompleteAdvance")
            self.fireReadyEventsAwaited.append("azimuthAimingCompletesteppingCompleteFinalStep")

    def fireReadyEventHandler(self, calleeId, eventInfo):
        """
        Method registered with EventDispatcher.
        @param calleeId: EventDispatcher creates a uuid for each callable you can use to unregister it.
        @param eventInfo: custom.
        @return:
        """
        ed = EventDispatcher.instance()
        # does custom checks on eventInfo and fire App level event
        ed.publishMainLoop("MyBusinessLogicEventMotorDone", {'isReady': True})
