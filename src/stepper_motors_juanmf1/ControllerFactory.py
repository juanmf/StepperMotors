import multiprocess as mp
from multiprocess import Process, Value, Lock

from typing import Callable

from stepper_motors_juanmf1.AccelerationStrategy import (LinearAcceleration, AccelerationStrategy,
                                                         ExponentialAcceleration,
                                                         CustomAccelerationPerPps, DynamicDelayPlanner,
                                                         StaticDelayPlanner,
                                                         InteractiveAcceleration, DelayPlanner)
from stepper_motors_juanmf1.Controller import DRV8825MotorDriver, TMC2209MotorDriver, DriverSharedPositionStruct, MotorDriver
from stepper_motors_juanmf1.Navigation import (DynamicNavigation, StaticNavigation, Navigation,
                                               BasicSynchronizedNavigation)

from stepper_motors_juanmf1.BlockingQueueWorker import MpQueue
from stepper_motors_juanmf1.EventDispatcher import EventDispatcher, MultiprocessObserver
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint, flush_streams_if_not_empty


class ControllerFactory:
    def getDelayPlanner(self) -> DelayPlanner:
        pass

    def getNavigation(self) -> Navigation:
        pass

    # Returns a controller that can't (de)accelerate.
    def getFlatDRV8825With(self, stepperMotor, directionPin, stepPin, sleepGpioPin=None,
                           stepsMode="Full",
                           modeGpioPins=None,
                           enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = AccelerationStrategy(stepperMotor, delayPlanner)
        return DRV8825MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  sleepGpioPin=sleepGpioPin,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin)

    def getLinearDRV8825With(self, stepperMotor, directionPin, stepPin, sleepGpioPin=None,
                             stepsMode="Full",
                             modeGpioPins=None,
                             enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = LinearAcceleration(stepperMotor, delayPlanner)
        return DRV8825MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  sleepGpioPin=sleepGpioPin,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin)

    def getExponentialDRV8825With(self, stepperMotor, directionPin, stepPin, sleepGpioPin=None,
                                  stepsMode="Full",
                                  modeGpioPins=None,
                                  enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = ExponentialAcceleration(stepperMotor, delayPlanner)
        return DRV8825MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  sleepGpioPin=sleepGpioPin,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin)

    def getCustomTorqueCharacteristicsDRV8825With(self, stepperMotor, directionPin, stepPin, transformations=None,
                                                  sleepGpioPin=None,
                                                  stepsMode="Full",
                                                  modeGpioPins=None,
                                                  enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = CustomAccelerationPerPps(stepperMotor, delayPlanner, transformations=transformations)
        return DRV8825MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  sleepGpioPin=sleepGpioPin,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin)

    def getInteractiveDRV8825With(self, stepperMotor, directionPin, stepPin, minSpeedDelta, minPps, sleepGpioPin=None,
                                  stepsMode="Full",
                                  modeGpioPins=None,
                                  enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = InteractiveAcceleration(stepperMotor, delayPlanner, minSpeedDelta, minPps)
        return DRV8825MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  sleepGpioPin=sleepGpioPin,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin)

    def getFlatTCM2209With(self, stepperMotor, directionPin, stepPin,
                           stepsMode="Full",
                           modeGpioPins=None,
                           enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = AccelerationStrategy(stepperMotor, delayPlanner)
        return TMC2209MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin)

    def getLinearTCM2209With(self, stepperMotor, directionPin, stepPin,
                             stepsMode="Full",
                             modeGpioPins=None,
                             enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = LinearAcceleration(stepperMotor, delayPlanner)
        return TMC2209MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin)

    def getExponentialTCM2209With(self, stepperMotor, directionPin, stepPin,
                                  stepsMode="Full",
                                  modeGpioPins=None,
                                  enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = ExponentialAcceleration(stepperMotor, delayPlanner)
        return TMC2209MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin)

    def getCustomTorqueCharacteristicsTCM2209With(self, stepperMotor, directionPin, stepPin, transformations=None,
                                                  stepsMode="Full",
                                                  modeGpioPins=None,
                                                  enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = CustomAccelerationPerPps(stepperMotor, delayPlanner, transformations=transformations)
        return TMC2209MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin)

    def getInteractiveTCM2209With(self, stepperMotor, directionPin, stepPin, minSpeedDelta, minPps,
                                  stepsMode="Full",
                                  modeGpioPins=None,
                                  enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = InteractiveAcceleration(stepperMotor, delayPlanner, minSpeedDelta, minPps)
        return TMC2209MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin)


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


class SynchronizedControllerFactory(ControllerFactory):

    def getDelayPlanner(self):
        return DynamicDelayPlanner()

    def getNavigation(self):
        return BasicSynchronizedNavigation()


class MultiProcessingControllerFactory(SynchronizedControllerFactory):

    def __init__(self):
        self.eventDispatcher = EventDispatcher.instance()
        self._factoryOrders = []
        self.runningProcesses = []
        self._clientMultiprocessObservers = []

    def setUpProcess(self) -> 'MultiProcessingControllerFactory':
        self._factoryOrders = []
        self._clientMultiprocessObservers = []
        return self

    def withDriver(self, multiprocessObserver: MultiprocessObserver,
                   factoryFnReference: Callable, *args, **kwargs) -> 'MultiProcessingControllerFactory':
        """

        @param factoryFnReference: any of MultiProcessingControllerFactory.getMp** methods.
        @param multiprocessObserver: client specific MP shared memory and handlers to pass along. Final shared memory
                                     will have these appended AFTER standard ones as:
                                     [Lock, DriverSharedPositionStruct, multiprocessObserver]
        @return: this factory
        """
        self._factoryOrders.append((factoryFnReference, (args, kwargs)))
        self._clientMultiprocessObservers.append(multiprocessObserver)
        return self

    def spawn(self):
        """
        @return: A list of proxy drivers you can use on MainProcess to send stepping jobs to counterpart drivers in
        child process.
        """
        if self._clientMultiprocessObservers:
            assert len(self._clientMultiprocessObservers) == len(self._factoryOrders)

        queues = []
        sharedMemories = []
        for i, order in enumerate(self._factoryOrders):
            sharedLock = Lock()
            sharedMemory = []
            # Position updates
            positionValue = Value(DriverSharedPositionStruct, 0, 0, lock=True)
            sharedMemory.extend([sharedLock, positionValue])
            if self._clientMultiprocessObservers:
                sharedMemory.append(self._clientMultiprocessObservers[i])

            # Worker Queue
            queues.append(MpQueue())
            sharedMemories.append(sharedMemory)

        eventsMultiprocessObserver = self.eventDispatcher.getMultiprocessObserver()

        proxies = self.Unpacker.doUnpack(self._factoryOrders, queues, sharedMemories, isProxy=True)
        jobCompletionObservers = [driver.jobCompletionObserver for driver in proxies]
        unpacker = self.Unpacker(self._factoryOrders, queues, sharedMemories, eventsMultiprocessObserver)
        childProcess = Process(target=unpacker.unpack, args=(jobCompletionObservers,))
        childProcess.daemon = True
        flush_streams_if_not_empty()
        childProcess.start()
        self.runningProcesses.append((childProcess, proxies))
        self._factoryOrders = []
        self._clientMultiprocessObservers = []
        return proxies

    def getMpCustomTorqueCharacteristicsDRV8825With(self, queue, sharedMemory, isProxy, jobCompletionObserver,
                                                    stepperMotor,
                                                    directionPin,
                                                    stepPin,
                                                    transformations=None,
                                                    sleepGpioPin=None,
                                                    stepsMode="Full",
                                                    modeGpioPins=None,
                                                    enableGpioPin=None,
                                                    steppingCompleteEventName="steppingComplete"):

        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = CustomAccelerationPerPps(stepperMotor, delayPlanner, transformations=transformations)

        driver = DRV8825MotorDriver(stepperMotor=stepperMotor,
                                    accelerationStrategy=acceleration,
                                    directionGpioPin=directionPin,
                                    stepGpioPin=stepPin,
                                    navigation=navigation,
                                    sleepGpioPin=sleepGpioPin,
                                    stepsMode=stepsMode,
                                    modeGpioPins=modeGpioPins,
                                    enableGpioPin=enableGpioPin,
                                    jobQueue=queue,
                                    sharedMemory=sharedMemory,
                                    isProxy=isProxy,
                                    steppingCompleteEventName=steppingCompleteEventName,
                                    jobCompletionObserver=jobCompletionObserver)

        print(f"driver is proxy? {isProxy} {driver} {driver.getJobQueue()}")
        return driver

    def getMpLinearDRV8825With(self, queue, sharedMemory, isProxy, jobCompletionObserver,
                               stepperMotor,
                               directionPin,
                               stepPin,
                               sleepGpioPin=None,
                               stepsMode="Full",
                               modeGpioPins=None,
                               enableGpioPin=None,
                               steppingCompleteEventName="steppingComplete"):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = LinearAcceleration(stepperMotor, delayPlanner)
        return DRV8825MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  sleepGpioPin=sleepGpioPin,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin,
                                  jobQueue=queue,
                                  sharedMemory=sharedMemory,
                                  isProxy=isProxy,
                                  steppingCompleteEventName=steppingCompleteEventName,
                                  jobCompletionObserver=jobCompletionObserver)

    def getMpExponentialDRV8825With(self, queue, sharedMemory, isProxy, jobCompletionObserver,
                                    stepperMotor,
                                    directionPin,
                                    stepPin,
                                    sleepGpioPin=None,
                                    stepsMode="Full",
                                    modeGpioPins=None,
                                    enableGpioPin=None,
                                    steppingCompleteEventName="steppingComplete"):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = ExponentialAcceleration(stepperMotor, delayPlanner)
        return DRV8825MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  sleepGpioPin=sleepGpioPin,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin,
                                  jobQueue=queue,
                                  sharedMemory=sharedMemory,
                                  isProxy=isProxy,
                                  steppingCompleteEventName=steppingCompleteEventName,
                                  jobCompletionObserver=jobCompletionObserver)

    def getMpCustomTorqueCharacteristicsTMC2209With(self, queue, sharedMemory, isProxy, jobCompletionObserver,
                                                    stepperMotor,
                                                    directionPin,
                                                    stepPin,
                                                    transformations=None,
                                                    stepsMode="Full",
                                                    modeGpioPins=None,
                                                    enableGpioPin=None,
                                                    steppingCompleteEventName="steppingComplete"):

        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = CustomAccelerationPerPps(stepperMotor, delayPlanner, transformations=transformations)

        driver = TMC2209MotorDriver(stepperMotor=stepperMotor,
                                    accelerationStrategy=acceleration,
                                    directionGpioPin=directionPin,
                                    stepGpioPin=stepPin,
                                    navigation=navigation,
                                    stepsMode=stepsMode,
                                    modeGpioPins=modeGpioPins,
                                    enableGpioPin=enableGpioPin,
                                    jobQueue=queue,
                                    sharedMemory=sharedMemory,
                                    isProxy=isProxy,
                                    steppingCompleteEventName=steppingCompleteEventName,
                                    jobCompletionObserver=jobCompletionObserver)

        return driver

    def getMpLinearTMC2209With(self, queue, sharedMemory, isProxy, jobCompletionObserver,
                               stepperMotor,
                               directionPin,
                               stepPin,
                               stepsMode="Full",
                               modeGpioPins=None,
                               enableGpioPin=None,
                               steppingCompleteEventName="steppingComplete"):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = LinearAcceleration(stepperMotor, delayPlanner)
        return TMC2209MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin,
                                  jobQueue=queue,
                                  sharedMemory=sharedMemory,
                                  isProxy=isProxy,
                                  steppingCompleteEventName=steppingCompleteEventName,
                                  jobCompletionObserver=jobCompletionObserver)

    def getMpExponentialTMC2209With(self, queue, sharedMemory, isProxy, jobCompletionObserver,
                                    stepperMotor,
                                    directionPin,
                                    stepPin,
                                    stepsMode="Full",
                                    modeGpioPins=None,
                                    enableGpioPin=None,
                                    steppingCompleteEventName="steppingComplete"):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = ExponentialAcceleration(stepperMotor, delayPlanner)
        return TMC2209MotorDriver(stepperMotor=stepperMotor,
                                  accelerationStrategy=acceleration,
                                  directionGpioPin=directionPin,
                                  stepGpioPin=stepPin,
                                  navigation=navigation,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin,
                                  jobQueue=queue,
                                  sharedMemory=sharedMemory,
                                  isProxy=isProxy,
                                  steppingCompleteEventName=steppingCompleteEventName,
                                  jobCompletionObserver=jobCompletionObserver)

    class Unpacker:
        def __init__(self, factoryOrders, queues: list, sharedMemories: list, eventMultiprocessObserver):
            self.factoryOrders = factoryOrders
            self.queues = queues
            self.sharedMemories = sharedMemories
            self.multiprocessObserver = eventMultiprocessObserver
            self.eventDispatcher = None

        def unpack(self, jobCompletionObservers):
            """
            In new process, build Drivers
            @param factoryOrders:
            @param queues:
            @param sharedMemory:
            @return:
            """
            print(f"Unpacking in child process!! {self.factoryOrders} {self.queues} {self.sharedMemories}")
            # Removing Singletons instances in case its state was cloned from Parent Process:
            EventDispatcher._instance = None
            BasicSynchronizedNavigation._instance = None
            DynamicDelayPlanner.Rest._instance = None
            DynamicDelayPlanner.Steady._instance = None
            DynamicDelayPlanner.RampingUp._instance = None
            DynamicDelayPlanner.RampingDown._instance = None

            self.eventDispatcher = EventDispatcher.instance(self.multiprocessObserver)
            tprint("Unpacker: Instantiating EventDispatcher with shared memory", self.multiprocessObserver,
                   self.eventDispatcher, self.eventDispatcher._multiprocessObserver,
                   self.eventDispatcher._shouldDispatchToParentProcess, jobCompletionObservers)
            drivers = self.doUnpack(self.factoryOrders, self.queues, self.sharedMemories, isProxy=False,
                                    jobCompletionObservers=jobCompletionObservers)
            # block main forever.
            drivers.pop().workerFuture.result()

        @staticmethod
        def doUnpack(factoryOrders, queues, sharedMemories, isProxy, jobCompletionObservers=None) -> list[MotorDriver]:
            drivers = []
            for index, order in enumerate(factoryOrders):
                if jobCompletionObservers:
                    drivers.append(
                        order[0](queues[index], sharedMemories[index], isProxy, jobCompletionObservers[index],
                                 *order[1][0], **order[1][1]))
                else:
                    drivers.append(
                        order[0](queues[index], sharedMemories[index], isProxy, None,
                                 *order[1][0], **order[1][1]))
            return drivers
