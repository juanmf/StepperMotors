from multiprocessing import Process, Queue, Value, Lock
from typing import Callable, List
import ctypes

from stepper_motors_juanmf1.AccelerationStrategy import (LinearAcceleration, AccelerationStrategy,
                                                         ExponentialAcceleration,
                                                         CustomAccelerationPerPps, DynamicDelayPlanner,
                                                         StaticDelayPlanner,
                                                         InteractiveAcceleration, DelayPlanner)
from stepper_motors_juanmf1.Controller import DRV8825MotorDriver, DriverSharedPositionStruct
from stepper_motors_juanmf1.Navigation import (DynamicNavigation, StaticNavigation, Navigation, BasicSynchronizedNavigation,
                                               SynchronizedNavigation)

from stepper_motors_juanmf1.BlockingQueueWorker import BlockingQueueWorker, MpQueue

from stepper_motors_juanmf1.EventDispatcher import EventDispatcher


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
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation,
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
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation,
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
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation,
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
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation,
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
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation,
                                  sleepGpioPin=sleepGpioPin,
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
        self.eventDispatcher = EventDispatcher()
        self._factoryOrders = []
        self.runningProcesses = []
        self._clientSharedMemory = []

    def setUpProcess(self) -> 'MultiProcessingControllerFactory':
        self._factoryOrders = []
        self._clientSharedMemory = []
        return self

    def withDriver(self, clientSharedMemory: list,
                   factoryFnReference: Callable, *args, **kwargs) -> 'MultiProcessingControllerFactory':
        """

        @param factoryFnReference: any of MultiProcessingControllerFactory.getMp** methods.
        @param clientSharedMemory: client specific MP shared memory objects to pass along. Final shared memory will have
        these appended AFTER standard ones as: [Lock, DriverSharedPositionStruct] + [clientSpecificSharedValues...]
        @return:
        """
        self._factoryOrders.append((factoryFnReference, (args, kwargs)))
        self._clientSharedMemory.append(clientSharedMemory)
        return self

    def spawn(self):
        if self._clientSharedMemory:
            assert len(self._clientSharedMemory) == len(self._factoryOrders)

        eventSharedMemory = self.eventDispatcher.getMpSharedMemory()

        queues = []
        sharedMemories = []
        for i, order in enumerate(self._factoryOrders):
            sharedMemory = []
            # Position updates
            positionValue = Value(DriverSharedPositionStruct)
            sharedLock = Lock()
            sharedMemory.extend([sharedLock, positionValue])
            if self._clientSharedMemory:
                sharedMemory.extend(self._clientSharedMemory[i])

            # Worker Queue
            queues.append(MpQueue())
            sharedMemories.append(sharedMemory)

        unpacker = self.Unpacker(self._factoryOrders, queues, sharedMemories, eventSharedMemory)
        childProcess = Process(target=unpacker.unpack)
        proxies = self.Unpacker.doUnpack(self._factoryOrders, queues, sharedMemories, isProxy=True)
        self.runningProcesses.append((childProcess, proxies))
        self._factoryOrders = []
        self._clientSharedMemory = []
        return proxies

    def getMpCustomTorqueCharacteristicsDRV8825With(self, queue, sharedMemory, isProxy, stepperMotor, directionPin, stepPin,
                                                    transformations=None,
                                                    sleepGpioPin=None,
                                                    stepsMode="Full",
                                                    modeGpioPins=None,
                                                    enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = CustomAccelerationPerPps(stepperMotor, delayPlanner, transformations=transformations)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation,
                                  sleepGpioPin=sleepGpioPin,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin,
                                  jobQueue=queue,
                                  sharedMemory=sharedMemory,
                                  isProxy=isProxy)

    def getMpLinearDRV8825With(self, queue, sharedMemory, isProxy, stepperMotor, directionPin, stepPin,
                               sleepGpioPin=None,
                               stepsMode="Full",
                               modeGpioPins=None,
                               enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = LinearAcceleration(stepperMotor, delayPlanner)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation,
                                  sleepGpioPin=sleepGpioPin,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin,
                                  jobQueue=queue,
                                  sharedMemory=sharedMemory,
                                  isProxy=isProxy)

    def getMpExponentialDRV8825With(self, queue, sharedMemory, isProxy, stepperMotor, directionPin, stepPin,
                                    sleepGpioPin=None,
                                    stepsMode="Full",
                                    modeGpioPins=None,
                                    enableGpioPin=None):
        delayPlanner = self.getDelayPlanner()
        navigation = self.getNavigation()
        acceleration = ExponentialAcceleration(stepperMotor, delayPlanner)
        return DRV8825MotorDriver(stepperMotor, acceleration, directionPin, stepPin, navigation,
                                  sleepGpioPin=sleepGpioPin,
                                  stepsMode=stepsMode,
                                  modeGpioPins=modeGpioPins,
                                  enableGpioPin=enableGpioPin,
                                  jobQueue=queue,
                                  sharedMemory=sharedMemory,
                                  isProxy=isProxy)

    class Unpacker:
        def __init__(self, factoryOrders, queues: list, sharedMemories: list, eventSharedMemory):
            self.factoryOrders = factoryOrders
            self.queues = queues
            self.sharedMemories = sharedMemories
            self.eventSharedMemory = eventSharedMemory
            self.eventDispatcher = None

        def unpack(self):
            """
            In new process, build Drivers
            @param factoryOrders:
            @param queues:
            @param sharedMemory:
            @return:
            """
            self.eventDispatcher = EventDispatcher(self.eventSharedMemory)
            drivers = self.doUnpack(self.factoryOrders, self.queues, self.sharedMemories, isProxy=False)
            # block main forever.
            drivers.pop().workerFuture.result()

        @staticmethod
        def doUnpack(factoryOrders, queues, sharedMemories, isProxy):
            drivers = []
            for index, order in enumerate(factoryOrders):
                drivers.append(
                    factoryOrders[0](queues[index], sharedMemories[index], isProxy,
                                     *factoryOrders[1][0], **factoryOrders[1][1]))
            return drivers
