import inspect
import queue
from abc import abstractmethod

import multiprocess as mp
from multiprocess import Process, Value, Lock

from typing import Callable

from stepper_motors_juanmf1.AccelerationStrategy import (LinearAcceleration, AccelerationStrategy,
                                                         ExponentialAcceleration,
                                                         CustomAccelerationPerPps, DynamicDelayPlanner,
                                                         StaticDelayPlanner,
                                                         InteractiveAcceleration, DelayPlanner)

from adafruit_motor.stepper import StepperMotor as AdafruitStepperDriver
from stepper_motors_juanmf1.AdafruitControllerAdapter import AdafruitStepperDriverAdapter

from stepper_motors_juanmf1.Controller import (DRV8825MotorDriver, TMC2209StandaloneMotorDriver,
                                               DriverSharedPositionStruct, MotorDriver, BipolarStepperMotorDriver)
from stepper_motors_juanmf1.Navigation import (DynamicNavigation, StaticNavigation, Navigation,
                                               BasicSynchronizedNavigation)

from stepper_motors_juanmf1.BlockingQueueWorker import MpQueue
from stepper_motors_juanmf1.EventDispatcher import EventDispatcher, MultiprocessObserver
from stepper_motors_juanmf1.StepperMotor import StepperMotor
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint, flush_streams_if_not_empty
from stepper_motors_juanmf1.UnipolarController import UnipolarMotorDriver, ULN2003MotorDriver


class ControllerBuilder:
    def __init__(self, fromBuilder: 'ControllerBuilder' = None):
        self.stepperMotor: StepperMotor or None = None

        # Generic Pinout
        self.directionGpioPin: int or None = None
        # Unipolar uses a list of 4 step pins
        self.stepGpioPin: int or list = None
        self.sleepGpioPin: int or None = None
        self.enableGpioPin: int or None = None
        # Related to Sleep/Enable
        self.useHoldingTorque: bool or None = None

        self.modeGpioPins: list or None = None

        # ThirdParty Adapted libs might use int as step styles
        self.stepsMode: str or int = None

        self.transformations: list[tuple] or None = None
        self.accelerationStrategy: AccelerationStrategy or None = None
        # Navigation style config
        self.navigation: Navigation or None = None
        self.delayPlanner: DelayPlanner or None = None
        self.built = False

        self.workerName: str or None = None
        self.jobQueueMaxSize = 2
        # multiprocess
        self.jobQueue: queue.Queue or MpQueue = None
        self.sharedMemory: list or None = None
        self.isProxy = False
        self.jobCompletionObserver: MultiprocessObserver or None = None

        # This one does not have a constructor argument, goes in sharedMemory[2]
        self.clientMultiprocessObserver: MultiprocessObserver or None = None

        self.steppingCompleteEventName = "steppingComplete"

        # TMC22XX specific
        self.autoResetOnError = False
        # CounterClockWise is home.
        self.homeDirection = -1
        self.indexGpioPin: int or None = None

        self.stepsPerIndexPulse = 4
        self.spreadGpioPin: int or None = None
        self.diagGpioPin: int or None = None

        # Adafruit
        self.adafruitDriver: AdafruitStepperDriver or None = None

        if fromBuilder:
            self.copyFromBuilder(fromBuilder)

    def copyFromBuilder(self, fromBuilder: 'ControllerBuilder') -> 'ControllerBuilder':
        """
        Only copy values that would not cause conflict. e.g. skipping pins.
        @param fromBuilder:
        @return:
        """
        kwargs = fromBuilder._driverConstructorKwArgs()
        conflictFields = [
            'stepperMotor',
            'directionGpioPin',
            'stepGpioPin',
            'sleepGpioPin',
            'enableGpioPin',
            'modeGpioPins',
            'accelerationStrategy',
            'navigation',
            'delayPlanner',
            'built',
            'jobQueue',
            'sharedMemory',
            'isProxy',
            'jobCompletionObserver',
            'indexGpioPin',
            'spreadGpioPin',
            'diagGpioPin',
            'adafruitDriver',
        ]

        copiedValues = {key: kwargs[key] for key in kwargs if key not in conflictFields}
        for key, value in copiedValues.items():
            setattr(self, key, value)
        return self

    """
    Builder pattern Fluent API
    """

    def withClientMultiprocessObserver(self, clientMultiprocessObserver: MultiprocessObserver):
        self.clientMultiprocessObserver = clientMultiprocessObserver
        return self

    def withsteppingCompleteEventName(self, steppingCompleteEventName):
        self.steppingCompleteEventName = steppingCompleteEventName
        return self

    def withWorkerName(self, name):
        self.workerName = name
        return self

    def withTMCPerks(self, *,
                     autoResetOnError=False,
                     homeDirection=-1,
                     indexGpioPin: int or None = None,
                     stepsPerIndexPulse=4,
                     spreadGpioPin: int or None = None,
                     diagGpioPin: int or None = None):
        self.autoResetOnError = autoResetOnError
        self.homeDirection = homeDirection
        self.indexGpioPin = indexGpioPin
        self.stepsPerIndexPulse = stepsPerIndexPulse
        self.spreadGpioPin = spreadGpioPin
        self.diagGpioPin = diagGpioPin

    def withSharedMemory(self, sharedMemory: list):
        self.sharedMemory = sharedMemory
        return self

    def withJobCompletionObserver(self, jobCompletionObserver: MultiprocessObserver):
        self.jobCompletionObserver = jobCompletionObserver
        return self

    def withJobQueue(self, jobQueue: queue.Queue or MpQueue):
        self.jobQueue = jobQueue
        return self

    def isMultiprocessProxy(self, isProxy):
        self.isProxy = isProxy
        return self

    def withHoldingTorqueEnabled(self, enabled=True):
        self.useHoldingTorque = enabled
        return self

    def withsteppingMode(self, stepsMode):
        self.stepsMode = stepsMode
        return self

    def withStepperMotor(self, stepperMotor: StepperMotor):
        self.stepperMotor = stepperMotor
        return self

    def withPins(self, *, directionGpioPin=None, stepGpioPin=None, enableGpioPin=None, sleepGpioPin=None):
        self.directionGpioPin = directionGpioPin
        self.stepGpioPin = stepGpioPin
        self.enableGpioPin = enableGpioPin
        self.sleepGpioPin = sleepGpioPin
        return self

    def withsteppingModePins(self, modeGpioPins):
        self.modeGpioPins = modeGpioPins
        return self

    def withTorqueCurve(self, transformations: list):
        """
        @param transformations: list of tuples
                [(minPPS, maxIncrementPPS_1), (minPPS + maxIncrementPPS_1, maxIncrementPPS_2),...(maxPPS, 0)]
                in Full step mode, where 1 pulse == 1 step
        see Benchmark module
        @return:
        """
        self.transformations = transformations
        return self

    def withAdafruitDriver(self, adafruitDriver: AdafruitStepperDriver):
        self.adafruitDriver = adafruitDriver
        return self

    def _accelerationStrategyConstructorArgs(self):
        assert self.stepperMotor and self.delayPlanner and self.stepsMode
        return {'stepperMotor': self.stepperMotor,
                'delayPlanner': self.delayPlanner,
                'steppingModeMultiple': BipolarStepperMotorDriver.RESOLUTION_MULTIPLE[self.stepsMode]}

    def withNoAcceleration(self):
        self.accelerationStrategy = AccelerationStrategy(**self._accelerationStrategyConstructorArgs())
        return self

    def withLinearAcceleration(self):
        self.accelerationStrategy = LinearAcceleration(**self._accelerationStrategyConstructorArgs())
        return self

    def withExponentialAcceleration(self):
        self.accelerationStrategy = ExponentialAcceleration(**self._accelerationStrategyConstructorArgs())
        return self

    def withCustomTorqueCurveAccelerationAcceleration(self):
        """
        CustomTorqueCurve refers to max increment in PPS at each PPS from minPPS to maxPPS
        """
        self.accelerationStrategy = CustomAccelerationPerPps(**self._accelerationStrategyConstructorArgs(),
                                                             transformations=self.transformations)
        return self

    def withInteractiveAcceleration(self, minSpeedDelta, minPPS):
        self.accelerationStrategy = InteractiveAcceleration(**self._accelerationStrategyConstructorArgs(),
                                                            minSpeedDelta=minSpeedDelta, minPPS=minPPS)
        return self

    def withNavigationStyleStatic(self):
        self.navigation = StaticNavigation()
        self.delayPlanner = StaticDelayPlanner()
        return self

    def withNavigationStyleDynamic(self):
        self.navigation = DynamicNavigation()
        self.delayPlanner = DynamicDelayPlanner()
        return self

    def withNavigationStyleSynchronized(self, newMultitonKey=0):
        """
        Initializes a set of Navigation and DelayPlain using BasicSynchronizedNavigation. Keep in mind
        BasicSynchronizedNavigation is Singleton and can be initialized only once, setting the high and
        low values that will be applied to all pulses sent to Drivers (except for ThirdPartyAdapter ones).
        """
        self.navigation = BasicSynchronizedNavigation.getInstance(newMultitonKey=newMultitonKey)
        self.delayPlanner = DynamicDelayPlanner()
        return self

    """
    Builder methods
    """

    def _driverConstructorKwArgs(self, classToConstruct=None):
        if not self.delayPlanner:
            self.withNavigationStyleStatic()
        if not self.accelerationStrategy:
            self.withNoAcceleration()
            
        allParams = {
            'stepperMotor': self.stepperMotor,
            'directionGpioPin': self.directionGpioPin,
            'stepGpioPin': self.stepGpioPin,
            'sleepGpioPin': self.sleepGpioPin,
            'enableGpioPin': self.enableGpioPin,
            'useHoldingTorque': self.useHoldingTorque,
            'modeGpioPins': self.modeGpioPins,
            'stepsMode': self.stepsMode,
            'transformations': self.transformations,
            'accelerationStrategy': self.accelerationStrategy,
            'navigation': self.navigation,
            'delayPlanner': self.delayPlanner,
            'built': self.built,
            'workerName': self.workerName,
            'jobQueueMaxSize': self.jobQueueMaxSize,
            'jobQueue': self.jobQueue,
            'sharedMemory': self.sharedMemory,
            'isProxy': self.isProxy,
            'jobCompletionObserver': self.jobCompletionObserver,
            'steppingCompleteEventName': self.steppingCompleteEventName,
            'autoResetOnError': self.autoResetOnError,
            'homeDirection': self.homeDirection,
            'indexGpioPin': self.indexGpioPin,
            'stepsPerIndexPulse': self.stepsPerIndexPulse,
            'spreadGpioPin': self.spreadGpioPin,
            'diagGpioPin': self.diagGpioPin,
            'adafruitDriver': self.adafruitDriver,
        }
        if classToConstruct is None:
            return allParams
        validParams = list(inspect.signature(classToConstruct.__init__).parameters)
        constructorKwArgs = {key: allParams[key] for key in allParams if key in validParams}
        return constructorKwArgs

    def buildDRV8825Driver(self) -> DRV8825MotorDriver:
        assert not self.built
        driver = DRV8825MotorDriver(**self._driverConstructorKwArgs(DRV8825MotorDriver))
        self.built = True
        return driver

    def buildUnipolarDriver(self) -> UnipolarMotorDriver:
        assert not self.built
        driver = UnipolarMotorDriver(**self._driverConstructorKwArgs(UnipolarMotorDriver))
        self.built = True
        return driver

    def buildUNL2003Driver(self) -> UnipolarMotorDriver:
        """
        Same behavior as buildUnipolarDriver()
        @return: ULN2003MotorDriver
        """
        assert not self.built
        driver = ULN2003MotorDriver(**self._driverConstructorKwArgs(UnipolarMotorDriver))
        self.built = True
        return driver

    def buildTMC2209StandaloneDriver(self) -> TMC2209StandaloneMotorDriver:
        assert not self.built
        driver = TMC2209StandaloneMotorDriver(**self._driverConstructorKwArgs(TMC2209StandaloneMotorDriver))
        self.built = True
        return driver

    def buildAdafruitStepperDriverAdapter(self) -> AdafruitStepperDriverAdapter:
        assert not self.built
        driver = AdafruitStepperDriverAdapter(**self._driverConstructorKwArgs(AdafruitStepperDriverAdapter))
        self.built = True
        return driver

    @staticmethod
    def getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin=None,
                        stepsMode="Full",
                        modeGpioPins=None,
                        enableGpioPin=None) -> 'ControllerBuilder':
        return (ControllerBuilder().withPins(directionGpioPin=directionGpioPin, stepGpioPin=stepGpioPin,
                                             enableGpioPin=enableGpioPin, sleepGpioPin=sleepGpioPin)
                 .withStepperMotor(stepperMotor)
                 .withsteppingMode(stepsMode)
                 .withsteppingModePins(modeGpioPins))


class ControllerFactory:

    def getDelayPlanner(self) -> DelayPlanner:
        pass

    def getNavigation(self) -> Navigation:
        pass

    def getBasicBuilder(self, stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin=None,
                        stepsMode="Full",
                        modeGpioPins=None,
                        enableGpioPin=None,
                        syncNavigationMultitonKey=0) -> ControllerBuilder:
        builder = ControllerBuilder.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin,
                                                    stepsMode, modeGpioPins, enableGpioPin)
        return self.addNavigationStyle(builder, syncNavigationMultitonKey)

    # Returns a controller that can't (de)accelerate.
    def getFlatDRV8825With(self, stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin=None,
                           stepsMode="Full",
                           modeGpioPins=None,
                           enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin, stepsMode, modeGpioPins,
                                     enableGpioPin)
                    .withNoAcceleration()
                    .buildDRV8825Driver())

    def getLinearDRV8825With(self, stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin=None,
                             stepsMode="Full",
                             modeGpioPins=None,
                             enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin, stepsMode, modeGpioPins,
                                     enableGpioPin)
                .withLinearAcceleration()
                .buildDRV8825Driver())

    def getExponentialDRV8825With(self, stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin=None,
                                  stepsMode="Full",
                                  modeGpioPins=None,
                                  enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin, stepsMode, modeGpioPins,
                                     enableGpioPin)
                .withExponentialAcceleration()
                .buildDRV8825Driver())

    def getCustomTorqueCharacteristicsDRV8825With(self, stepperMotor, directionGpioPin, stepGpioPin, transformations=None,
                                                  sleepGpioPin=None,
                                                  stepsMode="Full",
                                                  modeGpioPins=None,
                                                  enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin, stepsMode, modeGpioPins,
                                     enableGpioPin)
                .withTorqueCurve(transformations)
                .withCustomTorqueCurveAccelerationAcceleration()
                .buildDRV8825Driver())

    def getInteractiveDriverBuilder(self, stepperMotor, directionGpioPin, stepGpioPin, minSpeedDelta, minPps,
                                  sleepGpioPin=None,
                                  stepsMode="Full",
                                  modeGpioPins=None,
                                  enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin, stepsMode, modeGpioPins,
                                     enableGpioPin)
                .withInteractiveAcceleration(minSpeedDelta, minPps))

    def getInteractiveDRV8825With(self, stepperMotor, directionGpioPin, stepGpioPin, minSpeedDelta, minPps, sleepGpioPin=None,
                                  stepsMode="Full",
                                  modeGpioPins=None,
                                  enableGpioPin=None):
        return (self.getInteractiveDriverBuilder(stepperMotor, directionGpioPin, stepGpioPin, minSpeedDelta, minPps,
                                                 sleepGpioPin, stepsMode, modeGpioPins, enableGpioPin)
                    .buildDRV8825Driver())

    def getFlatTCM2209With(self, stepperMotor, directionGpioPin, stepGpioPin,
                           stepsMode="1/8",
                           modeGpioPins=None,
                           enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin,
                                     stepsMode=stepsMode,
                                     modeGpioPins=modeGpioPins,
                                     enableGpioPin=enableGpioPin)
                .withNoAcceleration()
                .buildTMC2209StandaloneDriver())

    def getLinearTCM2209With(self, stepperMotor, directionGpioPin, stepGpioPin,
                             stepsMode="1/8",
                             modeGpioPins=None,
                             enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin,
                                     stepsMode=stepsMode,
                                     modeGpioPins=modeGpioPins,
                                     enableGpioPin=enableGpioPin)
                .withLinearAcceleration()
                .buildTMC2209StandaloneDriver())

    def getExponentialTCM2209With(self, stepperMotor, directionGpioPin, stepGpioPin,
                                  stepsMode="1/8",
                                  modeGpioPins=None,
                                  enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin,
                                     stepsMode=stepsMode,
                                     modeGpioPins=modeGpioPins,
                                     enableGpioPin=enableGpioPin)
                .withExponentialAcceleration()
                .buildTMC2209StandaloneDriver())

    def getCustomTorqueCharacteristicsTCM2209With(self, stepperMotor, directionGpioPin, stepGpioPin, transformations=None,
                                                  stepsMode="1/8",
                                                  modeGpioPins=None,
                                                  enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin,
                                     stepsMode=stepsMode,
                                     modeGpioPins=modeGpioPins,
                                     enableGpioPin=enableGpioPin)
                .withCustomTorqueCurveAccelerationAcceleration()
                .buildTMC2209StandaloneDriver())

    def getInteractiveTCM2209With(self, stepperMotor, directionGpioPin, stepGpioPin, minSpeedDelta, minPps,
                                  stepsMode="1/8",
                                  modeGpioPins=None,
                                  enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin,
                                     stepsMode=stepsMode,
                                     modeGpioPins=modeGpioPins,
                                     enableGpioPin=enableGpioPin)
                .withInteractiveAcceleration(minSpeedDelta, minPps)
                .buildTMC2209StandaloneDriver())

    def getLinearUnipolarDriverWith(self,
                                    stepperMotor,
                                    directionGpioPin,
                                    stepGpioPin,
                                    sleepGpioPin=None,
                                    stepsMode="Full",
                                    enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin,
                                     sleepGpioPin=sleepGpioPin,
                                     stepsMode=stepsMode,
                                     enableGpioPin=enableGpioPin)
                .withLinearAcceleration()
                .buildUnipolarDriver())

    def getExponentialUnipolarDriverWith(self,
                                         stepperMotor,
                                         directionGpioPin,
                                         stepGpioPin,
                                         sleepGpioPin=None,
                                         stepsMode="Full",
                                         enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin,
                                     sleepGpioPin=sleepGpioPin,
                                     stepsMode=stepsMode,
                                     enableGpioPin=enableGpioPin)
                .withExponentialAcceleration()
                .buildUnipolarDriver())

    def getCustomTorqueCharacteristicsUnipolarDriverWith(self,
                                                         stepperMotor,
                                                         directionGpioPin,
                                                         stepGpioPin,
                                                         transformations=None,
                                                         sleepGpioPin=None,
                                                         stepsMode="Full",
                                                         enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin,
                                     sleepGpioPin=sleepGpioPin,
                                     stepsMode=stepsMode,
                                     enableGpioPin=enableGpioPin)
                .withTorqueCurve(transformations)
                .withCustomTorqueCurveAccelerationAcceleration()
                .buildUnipolarDriver())

    def getInteractiveUnipolarDriverWith(self,
                                         stepperMotor,
                                         directionGpioPin,
                                         stepGpioPin,
                                         minSpeedDelta,
                                         minPps,
                                         sleepGpioPin=None,
                                         stepsMode="Full",
                                         enableGpioPin=None):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin,
                                     sleepGpioPin=sleepGpioPin,
                                     stepsMode=stepsMode,
                                     enableGpioPin=enableGpioPin)
                .withInteractiveAcceleration(minSpeedDelta, minPps)
                .buildUnipolarDriver())

    def getFlatAdafruitStepperWith(self, stepperMotor, adafruitDriver: AdafruitStepperDriver, stepsMode="Full"):
        return (self.getBasicBuilder(stepperMotor, None, None, stepsMode=stepsMode)
                .withAdafruitDriver(adafruitDriver)
                .withNoAcceleration()
                .buildAdafruitStepperDriverAdapter())

    def getLinearAdafruitStepperWith(self, stepperMotor, adafruitDriver: AdafruitStepperDriver, stepsMode="Full"):
        return (self.getBasicBuilder(stepperMotor, None, None, stepsMode=stepsMode)
                .withAdafruitDriver(adafruitDriver)
                .withLinearAcceleration()
                .buildAdafruitStepperDriverAdapter())

    def getExponentialAdafruitStepperWith(self, stepperMotor, adafruitDriver: AdafruitStepperDriver, stepsMode="Full"):
        return (self.getBasicBuilder(stepperMotor, None, None, stepsMode=stepsMode)
                .withAdafruitDriver(adafruitDriver)
                .withExponentialAcceleration()
                .buildAdafruitStepperDriverAdapter())

    def getCustomTorqueCharacteristicsAdafruitStepperWith(self, stepperMotor,
                                                          adafruitDriver: AdafruitStepperDriver,
                                                          transformations=None,
                                                          stepsMode="Full"):
        return (self.getBasicBuilder(stepperMotor, None, None, stepsMode=stepsMode)
                .withAdafruitDriver(adafruitDriver)
                .withTorqueCurve(transformations)
                .withCustomTorqueCurveAccelerationAcceleration()
                .buildAdafruitStepperDriverAdapter())

    def getInteractiveAdafruitStepperWith(self, stepperMotor, minSpeedDelta, minPps,
                                          adafruitDriver: AdafruitStepperDriver,
                                          stepsMode="Full"):
        return (self.getBasicBuilder(stepperMotor, None, None, stepsMode=stepsMode)
                .withAdafruitDriver(adafruitDriver)
                .withInteractiveAcceleration(minSpeedDelta, minPps)
                .buildAdafruitStepperDriverAdapter())

    @abstractmethod
    def addNavigationStyle(self, builder: ControllerBuilder, syncNavigationMultitonKey=0) -> ControllerBuilder:
        pass

class StaticControllerFactory(ControllerFactory):
    def getDelayPlanner(self):
        return StaticDelayPlanner()

    def getNavigation(self):
        return StaticNavigation()

    def addNavigationStyle(self, builder: ControllerBuilder, syncNavigationMultitonKey=0):
        return builder.withNavigationStyleStatic()

class DynamicControllerFactory(ControllerFactory):
    def getDelayPlanner(self):
        return DynamicDelayPlanner()

    def getNavigation(self):
        return DynamicNavigation()

    def addNavigationStyle(self, builder: ControllerBuilder, syncNavigationMultitonKey=0):
        return builder.withNavigationStyleDynamic()

class SynchronizedControllerFactory(ControllerFactory):

    def getDelayPlanner(self):
        return DynamicDelayPlanner()

    def getNavigation(self):
        return BasicSynchronizedNavigation.getInstance()

    def addNavigationStyle(self, builder: ControllerBuilder, syncNavigationMultitonKey=0):
        return builder.withNavigationStyleSynchronized(syncNavigationMultitonKey)


class MultiProcessingControllerFactory(SynchronizedControllerFactory):
    runningProcesses = []

    def __init__(self):
        self.eventDispatcher = EventDispatcher.instance()
        self._factoryOrders = []
        self._clientMultiprocessObservers = []

    def setUpProcess(self) -> 'MultiProcessingControllerFactory':
        self._factoryOrders = []
        self._clientMultiprocessObservers = []
        return self

    def withDriverBuilder(self, builder: ControllerBuilder, builderMethodRef):
        # Todo: test if breaks.
        self._factoryOrders.append((builder, builderMethodRef))
        # Uses buider.clientMultiprocessObserver
        self._clientMultiprocessObservers.append(None)
        return self

    def withDriver(self, factoryFnReference: Callable, multiprocessObserver: MultiprocessObserver = None,
                   *args, **kwargs) -> 'MultiProcessingControllerFactory':
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

    def spawn(self, syncNavigationMultitonKey=0):
        """
        @param withBasicSynchronizedNavigationMultitonKey: In case you need to coordinate a group of motors per
                                                           process, they'll need one instance of
                                                           BasicSynchronizedNavigation each to avoid conflicts with the
                                                           latch. MainProcess will have keyed instances, Child process
                                                           one instance each.
        @return: A list of proxy drivers you can use on MainProcess to send stepping jobs to counterpart drivers in
        child process.        
        """
        print("SPAWNING!!")
        syncNavigationCountDownLatch = (BasicSynchronizedNavigation.getInstance(
                    newMultitonKey=syncNavigationMultitonKey)
                .getCountDownLatch(default=Value('i', 0)))
        if isinstance(syncNavigationCountDownLatch, BasicSynchronizedNavigation.CountDownLatch):
            raise RuntimeError("BasicSynchronizedNavigation.CountDownLatch already initialized as mono-process")

        if self._clientMultiprocessObservers:
            assert len(self._clientMultiprocessObservers) == len(self._factoryOrders)

        queues = []
        sharedMemories = []
        for i, order in enumerate(self._factoryOrders):
            sharedMemory = []
            # Position updates
            sharedLock = Lock()
            positionValue = Value(DriverSharedPositionStruct, 0, 0, lock=True)
            sharedMemory.extend([sharedLock, positionValue])
            if self._clientMultiprocessObservers:
                if isinstance(order[0], ControllerBuilder):
                    builder = order[0]
                    observer = builder.clientMultiprocessObserver
                    sharedMemory.append(observer)
                else:
                    sharedMemory.append(self._clientMultiprocessObservers[i])

            # Worker Queue
            queues.append(MpQueue())
            sharedMemories.append(sharedMemory)

        eventsMultiprocessObserver = self.eventDispatcher.getMultiprocessObserver()

        proxies = self.Unpacker.doUnpack(self._factoryOrders, queues, sharedMemories, isProxy=True,
                                         syncNavigationMultitonKey=syncNavigationMultitonKey)

        jobCompletionObservers = [driver.jobCompletionObserver for driver in proxies]
        unpacker = self.Unpacker(self._factoryOrders, queues, sharedMemories, eventsMultiprocessObserver)
        childProcess = Process(target=unpacker.unpack, args=(jobCompletionObservers, syncNavigationCountDownLatch))
        childProcess.daemon = True
        flush_streams_if_not_empty()
        childProcess.start()
        self.runningProcesses.append((childProcess, proxies))
        self._factoryOrders = []
        self._clientMultiprocessObservers = []
        return proxies

    def getMpCustomTorqueCharacteristicsDRV8825With(self, jobQueue, sharedMemory, isProxy, jobCompletionObserver,
                                                    syncNavigationMultitonKey, stepperMotor, directionGpioPin,
                                                    stepGpioPin,
                                                    transformations=None,
                                                    sleepGpioPin=None,
                                                    stepsMode="Full",
                                                    modeGpioPins=None,
                                                    enableGpioPin=None,
                                                    steppingCompleteEventName="steppingComplete"):

        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin, stepsMode, modeGpioPins,
                                     enableGpioPin, syncNavigationMultitonKey)
                    .withSharedMemory(sharedMemory)
                    .withJobQueue(jobQueue)
                    .withJobCompletionObserver(jobCompletionObserver)
                    .isMultiprocessProxy(isProxy)
                    .withTorqueCurve(transformations)
                    .withsteppingCompleteEventName(steppingCompleteEventName)
                    .withCustomTorqueCurveAccelerationAcceleration()
                    .buildDRV8825Driver())

    def getMpLinearDRV8825With(self, jobQueue, sharedMemory, isProxy, jobCompletionObserver,
                               syncNavigationMultitonKey, stepperMotor, directionGpioPin,
                               stepGpioPin,
                               sleepGpioPin=None,
                               stepsMode="Full",
                               modeGpioPins=None,
                               enableGpioPin=None,
                               steppingCompleteEventName="steppingComplete"):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin, stepsMode, modeGpioPins,
                                     enableGpioPin, syncNavigationMultitonKey)
                    .withSharedMemory(sharedMemory)
                    .withJobQueue(jobQueue)
                    .withJobCompletionObserver(jobCompletionObserver)
                    .isMultiprocessProxy(isProxy)
                    .withsteppingCompleteEventName(steppingCompleteEventName)
                    .withLinearAcceleration()
                    .buildDRV8825Driver())

    def getMpExponentialDRV8825With(self, jobQueue, sharedMemory, isProxy, jobCompletionObserver,
                                    syncNavigationMultitonKey, stepperMotor,
                                    directionGpioPin, stepGpioPin,
                                    sleepGpioPin=None,
                                    stepsMode="Full",
                                    modeGpioPins=None,
                                    enableGpioPin=None,
                                    steppingCompleteEventName="steppingComplete"):

        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, sleepGpioPin, stepsMode, modeGpioPins,
                                     enableGpioPin, syncNavigationMultitonKey)
                    .withSharedMemory(sharedMemory)
                    .withJobQueue(jobQueue)
                    .withJobCompletionObserver(jobCompletionObserver)
                    .isMultiprocessProxy(isProxy)
                    .withsteppingCompleteEventName(steppingCompleteEventName)
                    .withExponentialAcceleration()
                    .buildDRV8825Driver())

    def getMpCustomTorqueCharacteristicsTMC2209With(self, jobQueue, sharedMemory, isProxy, jobCompletionObserver,
                                                    syncNavigationMultitonKey, stepperMotor, directionGpioPin,
                                                    stepGpioPin,
                                                    transformations=None,
                                                    stepsMode="1/8",
                                                    modeGpioPins=None,
                                                    enableGpioPin=None,
                                                    steppingCompleteEventName="steppingComplete"):

        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, stepsMode=stepsMode,
                                     modeGpioPins=modeGpioPins, enableGpioPin=enableGpioPin,
                                     syncNavigationMultitonKey=syncNavigationMultitonKey)
                    .withSharedMemory(sharedMemory)
                    .withJobQueue(jobQueue)
                    .withJobCompletionObserver(jobCompletionObserver)
                    .isMultiprocessProxy(isProxy)
                    .withsteppingCompleteEventName(steppingCompleteEventName)
                    .withTorqueCurve(transformations)
                    .withCustomTorqueCurveAccelerationAcceleration()
                    .buildTMC2209StandaloneDriver())

    def getMpLinearTMC2209With(self, jobQueue, sharedMemory, isProxy, jobCompletionObserver,
                               syncNavigationMultitonKey, stepperMotor, directionGpioPin, stepGpioPin,
                               stepsMode="1/8",
                               modeGpioPins=None,
                               enableGpioPin=None,
                               steppingCompleteEventName="steppingComplete"):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, stepsMode=stepsMode,
                                     modeGpioPins=modeGpioPins, enableGpioPin=enableGpioPin,
                                     syncNavigationMultitonKey=syncNavigationMultitonKey)
                    .withSharedMemory(sharedMemory)
                    .withJobQueue(jobQueue)
                    .withJobCompletionObserver(jobCompletionObserver)
                    .isMultiprocessProxy(isProxy)
                    .withsteppingCompleteEventName(steppingCompleteEventName)
                    .withLinearAcceleration()
                    .buildTMC2209StandaloneDriver())

    def getMpExponentialTMC2209With(self, jobQueue, sharedMemory, isProxy, jobCompletionObserver,
                                    syncNavigationMultitonKey, stepperMotor,
                                    directionGpioPin, stepGpioPin,
                                    stepsMode="1/8",
                                    modeGpioPins=None,
                                    enableGpioPin=None,
                                    steppingCompleteEventName="steppingComplete"):
        return (
            self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, stepsMode=stepsMode,
                                 modeGpioPins=modeGpioPins, enableGpioPin=enableGpioPin,
                                 syncNavigationMultitonKey=syncNavigationMultitonKey)
                    .withSharedMemory(sharedMemory)
                    .withJobQueue(jobQueue)
                    .withJobCompletionObserver(jobCompletionObserver)
                    .isMultiprocessProxy(isProxy)
                    .withsteppingCompleteEventName(steppingCompleteEventName)
                    .withExponentialAcceleration()
                    .buildTMC2209StandaloneDriver())

    def getMpCustomTorqueCharacteristicsUnipolarDriverWith(self, jobQueue, sharedMemory, isProxy, jobCompletionObserver,
                                                           syncNavigationMultitonKey, stepperMotor, directionGpioPin,
                                                           stepGpioPin,
                                                           transformations=None,
                                                           sleepGpioPin=None,
                                                           stepsMode="Full",
                                                           enableGpioPin=None,
                                                           steppingCompleteEventName="steppingComplete"):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, stepsMode=stepsMode,
                                     sleepGpioPin=sleepGpioPin, enableGpioPin=enableGpioPin,
                                     syncNavigationMultitonKey=syncNavigationMultitonKey)
                    .withSharedMemory(sharedMemory)
                    .withJobQueue(jobQueue)
                    .withJobCompletionObserver(jobCompletionObserver)
                    .isMultiprocessProxy(isProxy)
                    .withsteppingCompleteEventName(steppingCompleteEventName)
                    .withTorqueCurve(transformations)
                    .withCustomTorqueCurveAccelerationAcceleration()
                    .buildUnipolarDriver())

    def getMpLinearUnipolarDriverWith(self, jobQueue, sharedMemory, isProxy, jobCompletionObserver,
                                      syncNavigationMultitonKey, stepperMotor,
                                      directionGpioPin, stepGpioPin,
                                      sleepGpioPin=None,
                                      stepsMode="Full",
                                      enableGpioPin=None,
                                      steppingCompleteEventName="steppingComplete"):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, stepsMode=stepsMode,
                                     sleepGpioPin=sleepGpioPin, enableGpioPin=enableGpioPin,
                                     syncNavigationMultitonKey=syncNavigationMultitonKey)
                    .withSharedMemory(sharedMemory)
                    .withJobQueue(jobQueue)
                    .withJobCompletionObserver(jobCompletionObserver)
                    .isMultiprocessProxy(isProxy)
                    .withsteppingCompleteEventName(steppingCompleteEventName)
                    .withLinearAcceleration()
                    .buildUnipolarDriver())

    def getMpExponentialUnipolarDriverWith(self, jobQueue, sharedMemory, isProxy, jobCompletionObserver,
                                           syncNavigationMultitonKey, stepperMotor,
                                           directionGpioPin, stepGpioPin,
                                           sleepGpioPin=None,
                                           stepsMode="Full",
                                           enableGpioPin=None,
                                           steppingCompleteEventName="steppingComplete"):
        return (self.getBasicBuilder(stepperMotor, directionGpioPin, stepGpioPin, stepsMode=stepsMode,
                                     sleepGpioPin=sleepGpioPin, enableGpioPin=enableGpioPin,
                                     syncNavigationMultitonKey=syncNavigationMultitonKey)
                    .withSharedMemory(sharedMemory)
                    .withJobQueue(jobQueue)
                    .withJobCompletionObserver(jobCompletionObserver)
                    .isMultiprocessProxy(isProxy)
                    .withsteppingCompleteEventName(steppingCompleteEventName)
                    .withExponentialAcceleration()
                    .buildUnipolarDriver())

    def getMpCustomTorqueCharacteristicsAdafruitStepperWith(self, jobQueue, sharedMemory, isProxy,
                                                            jobCompletionObserver,
                                                            syncNavigationMultitonKey, stepperMotor,
                                                            adafruitStepperDriver: AdafruitStepperDriver,
                                                            transformations=None,
                                                            stepsMode="Full",
                                                            steppingCompleteEventName="steppingComplete"):
        return (self.getBasicBuilder(stepperMotor, None, None, stepsMode=stepsMode,
                                     syncNavigationMultitonKey=syncNavigationMultitonKey)
                    .withSharedMemory(sharedMemory)
                    .withJobQueue(jobQueue)
                    .withJobCompletionObserver(jobCompletionObserver)
                    .isMultiprocessProxy(isProxy)
                    .withAdafruitDriver(adafruitStepperDriver)
                    .withsteppingCompleteEventName(steppingCompleteEventName)
                    .withTorqueCurve(transformations)
                    .withLinearAcceleration()
                    .buildAdafruitStepperDriverAdapter())

    def getMpLinearAdafruitStepperWith(self, jobQueue, sharedMemory, isProxy, jobCompletionObserver,
                                       syncNavigationMultitonKey, stepperMotor,
                                       adafruitStepperDriver: AdafruitStepperDriver,
                                       stepsMode="Full",
                                       steppingCompleteEventName="steppingComplete"):
        return (self.getBasicBuilder(stepperMotor, None, None, stepsMode=stepsMode,
                                     syncNavigationMultitonKey=syncNavigationMultitonKey)
                    .withSharedMemory(sharedMemory)
                    .withJobQueue(jobQueue)
                    .withJobCompletionObserver(jobCompletionObserver)
                    .isMultiprocessProxy(isProxy)
                    .withAdafruitDriver(adafruitStepperDriver)
                    .withsteppingCompleteEventName(steppingCompleteEventName)
                    .withLinearAcceleration()
                    .buildAdafruitStepperDriverAdapter())

    def getMpExponentialAdafruitStepperWith(self, jobQueue, sharedMemory, isProxy, jobCompletionObserver,
                                            syncNavigationMultitonKey, stepperMotor,
                                            adafruitStepperDriver: AdafruitStepperDriver,
                                            stepsMode="Full",
                                            steppingCompleteEventName="steppingComplete"):
        return (self.getBasicBuilder(stepperMotor, None, None, stepsMode=stepsMode,
                                     syncNavigationMultitonKey=syncNavigationMultitonKey)
                    .withSharedMemory(sharedMemory)
                    .withJobQueue(jobQueue)
                    .withJobCompletionObserver(jobCompletionObserver)
                    .isMultiprocessProxy(isProxy)
                    .withAdafruitDriver(adafruitStepperDriver)
                    .withsteppingCompleteEventName(steppingCompleteEventName)
                    .withExponentialAcceleration()
                    .buildAdafruitStepperDriverAdapter())

    class Unpacker:
        def __init__(self, factoryOrders, queues: list, sharedMemories: list, eventMultiprocessObserver):
            self.factoryOrders = factoryOrders
            self.queues = queues
            self.sharedMemories = sharedMemories
            self.multiprocessObserver = eventMultiprocessObserver
            self.eventDispatcher = None

        def unpack(self, jobCompletionObservers, syncNavigationCountDownLatch):
            """
            In new process, build Drivers
            @return:
            """
            # Removing Singletons instances in case its state was cloned from Parent Process:
            EventDispatcher._instance = None
            BasicSynchronizedNavigation._instances = {}
            BasicSynchronizedNavigation.getInstance(countDownLatch=syncNavigationCountDownLatch)
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
            flush_streams_if_not_empty()
            # block main forever.
            drivers.pop().workerFuture.result()

        @staticmethod
        def doUnpack(factoryOrders, queues, sharedMemories, isProxy, jobCompletionObservers=None,
                     syncNavigationMultitonKey=0) \
                -> list[MotorDriver]:
            drivers = []
            for index, order in enumerate(factoryOrders):
                if isinstance(order[0], ControllerBuilder):
                    # jobCompletionObservers[index] should be None as the Builder has (or not) a reference to it.
                    builder: ControllerBuilder = order[0]
                    (builder.isMultiprocessProxy(isProxy)
                            .withJobQueue(queues[index])
                            .withSharedMemory(sharedMemories[index]))
                    if isinstance(builder.navigation, BasicSynchronizedNavigation):
                        # Make sure driers that are synched use the right multiton instance.
                        builder.navigation = BasicSynchronizedNavigation.getInstance(
                                newMultitonKey=syncNavigationMultitonKey)

                    if jobCompletionObservers:
                        builder.withJobCompletionObserver(jobCompletionObservers[index])

                    # Unique scenario, builder can build same Driver twice, if each lives in a different process.
                    builder.built = False
                    driver = order[1]()
                    drivers.append(driver)
                elif jobCompletionObservers:
                    driver = order[0](queues[index], sharedMemories[index], isProxy, jobCompletionObservers[index],
                                      syncNavigationMultitonKey, *order[1][0], **order[1][1])
                    drivers.append(driver)
                else:
                    driver = order[0](queues[index], sharedMemories[index], isProxy, None,
                                      syncNavigationMultitonKey, *order[1][0], **order[1][1])
                    drivers.append(driver)
                tprint(driver)
            return drivers
