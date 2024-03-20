import uuid

from multiprocess import Manager

from stepper_motors_juanmf1.BlockingQueueWorker import BlockingQueueWorker, MultiprocessObserver
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint
from stepper_motors_juanmf1.MultiProcessShared import SharedManager


class EventDispatcher(BlockingQueueWorker):
    MAX_EVENTS = 100
    _instance = None

    @staticmethod
    def instance(*args, **kwargs):
        if not EventDispatcher._instance:
            EventDispatcher._instance = EventDispatcher(*args, **kwargs)
        return EventDispatcher._instance

    def __init__(self, multiprocessObserver=None):
        assert EventDispatcher._instance is None
        super().__init__(self._dispatchMainLoop, jobQueueMaxSize=self.MAX_EVENTS, workerName="EventDispatcher_")
        tprint(f"ED.__init__ > multiprocessObserver {multiprocessObserver}")
        self.events = {}
        self.markForUnregister = []
        self._mpManager: Manager = None
        self._multiprocessObserver = multiprocessObserver
        self._shouldDispatchToParentProcess = (multiprocessObserver is not None)
        self._interProcessWorker = None

    def getMultiprocessObserver(self):
        if not self._multiprocessObserver:
            self._mpManager = SharedManager.getInstance().getManager()
            eventInfo = self._mpManager.dict()
            eventInfo["eventName"] = ""
            eventInfo["eventInfo"] = ""
            self._multiprocessObserver = MultiprocessObserver(eventObserver=self._doWaitChildProcessEvent,
                                                              eventPublisher=self.multiProcessPublish,
                                                              sharedMemory=eventInfo)
        return self._multiprocessObserver

    def register(self, eventName, callee, oneTimeHandler=False):
        """
        Registers one callee to one or more eventNames.
        :param eventName: str or list of event Name(s)
        :param callee: The callable, must accept (calleeId, eventInfo) arguments.
        :param oneTimeHandler: if unregister after first event dispatched to it.
        :return: uuid or list of uuid depending on how many events were registered.
        """
        assert callable(callee)
        eventName = [eventName] if isinstance(eventName, str) else eventName
        cIds = []
        for eName in eventName:
            self.events.setdefault(eName, {})
            cId = uuid.uuid4()
            cIds.append(cId)
            if oneTimeHandler:
                self.events[eName][cId] = lambda calleeId, eventInfo: (
                    self._callThenUnregister(calleeId, eventInfo, callee))
            else:
                # Todo: add prints in DEBUG level. This print normally happens seldom, but one could abuse
                #  oneTimeHandler feature
                tprint(f"Register eventName {eName}; calleeId {cId}")
                self.events[eName][cId] = callee

        return cIds[0] if len(cIds) == 1 else cIds

    def unregister(self, calleeId):
        for event_dict in self.events.values():
            if calleeId in event_dict:
                del event_dict[calleeId]
                return
        tprint(f"Could not Unregister calleeId {calleeId}; Not found")

    def publishMainLoop(self, eventName, eventInfo=None):
        self.work([eventName, eventInfo], block=False)
        tprint(f"self._shouldDispatchToParentProcess {self._shouldDispatchToParentProcess}")
        if self._shouldDispatchToParentProcess:
            MultiprocessObserver.eventPublisher(self._multiprocessObserver, eventName, eventInfo)

    def _dispatchMainLoop(self, eventName, eventInfo):
        # Todo: should I add eventId to callee parameters?
        if eventName not in self.events:
            # Todo: assess how much this print bothers in normal operation. Normally client app manages its events.
            #   But operateStepper shuts 2 events per job regardless. Might be a good case for making Stepper events
            #   optional.
            tprint(f"Missed event {eventName}")
            return
        tprint(f"eventName {eventName}")
        for calleeId, callee in self.events.get(eventName, {}).items():
            callee.__call__(calleeId, {**eventInfo, **{'eventName': eventName}})

        while self.markForUnregister:
            self.unregister(self.markForUnregister.pop(0))

    def _callThenUnregister(self, calleeId, eventInfo, callee):
        callee(calleeId=calleeId, eventInfo=eventInfo)
        self.markForUnregister.append(calleeId)

    def _doWaitChildProcessEvent(self, sharedMemory):
        eName, eInfo = sharedMemory["eventName"], sharedMemory["eventInfo"]
        sharedMemory["eventName"] = ""
        sharedMemory["eventInfo"] = ""
        tprint(f"Fwding Event details IN parent: {eName}")
        self.publishMainLoop(eName, eInfo)

    @staticmethod
    def multiProcessPublish(sharedMemory, eventName, eventInfo):
        tprint(f"writing Event details for parent: {eventName}")
        sharedMemory["eventName"] = eventName
        sharedMemory["eventInfo"] = eventInfo
