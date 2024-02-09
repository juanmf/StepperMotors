import ctypes
import uuid
from multiprocess import Manager, Lock, Event


from stepper_motors_juanmf1.BlockingQueueWorker import BlockingQueueWorker
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint


class EventDispatcher(BlockingQueueWorker):
    MAX_EVENTS = 100
    _instance = None

    @staticmethod
    def instance(*args, **kwargs):
        if not EventDispatcher._instance:
            EventDispatcher._instance = EventDispatcher(*args, **kwargs)
        return EventDispatcher._instance

    def __init__(self, mpEventSharedMemory=None):
        assert EventDispatcher._instance is None
        super().__init__(self._dispatchMainLoop, jobQueueMaxSize=self.MAX_EVENTS, workerName="EventDispatcher_")
        self.events = {}
        self.markForUnregister = []
        self._mpManager: Manager = None
        self._mpEventSharedMemory = mpEventSharedMemory
        self._shouldDispatchToParentProcess = (mpEventSharedMemory is not None)
        self._interProcessWorker = None

    def getMpSharedMemory(self):
        if not self._mpEventSharedMemory:
            self._mpManager = Manager()
            eventInfo = self._mpManager.dict()
            eventInfo["eventName"] = ""
            eventInfo["eventInfo"] = ""
            self._mpEventSharedMemory = [Lock(), Event(), eventInfo]
            self._interProcessWorker = BlockingQueueWorker(self._waitChildProcessEvent, jobQueueMaxSize=10,
                                                           workerName="_interProcessEventDispatcherWorker")
            # Only one job, wait in infinite loop for mp Events.
            self._interProcessWorker.work([])

        return self._mpEventSharedMemory

    def register(self, eventName, callee, oneTimeHandler=False):
        """
        Registers one callee to one or more eventNames.
        :param eventName: str or list of event Name(s)
        :param callee: The callable, must accept (calleeId, eventInfo) arguments.
        :param oneTimeHandler: if unregister after first event dispatched to it.
        :return: uuid or list of uuid depending on how many events were registered.
        """
        eventName = [eventName] if isinstance(eventName, str) else eventName
        cIds = []
        for eName in eventName:
            self.events.setdefault(eName, {})
            cId = uuid.uuid4()
            cIds.append(cId)
            tprint(f"Register eventName {eName}; calleeId {cId}")
            self.events[eventName][cId] = \
                lambda calleeId, eventInfo: self._callThenUnregister(calleeId, eventInfo, callee) \
                if oneTimeHandler \
                else callee

        return cIds[0] if len(cIds) == 1 else cIds

    def unregister(self, calleeId):
        tprint(f"Unregister calleeId {calleeId}")
        for event_dict in self.events.values():
            if calleeId in event_dict:
                del event_dict[calleeId]
                return
        tprint(f"Could not Unregister calleeId {calleeId}; Not found")

    def publishMainLoop(self, eventName, eventInfo=None):
        self.work([eventName, eventInfo], block=False)
        tprint(self)
        if self._shouldDispatchToParentProcess:
            lock = self._mpEventSharedMemory[0]
            event = self._mpEventSharedMemory[1]
            with lock:
                self._mpEventSharedMemory[2]["eventName"] = eventName
                self._mpEventSharedMemory[2]["eventInfo"] = eventInfo

            event.set()

    def _dispatchMainLoop(self, eventName, eventInfo):
        # Todo: should I add eventId to callee parameters?
        tprint("dispatchMainLoop", f"eventName: {eventName}, eventInfo: {eventInfo}")
        for calleeId, callee in self.events.get(eventName, {}).items():
            callee(calleeId=calleeId, eventInfo={**eventInfo, **{'eventName': eventName}})
        else:
            tprint(f"Missed event {eventName}")
        while self.markForUnregister:
            self.unregister(self.markForUnregister.pop(0))

    def _callThenUnregister(self, calleeId, eventInfo, callee):
        callee(calleeId=calleeId, eventInfo=eventInfo)
        self.markForUnregister.append(calleeId)

    def _waitChildProcessEvent(self):
        lock = self._mpEventSharedMemory[0]
        event = self._mpEventSharedMemory[1]
        while True:
            event.wait()
            with lock:
                eName, eInfo = self._mpEventSharedMemory[2]["eventName"], self._mpEventSharedMemory[2]["eventInfo"]
                self._mpEventSharedMemory[2]["eventName"] = ""
                self._mpEventSharedMemory[2]["eventInfo"] = ""
                event.clear()
            self._dispatchMainLoop(eName, eInfo)
