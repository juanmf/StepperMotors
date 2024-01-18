import uuid

from stepper_motors_juanmf1.BlockingQueueWorker import BlockingQueueWorker
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint


class EventDispatcher(BlockingQueueWorker):
    MAX_EVENTS = 100

    def __new__(cls, *args, **kwargs):
        """
        Singleton
        """
        if not hasattr(cls, 'instance'):
            cls.instance = super().__new__(cls)
        return cls.instance

    def __init__(self):
        super().__init__(self.dispatchMainLoop, jobQueueMaxSize=self.MAX_EVENTS, workerName="EventDispatcher_")
        self.events = {}
        self.markForUnregister = []

    def dispatchMainLoop(self, eventName, eventInfo):
        # Todo: should I add eventId to callee parameters?
        tprint("dispatchMainLoop")
        tprint("eventName, eventInfo")
        tprint(eventName, eventInfo)
        tprint("")
        for calleeId, callee in self.events.get(eventName, {}).items():
            callee(calleeId=calleeId, eventInfo=eventInfo)
        else:
            tprint(f"Missed event {eventName}")
        while self.markForUnregister:
            self.unregister(self.markForUnregister.pop(0))

    def register(self, eventName, callee, oneTimeHandler=False):
        self.events.setdefault(eventName, {})
        cId = uuid.uuid4()
        tprint(f"Register eventName {eventName}; calleeId {cId}")
        self.events[eventName][cId] = lambda calleeId, eventInfo: self.callThenUnregister(calleeId, eventInfo, callee) \
                                           if oneTimeHandler else callee
        return cId

    def unregister(self, calleeId):
        tprint(f"Unregister calleeId {calleeId}")
        for event_dict in self.events.values():
            if calleeId in event_dict:
                del event_dict[calleeId]
                break

    def callThenUnregister(self, calleeId, eventInfo, callee):
        callee(calleeId=calleeId, eventInfo=eventInfo)
        self.markForUnregister.append(calleeId)

    def publishMainLoop(self, eventName, eventInfo=None):
        self.work([eventName, eventInfo], block=False)
