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
            callee(calleeId=calleeId, eventInfo={**eventInfo, **{'eventName': eventName}})
        else:
            tprint(f"Missed event {eventName}")
        while self.markForUnregister:
            self.unregister(self.markForUnregister.pop(0))

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
                lambda calleeId, eventInfo: self.callThenUnregister(calleeId, eventInfo, callee) \
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

    def callThenUnregister(self, calleeId, eventInfo, callee):
        callee(calleeId=calleeId, eventInfo=eventInfo)
        self.markForUnregister.append(calleeId)

    def publishMainLoop(self, eventName, eventInfo=None):
        self.work([eventName, eventInfo], block=False)
