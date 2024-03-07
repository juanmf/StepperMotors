from multiprocess import Manager, parent_process


class SharedManager:
    instance = None

    @staticmethod
    def getInstance():
        if SharedManager.instance:
            return SharedManager.instance

        SharedManager.instance = SharedManager()
        return SharedManager.instance

    def __init__(self):
        assert not SharedManager.instance
        assert parent_process() is None
        self.manager = Manager()

    def getManager(self):
        return self.manager
