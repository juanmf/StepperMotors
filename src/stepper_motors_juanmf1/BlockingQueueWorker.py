import queue
import sys
import threading
import traceback
from concurrent.futures import ThreadPoolExecutor


class UsesSingleThreadedExecutor:
    def __init__(self):
        self.executor = ThreadPoolExecutorStackTraced(1)

    def __del__(self):
        self.executor.shutdown(wait=False, cancel_futures=True)


class BlockingQueueWorker(UsesSingleThreadedExecutor):

    def __init__(self, jobConsumer, jobQueueMaxSize=2):
        super().__init__()
        self.workerAtWork = False
        self.lock = threading.Lock()
        self.jobQueueMaxSize = jobQueueMaxSize
        self.jobQueue = queue.Queue(self.jobQueueMaxSize)
        self.startWorker(jobConsumer)

    def workChain(self, paramsList, block=False) -> 'BlockingQueueWorker.Chain':
        chain = BlockingQueueWorker.Chain(self, paramsList=paramsList)
        self.work(chain, block=block)
        return chain

    def work(self, paramsList, block=False):
        self.jobQueue.put(paramsList, block=block)

    def killWorker(self):
        self.jobQueue.put(BlockingQueueWorker.PoisonPill([]), block=True)

    def startWorker(self, jobConsumer):
        if self.workerAtWork:
            raise RuntimeError("There is already a worker at work.")
        self.executor.submit(lambda: self.consumer(jobConsumer))

    def consumer(self, jobConsumer):
        # execute job in dedicated thread.
        try:
            with self.lock:
                self.workerAtWork = True

            while True:
                # Block until movement job is sent our way.
                job = self.jobQueue.get(block=True)
                if isinstance(job, BlockingQueueWorker.PoisonPill):
                    self.jobQueue.task_done()
                    return

                jobConsumer(*job)
                self.jobQueue.task_done()
                if isinstance(job, BlockingQueueWorker.Chain):
                    with self.lock:
                        job.completed = True
                        if job.getNext() is not None:
                            # consumer thread producing, can't block!
                            self.work(job.getNext(), block=False)
        finally:
            with self.lock:
                self.workerAtWork = False

    class Chain(list):
        def __init__(self, worker: 'BlockingQueueWorker', *, paramsList: list = None, prev: 'BlockingQueueWorker.Chain' = None):
            super().__init__([] if paramsList is None else paramsList)
            self._next = None
            self._prev = paramsList if prev is None and isinstance(paramsList, BlockingQueueWorker.Chain) else prev
            self.completed = False
            self.worker = worker

        def then(self, paramsList):
            self._next = BlockingQueueWorker.Chain(self.worker, paramsList=paramsList, prev=self)
            if self.completed:
                # Sends next job to work immediately
                self.worker.work(self._next)
            return self._next

        def getNext(self):
            return self._next

        def getPrev(self):
            return self._prev


    class PoisonPill(list):
        pass


class ThreadPoolExecutorStackTraced(ThreadPoolExecutor):

    def submit(self, fn, *args, **kwargs):
        """Submits the wrapped function instead of `fn`"""

        return super(ThreadPoolExecutorStackTraced, self).submit(
            self._function_wrapper, fn, *args, **kwargs)

    def _function_wrapper(self, fn, *args, **kwargs):
        """Wraps `fn` in order to preserve the traceback of any kind of
        raised exception

        """
        try:
            return fn(*args, **kwargs)
        except Exception as e:
            print(e, traceback.format_exc())
            raise sys.exc_info()[0](traceback.format_exc())  # Creates an
            # exception of the
            # same type with the
            # traceback as
            # message
