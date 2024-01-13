import queue
import sys
import threading
import traceback
from concurrent.futures import ThreadPoolExecutor, Future

from stepper_motors_juanmf1.ThreadOrderedPrint import tprint, get_current_thread_info


_WORKERS: list['BlockingQueueWorker'] = []
_WORKERS_LOCK = threading.Lock()

class UsesSingleThreadedExecutor:
    def __init__(self, threadPrefix=""):
        self.executor = ThreadPoolExecutorStackTraced(1, thread_name_prefix=threadPrefix)

    def __del__(self):
        self.executor.shutdown(wait=False, cancel_futures=True)


class BlockingQueueWorker(UsesSingleThreadedExecutor):

    def __init__(self, jobConsumer, *, jobQueueMaxSize=2, workerName="John_Doe_Worker"):
        super().__init__(workerName)
        self.workerName = workerName
        self.workerThread = None
        self.workerAtWork = False
        self.lock = threading.Lock()
        self.jobQueueMaxSize = jobQueueMaxSize
        self.jobQueue = queue.Queue(self.jobQueueMaxSize)
        self.workerFuture = self.startWorker(jobConsumer)
        with _WORKERS_LOCK:
            _WORKERS.append(self)

    def workChain(self, paramsList, block=False) -> 'BlockingQueueWorker.Chain':
        chain = BlockingQueueWorker.Chain(self, paramsList=paramsList)
        self.work(chain, block=block)
        return chain

    def work(self, paramsList, block=False):
        # Todo:evaluate use furture interface not just list.
        self.jobQueue.put(paramsList, block=block)

    def killWorker(self) -> 'BlockingQueueWorker.PoisonPill':
        pill = BlockingQueueWorker.PoisonPill([])
        self.jobQueue.put(pill, block=True)
        return pill

    def startWorker(self, jobConsumer) -> Future:
        if self.workerAtWork:
            raise RuntimeError("There is already a worker at work.")
        return self.executor.submit(lambda: self.consumer(jobConsumer))

    def consumer(self, jobConsumer):
        # execute job in dedicated thread.
        self.workerThread = get_current_thread_info()
        try:
            with self.lock:
                self.workerAtWork = True

            while True:
                # Block until movement job is sent our way.
                job = self.jobQueue.get(block=True)
                if isinstance(job, BlockingQueueWorker.PoisonPill):
                    self.jobQueue.task_done()
                    job.isSwallowed(True)
                    return

                jobConsumer(*job)
                self.jobQueue.task_done()
                if isinstance(job, BlockingQueueWorker.Chain):
                    tprint("BlockingQueueWorker.Chain")
                    tprint("job")
                    tprint(job)
                    tprint("")
                    with self.lock:
                        job.completed = True
                        if job.getNext() is not None:
                            # consumer thread producing, can't block!
                            self.work(job.getNext(), block=False)
        finally:
            with self.lock:
                self.workerAtWork = False

    def __str__(self):
        # thread_id = threading.current_thread().ident

        return (f"BlockingQueueWorker {self.workerName} with jobList probable len {self.jobQueue.qsize()} \n"
                f"jobList max len {self.jobQueue.maxsize}"
                f"worker thread ({self.workerThread})")

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
        def __init__(self, worker: 'BlockingQueueWorker', *, paramsList: list = None):
            super().__init__([] if paramsList is None else paramsList)
            self._isSwallowed = False

        def isSwallowed(self, value=None):
            if value is None:
                return self._isSwallowed
            self._isSwallowed = value


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
            tprint(e, traceback.format_exc())
            raise sys.exc_info()[0](traceback.format_exc())  # Creates an
            # exception of the
            # same type with the
            # traceback as
            # message


def killWorkers():
    """
    kills all workers when program tears down.
    """

    workerPills = []
    for worker in _WORKERS:
        workerPills.append((worker.workerFuture, worker.killWorker()))

    for index, (workerFuture, pill) in enumerate(workerPills):
        workerFuture.result()
        if not pill.isSwallowed():
            tprint(f"Worker died of other causes. {_WORKERS[index]}")
