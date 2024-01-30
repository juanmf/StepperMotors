import queue
import sys
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor, Future

from stepper_motors_juanmf1.ThreadOrderedPrint import tprint, get_current_thread_info


_WORKERS: list['BlockingQueueWorker'] = []
_WORKERS_LOCK = threading.Lock()

class UsesSingleThreadedExecutor:
    def __init__(self, threadPrefix=''):
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
        self.__jobQueue = queue.Queue(self.jobQueueMaxSize)
        self.workerFuture = self.startWorker(jobConsumer)
        with _WORKERS_LOCK:
            _WORKERS.append(self)

    def hasQueuedJobs(self):
        return self.__jobQueue.qsize() > 0

    def workChain(self, paramsList: list, block=False) -> 'BlockingQueueWorker.Chain':
        """
        Individual chain links will have a future, same kind returned by self.work(). But these futures will be
        completed in LIFO order (Last In First Out.) So when the future associated with this root level Chain is
        completed, the whole Chain is done.
        @param paramsList: list of arguments to pass down your job handler fn.
        @param block: If you'd like your producer to block on full queues.
        @return: a BlockingQueueWorker.Chain instance representing the root level of a new chain of work that will be
        sequentially consumed.
        """
        chain = BlockingQueueWorker.Chain(self, paramsList=paramsList)
        return chain

    def work(self, paramsList: list, block=False, startTime: Future = None) -> 'BlockingQueueWorker.Job':
        """
        Adds you parameters, to be sent by executor to your handler fn in dedicated thread.
        @param paramsList: list of parameters to pass down to your registered handler (jobConsumer constructor param).
        @param block: Should block when queue is full?
        @param startTime: Overrides start time by client. Useful if you want to account for enqueueing time.
        @return: a Future that will be completed when your fn returns, with paramsList as value future.result().
        If your fn raise errors the future.exception() will contain infor about it.
        """
        job = paramsList if isinstance(paramsList, BlockingQueueWorker.Job) \
            else BlockingQueueWorker.Job(paramsList=paramsList)

        if startTime:
            job.startTime = startTime
        self.__jobQueue.put(job, block=block)
        return job

    def killWorker(self) -> 'BlockingQueueWorker.PoisonPill':
        pill = BlockingQueueWorker.PoisonPill(paramsList=[])
        self.__jobQueue.put(pill, block=True)
        return pill

    def startWorker(self, jobConsumer) -> Future:
        if self.workerAtWork:
            raise RuntimeError("There is already a worker at work.")
        return self.executor.submit(lambda: self.consumer(jobConsumer))

    def __doConsume(self, jobConsumer):
        # execute job in dedicated thread.
        self.workerThread = get_current_thread_info()
        job = None
        try:
            with self.lock:
                self.workerAtWork = True

            while True:
                # Block until movement job is sent our way.
                job = self.__jobQueue.get(block=True)
                if not isinstance(job, BlockingQueueWorker.Job):
                    raise ValueError(f"Job {job} not a BlockingQueueWorker.Job. You accessed jobQueue directly.")
                if isinstance(job, BlockingQueueWorker.PoisonPill):
                    job.isSwallowed(True)
                    self.__jobQueue.task_done()
                    return

                job.start()
                jobConsumer(*job)
                job.end()
                self.__jobQueue.task_done()
                if isinstance(job, BlockingQueueWorker.Chain):
                    self._handleChainOfWork(job)

        except Exception as e:
            throw = RuntimeError("Worker job failed.", job, e)
            job.setException(throw)
            tprint(f"Swallowing exception in worker {self.workerThread}; Error: {throw}\n{traceback.format_exc()}")
        finally:
            with self.lock:
                self.workerAtWork = False

    def consumer(self, jobConsumer):
        """
        restarts consumer on swallowed exceptions.
        """
        while True:
            self.__doConsume(jobConsumer)

    def _handleChainOfWork(self, job: 'BlockingQueueWorker.Chain'):
        tprint("BlockingQueueWorker.Chain")
        tprint("job")
        tprint(job, job.getNext())
        tprint("")
        if job.getNext() is not None:
            # consumer thread producing, can't block!
            self.work(job.getNext(), block=False)

    def __str__(self):
        # thread_id = threading.current_thread().ident

        return (f"BlockingQueueWorker {self.workerName} with jobList probable len {self.__jobQueue.qsize()} \n"
                f"jobList max len {self.__jobQueue.maxsize}\n"
                f"worker thread ({self.workerThread})")

    class Job(list):
        def __init__(self, *, paramsList: list = None, startTime: Future=None, endTime: Future=None):
            super().__init__(paramsList)
            self.startTime = startTime if startTime else Future()
            self.endTime = endTime if endTime else Future()

        def setException(self, throwable):
            if not self.startTime.done():
                tprint(f"Job didn't even start. Setting error to both startTime and endTime Futures.")
                self.startTime.set_exception(throwable)
                self.endTime.set_exception(throwable)
            elif not self.endTime.done():
                tprint(f"Job started but could not complete. Setting error to endTime Future.")
                self.endTime.set_exception(throwable)
            else:
                tprint(f"Job Futures Done when trying to set exception. Job:{self}.")

        def start(self):
            startTime = time.monotonic_ns()
            self.startTime.set_result(startTime)
            return startTime

        def end(self):
            endTime = time.monotonic_ns()
            self.endTime.set_result(time.monotonic_ns())
            return endTime

        def result(self):
            # Blocks caller until job is done. When JobConsumer sets endTime in ns
            return self.endTime.result()

    class Chain(Job):
        def __init__(self, worker: 'BlockingQueueWorker', *, paramsList: list = None,
                     prev: 'BlockingQueueWorker.Chain' = None):
            super().__init__(paramsList=[] if paramsList is None else paramsList)
            self.workStarted = False
            self._next = None
            self._prev = prev
            self.worker = worker
            if prev is None:
                # root link
                self.rootLink: BlockingQueueWorker.Chain = self
                self.chainEndTime = Future()
            else:
                self.rootLink: BlockingQueueWorker.Chain = prev.rootLink
                self.chainEndTime = self.rootLink.chainEndTime

        def then(self, paramsList):
            if self.workStarted:
                raise RuntimeError("Work chain already started.")
            self._next = BlockingQueueWorker.Chain(self.worker, paramsList=paramsList, prev=self)
            return self._next

        def getNext(self) -> 'BlockingQueueWorker.Chain':
            return self._next

        def getPrev(self):
            return self._prev

        def startChain(self, startTime: Future = None):
            self.workStarted = True
            if startTime:
                self.rootLink.startTime = startTime
            return self.worker.work(self.rootLink)

        def end(self):
            endTime = super().end()
            if self._next is None:
                # last work link in Chain.
                self.chainEndTime.set_result(endTime)

    class PoisonPill(Job):
        def __init__(self, *, paramsList: list = None):
            super().__init__(paramsList=paramsList)
            self._isSwallowed = False

        def isSwallowed(self, value=None):
            if value is None:
                return self._isSwallowed
            self._isSwallowed = value
            doneTime = time.monotonic_ns()
            self.startTime.set_result(doneTime)
            self.endTime.set_result(doneTime)


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
