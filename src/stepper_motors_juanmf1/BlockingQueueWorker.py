import queue

import sys
import threading
import time
import traceback
import uuid
from concurrent.futures import ThreadPoolExecutor, Future

import multiprocess as mp
from multiprocess.queues import Queue

from stepper_motors_juanmf1.ThreadOrderedPrint import tprint, get_current_thread_info, flush_streams_if_not_empty

_WORKERS: list['BlockingQueueWorker'] = []
_WORKERS_LOCK = threading.Lock()


class UsesSingleThreadedExecutor:
    def __init__(self, threadPrefix='', executor=None):
        self.executor = executor if executor is not None \
                        else ThreadPoolExecutorStackTraced(1, thread_name_prefix=threadPrefix)

    def __del__(self):
        self.executor.shutdown(wait=False, cancel_futures=True)


class BlockingQueueWorker(UsesSingleThreadedExecutor):
    WORKER_COUNT = 0

    def __init__(self, jobConsumer, *, jobQueueMaxSize=2, workerName="John_Doe_Worker", jobQueue=None, executor=None,
                 doneQueue=None, picklizeJobs=False, isProxy=False):
        with _WORKERS_LOCK:
            workerName += f"_{BlockingQueueWorker.WORKER_COUNT}"
            BlockingQueueWorker.WORKER_COUNT += 1

        super().__init__(workerName, executor)
        self.picklizeJobs = picklizeJobs
        self.workerName = workerName
        self.workerThread = None
        self.workerAtWork = False
        self.lock = threading.Lock()
        self.jobQueueMaxSize = jobQueueMaxSize
        self.__jobQueue = jobQueue if jobQueue is not None else queue.Queue(self.jobQueueMaxSize)
        self.isMultiprocess = isinstance(jobQueue, MpQueue)
        self.__doneQueue = doneQueue
        # parent process side do not actually work on queued items.
        self.isProxy = isProxy
        self.workerFuture = self.startWorker(jobConsumer)
        with _WORKERS_LOCK:
            _WORKERS.append(self)
        self.currentJob = None

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
        job = self.jobbify(paramsList, startTime)
        if self.isProxy:
            # Parent process/client side
            tprint(f"Queueing plain paramList as Proxied: {paramsList} at {time.monotonic_ns()}")

            self.__jobQueue.put(BlockingQueueWorker.ProxiedChain(paramsList)
                                if isinstance(paramsList, BlockingQueueWorker.Chain)
                                else BlockingQueueWorker.Proxied(paramsList))
            # Todo sync completion
            return job

        self.__jobQueue.put(job, block=block)
        return job

    def killWorker(self) -> 'BlockingQueueWorker.PoisonPill':
        pill = BlockingQueueWorker.PoisonPill(paramsList=[])
        self.__jobQueue.put(pill, block=True)
        return pill

    def startWorker(self, jobConsumer) -> Future:
        if self.isProxy:
            # Todo: never complete. the child process might end, we don't have visibility. we could use this thread
            #  to propagate events on parent process side.
            return Future()

        if self.workerAtWork:
            raise RuntimeError("There is already a worker at work.")
        return self.executor.submit(lambda: self.consumer(jobConsumer))

    def getJobQueue(self):
        return self.__jobQueue

    def __doConsume(self, jobConsumer):
        # execute job in dedicated thread.
        self.workerThread = get_current_thread_info()
        job = None
        try:
            with self.lock:
                self.workerAtWork = True

            while True:
                if self.isMultiprocess and not self.isProxy:
                    # forcing prints on child processes
                    tprint(f"waiting for MultiProcess jobs {self.__jobQueue}")
                    flush_streams_if_not_empty()
                # Block until movement job is sent our way.
                job = self.__jobQueue.get(block=True)
                if isinstance(job, BlockingQueueWorker.Proxied) or isinstance(job, BlockingQueueWorker.ProxiedChain):
                    job = self.jobbify(job)

                self.currentJob = job
                if not isinstance(job, BlockingQueueWorker.Job):
                    raise ValueError(f"Job {job} not a BlockingQueueWorker.Job. You accessed jobQueue directly.")
                if isinstance(job, BlockingQueueWorker.PoisonPill):
                    job.isSwallowed(True)
                    self.taskDone()
                    return

                job.start()
                jobConsumer(*job)
                job.end()
                self.taskDone()
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

    def jobbify(self, paramsList: list, startTime: Future = None):
        job = None
        if isinstance(paramsList, BlockingQueueWorker.Job):
            job = paramsList
        elif isinstance(paramsList, BlockingQueueWorker.ProxiedChain):
            root = self.workChain(paramsList.pop(0))
            for chainLink in paramsList:
                root.then(chainLink)
            job = root
        else:
            job = BlockingQueueWorker.Job(paramsList=paramsList)

        if startTime:
            job.startTime = startTime
        return job

    class Job(list):
        def __init__(self, *, paramsList: list = None, startTime: Future = None, endTime: Future = None):
            super().__init__(paramsList)
            self.id = uuid.uuid4()
            self.startTime = startTime if startTime else Future()
            self.endTime = endTime if endTime else Future()
            self.block = Future()

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
        # Todo: Driver Chained jobs might benefit from avoiding ramp-down.
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

    def taskDone(self):
        """
        Only handle successful jobs
        If a doneQueue is provided send completed jobs there.
        """
        self.__jobQueue.task_done()
        if self.__doneQueue is not None:
            job = self.currentJob if not self.picklizeJobs else self.picklizeJob()
            self.__doneQueue.put(job)

    def picklizeJob(self):
        return (self.currentJob.id, self.currentJob.startTime.result(), self.currentJob.endTime.result())

    class Proxied(list):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)

    class ProxiedChain(list):
        def __init__(self, chain: 'BlockingQueueWorker.Chain'):
            node = chain.rootLink
            paramList = []
            while node:
                paramList.append(list(node))
                node = node.getNext()

            super().__init__(paramList)


class SameThreadExecutor:

    def __init__(self, name="Jon Doe Executor"):
        self.name = name

    def submit(self, fn, *args, **kwargs):
        try:
            fn(*args, **kwargs)
        except RuntimeError as e:
            print(f"Executor {self.name}: Task failed with error: {e}", traceback.format_exc())
            raise e

    def shutdown(self, wait, cancel_futures):
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

    index: int
    for index, (workerFuture, pill) in enumerate(workerPills):
        workerFuture.result()
        if not pill.isSwallowed():
            tprint(f"Worker died of other causes. {_WORKERS[index]}")


class MpQueue(Queue):
    def __init__(self, maxsize=0, doneQueue=None, ignoreTaskDone=True):
        super().__init__(maxsize=maxsize, ctx=mp.get_context())
        self.maxsize = self._maxsize
        self.doneQueue = doneQueue
        self.ignoreTaskDone = ignoreTaskDone

    def task_done(self):
        """
        Makes API compatible with queue.Queue()
        """
        pass
