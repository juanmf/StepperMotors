import concurrent
import logging
from functools import partial

from multiprocess import Manager, Lock, Event

import queue

import sys
import threading
import time
import traceback
import uuid
from concurrent.futures import ThreadPoolExecutor, Future

import multiprocess as mp
from multiprocess.queues import Queue
from stepper_motors_juanmf1.MultiProcessShared import SharedManager

from stepper_motors_juanmf1.ThreadOrderedPrint import tprint, get_current_thread_info, flush_streams_if_not_empty, \
    flush_current_thread_only

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
                 picklizeJobs=False, isProxy=False, jobCompletionObserver=None):
        with _WORKERS_LOCK:
            workerName += f"_{BlockingQueueWorker.WORKER_COUNT}"
            BlockingQueueWorker.WORKER_COUNT += 1

        super().__init__(workerName, executor)
        # Multiprocess markers
        self.isMultiprocess = isinstance(jobQueue, MpQueue)
        self.jobQueueMaxSize = jobQueueMaxSize
        self.__jobQueue = jobQueue if jobQueue is not None else queue.Queue(self.jobQueueMaxSize)
        self.isProxy = isProxy
        self.jobCompletionObserver = jobCompletionObserver \
                                     if jobCompletionObserver else self.setupJobCompletionObserver(self.isProxy, self)
        self.proxiedJobs = {}

        self.picklizeJobs = picklizeJobs
        self.workerName = workerName
        self.workerThread = None
        self.workerAtWork = False
        self.lock = threading.Lock()
        # parent process side do not actually work on queued items.
        self.workerFuture = self.startWorker(jobConsumer)
        with _WORKERS_LOCK:
            _WORKERS.append(self)
        self.currentJob: BlockingQueueWorker.Job = None

    @staticmethod
    def setupJobCompletionObserver(isProxy, worker):
        if isProxy:
            manager = SharedManager.getInstance().getManager()
            sharedMemory = manager.dict()
            return MultiprocessObserver(eventObserver=partial(BlockingQueueWorker.jobDoneReaderCallback, worker),
                                        eventPublisher=BlockingQueueWorker.jobDoneWriterCallback,
                                        sharedMemory=sharedMemory)

    @staticmethod
    def jobDoneWriterCallback(sharedMemory, job):
        """
        Runs in child process after job is done.
        """
        BlockingQueueWorker.setSharedMemoryFutureResults('startTime', job.startTime, sharedMemory)
        BlockingQueueWorker.setSharedMemoryFutureResults('endTime', job.endTime, sharedMemory)
        BlockingQueueWorker.setSharedMemoryFutureResults('block', job.block, sharedMemory)
        if isinstance(job, BlockingQueueWorker.PoisonPill):
            sharedMemory['isSwallowed'] = job.isSwallowed()
        sharedMemory['id'] = job.jobId

    @staticmethod
    def setSharedMemoryFutureResults(propertyName, future, sharedMemory):
        if not future.exception():
            sharedMemory[propertyName] = future.result()
            sharedMemory[propertyName + 'Error'] = None
        else:
            sharedMemory[propertyName] = None
            sharedMemory[propertyName + 'Error'] = str(future.exception())

    @staticmethod
    def setJobFutureFromSharedMemory(propertyName, future, sharedMemory):
        if sharedMemory[propertyName + 'Error']:
            # Here we lose exception type raised in child process in favor of RuntimeError.
            future.set_exception(RuntimeError(sharedMemory[propertyName + 'Error']))
        else:
            future.set_result(sharedMemory[propertyName])

    def jobDoneReaderCallback(self, sharedMemory):
        """
        Main Process reads values from done job
        """
        doneJob: BlockingQueueWorker.Job = self.proxiedJobs.pop(sharedMemory['id'])
        BlockingQueueWorker.setJobFutureFromSharedMemory('startTime', doneJob.startTime, sharedMemory)
        BlockingQueueWorker.setJobFutureFromSharedMemory('endTime', doneJob.endTime, sharedMemory)
        BlockingQueueWorker.setJobFutureFromSharedMemory('block', doneJob.block, sharedMemory)
        if isinstance(doneJob, BlockingQueueWorker.PoisonPill):
            doneJob.isSwallowed(sharedMemory.pop('isSwallowed'))
            self.workerFuture.set_result(True)

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

    def work(self, paramsList, block=False, startTime: Future = None) -> 'BlockingQueueWorker.Job':
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
            flush_current_thread_only()
            if isinstance(paramsList, BlockingQueueWorker.Chain):
                proxyJob = BlockingQueueWorker.ProxiedChain(job)
            elif isinstance(paramsList, BlockingQueueWorker.PoisonPill):
                proxyJob = BlockingQueueWorker.ProxiedPoisonPill(job)
            else:
                proxyJob = BlockingQueueWorker.Proxied(job)

            self.__jobQueue.put(proxyJob)
            return job

        self.__jobQueue.put(job, block=block)
        return job

    def killWorker(self) -> 'BlockingQueueWorker.PoisonPill':
        pill = BlockingQueueWorker.PoisonPill(worker=self)
        self.work(pill, block=True)
        time.sleep(0.1)
        self.executor.shutdown()
        return pill

    def startWorker(self, jobConsumer) -> Future:
        if self.isProxy:
            # Todo: never complete. the child process might end, we don't have visibility. we could use this thread
            #  to propagate events on parent process side.
            workerFuture = Future()
            workerFuture.set_running_or_notify_cancel()
            return workerFuture

        if self.workerAtWork:
            raise RuntimeError("There is already a worker at work.")
        return self.executor.submit(lambda: self.consumer(jobConsumer))

    def getJobQueue(self):
        return self.__jobQueue

    def consumer(self, jobConsumer):
        """
        restarts consumer on swallowed exceptions.
        """
        while self.__doConsume(jobConsumer):
            pass

        tprint("Worker swallowed poison pill.")
        if self.isMultiprocess:
            self.__jobQueue.close()

        flush_current_thread_only()
        if self.isProxy:
            self.workerFuture.set_result(True)
        return True

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
                    flush_streams_if_not_empty()
                # Block until movement job is sent our way.
                job = self.__jobQueue.get(block=True)
                if isinstance(job, BlockingQueueWorker.Proxied):
                    # Proxied are job descriptions with no Futures or any non-picklable objects.
                    job = self.jobbify(job)

                # Todo: unset currentJob on error and when done?
                self.currentJob = job
                if not isinstance(job, BlockingQueueWorker.Job):
                    raise ValueError(f"Job {job} not a BlockingQueueWorker.Job. You accessed jobQueue directly.")
                if isinstance(job, BlockingQueueWorker.PoisonPill):
                    job.isSwallowed(True)
                    self.taskDone()
                    return False

                job.start()
                jobConsumer(*job)
                job.end()
                self.taskDone()
                if isinstance(job, BlockingQueueWorker.Chain):
                    self._handleChainOfWork(job)

        except Exception as e:
            throw = RuntimeError("Worker job failed.", job, e)
            self.taskErrored(job, throw)
            tprint(f"Swallowing exception in worker {self.workerThread}; Error: {throw}\n{traceback.format_exc()}")
        finally:
            with self.lock:
                self.workerAtWork = False
        return True

    def _handleChainOfWork(self, job: 'BlockingQueueWorker.Chain'):
        if job.getNext() is not None:
            # consumer thread producing, can't block!
            self.work(job.getNext(), block=False)

    def __str__(self):
        # thread_id = threading.current_thread().ident

        return (f"BlockingQueueWorker {self.workerName} "
                f"Is Proxy: {self.isProxy}\n"
                f"with job Queue {self.__jobQueue} of probable len {self.__jobQueue.qsize()} \n"
                f"jobList max len {self.__jobQueue.maxsize}\n"
                f"worker thread ({self.workerThread})")

    def jobbify(self, paramsList: list, startTime: Future = None):
        job = None
        if isinstance(paramsList, BlockingQueueWorker.Job):
            job = paramsList
        elif isinstance(paramsList, BlockingQueueWorker.ProxiedChain):
            node = self.workChain(paramsList.pop(0))
            for chainLink in paramsList:
                node = node.then(chainLink)
            job = node.rootLink
        elif isinstance(paramsList, BlockingQueueWorker.ProxiedPoisonPill):
            job = BlockingQueueWorker.PoisonPill(worker=self)
        else:
            job = BlockingQueueWorker.Job(worker=self, paramsList=paramsList)

        if startTime:
            job.startTime = startTime
        return job

    class Job(list):
        def __init__(self, *, worker: 'BlockingQueueWorker',
                     paramsList: list = None,
                     startTime: Future = None,
                     endTime: Future = None):
            super().__init__(paramsList)
            self.jobId = paramsList.jobId if isinstance(paramsList, BlockingQueueWorker.Proxied) else uuid.uuid4()
            self.worker = worker
            if worker.isProxy:
                worker.proxiedJobs[self.jobId] = self

            self.startTime = startTime if startTime else Future()
            self.endTime = endTime if endTime else Future()
            self.block = Future()

        def setException(self, throwable):
            if not self.startTime.done():
                tprint(f"Job didn't even start. Setting error to both startTime and endTime Futures.")
                self.startTime.set_exception(throwable)
                self.endTime.set_exception(throwable)
                not self.block.done() and self.block.set_exception(throwable)
            elif not self.endTime.done():
                tprint(f"Job started but could not complete. Setting error to endTime Future.")
                self.endTime.set_exception(throwable)
                not self.block.done() and self.block.set_exception(throwable)
            else:
                tprint(f"Job Futures Done when trying to set exception. Job:{self}.")

        def start(self):
            startTime = time.monotonic_ns()
            self.startTime.set_result(startTime)
            return startTime

        def end(self):
            endTime = time.monotonic_ns()
            self.endTime.set_result(time.monotonic_ns())
            not self.block.done() and self.block.set_result(True)
            return endTime

        def result(self):
            # Blocks caller until job is done. When JobConsumer sets endTime in ns
            return self.block.result()

    class Chain(Job):
        # Todo: Driver Chained jobs might benefit from avoiding ramp-down.
        def __init__(self, worker: 'BlockingQueueWorker', *, paramsList: list = None,
                     prev: 'BlockingQueueWorker.Chain' = None):
            super().__init__(worker=worker, paramsList=[] if paramsList is None else paramsList)
            self.workStarted = False
            self._next = None
            self._prev = prev
            if prev is None:
                # root link
                self.rootLink: BlockingQueueWorker.Chain = self
                self.chainEndTime = Future()
            else:
                self.rootLink: BlockingQueueWorker.Chain = prev.rootLink
                self.chainEndTime = self.rootLink.chainEndTime

        def then(self, paramsList: list):
            if self.workStarted:
                raise RuntimeError("Work chain already started.")
            self._next = BlockingQueueWorker.Chain(self.worker, paramsList=paramsList, prev=self)
            return self._next

        def getNext(self) -> 'BlockingQueueWorker.Chain':
            return self._next

        def getPrev(self):
            return self._prev

        def startChain(self, startTime: Future = None):
            from multiprocess.synchronize import process
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
        def __init__(self, *, worker):
            super().__init__(worker=worker, paramsList=[])
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
        """
        self.__jobQueue.task_done()
        if self.isMultiprocess:
            MultiprocessObserver.eventPublisher(self.jobCompletionObserver, self.currentJob)

    def taskErrored(self, job, error):
        job.setException(error)
        if self.isMultiprocess:
            MultiprocessObserver.eventPublisher(self.jobCompletionObserver, self.currentJob)

    def picklizeJob(self):
        return (self.currentJob.jobId, self.currentJob.startTime.result(), self.currentJob.endTime.result())

    class Proxied(list):
        def __init__(self, job):
            super().__init__(job)
            self.jobId = job.jobId if isinstance(job, BlockingQueueWorker.Job) else None

    class ProxiedPoisonPill(Proxied):
        def __init__(self, job):
            super().__init__(job)

    class ProxiedChain(Proxied):
        def __init__(self, chain: 'BlockingQueueWorker.Chain'):
            node = chain.rootLink
            paramList = []
            while node:
                paramList.append(BlockingQueueWorker.Proxied(node))
                node = node.getNext()
            super().__init__(paramList)


class SameThreadExecutor:

    def __init__(self, name="Jon Doe Executor"):
        self.name = name

    def submit(self, fn, *args, **kwargs):
        try:
            fn(*args, **kwargs)
        except RuntimeError as e:
            tprint(f"Executor {self.name}: Task failed with error: {e}", traceback.format_exc())
            raise e

    def shutdown(self, wait, cancel_futures):
        pass

class ThreadPoolExecutorStackTraced(ThreadPoolExecutor):

    def __init__(self, isDaemon=True, *args, **kwargs):
        self.isDaemon = isDaemon
        super().__init__( *args, **kwargs)

    def submit(self, fn, *args, **kwargs):
        """Submits the wrapped function instead of `fn`"""
        future = super(ThreadPoolExecutorStackTraced, self).submit(
            self._function_wrapper, fn, *args, **kwargs)

        if self.isDaemon:
            for t in self._threads:
                concurrent.futures.thread._threads_queues.pop(t, None)

        return future

    def shutdown(self, wait: bool = True, *, cancel_futures: bool = False) -> None:
        for t in self._threads:
            concurrent.futures.thread._threads_queues.pop(t, None)

        rs = super().shutdown(wait, cancel_futures=cancel_futures)
        return rs

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

class MpQueue(Queue):
    def __init__(self, maxsize=0, ignoreTaskDone=True):
        super().__init__(maxsize=maxsize, ctx=mp.get_context())
        self.maxsize = self._maxsize
        self.ignoreTaskDone = ignoreTaskDone

    def task_done(self):
        """
        Makes API compatible with queue.Queue()
        """
        pass


class MultiprocessObserver(BlockingQueueWorker):
    instanceCountLock = Lock()
    count = 0

    def __init__(self, *, eventObserver, eventPublisher, sharedMemory):
        """
        Sets up an observed multiprocess.Event to trigger behavior in one Process when Event is set in the other.
        @param eventObserver: Client side, observer blocks on Event waiting for it to be set, then eventObserver fn
                              operates on sharedMemory. Contract: eventObserver(sharedMemory)
        @param eventPublisher: Server side (most likely Child Process), sets sharedMemory accordingly and then Event
                               is set. Contract: eventPublisher(sharedMemory, *args, **kwargs)
        @param sharedMemory: shared data between two Python processes.
        """
        with MultiprocessObserver.instanceCountLock:
            super().__init__(partial(self.eventObserver, self),
                             jobQueueMaxSize=1, workerName=f"MultiprocessObserver_{self.count}")
            MultiprocessObserver.count += 1

        self._eventObserver = eventObserver
        self._eventPublisher = eventPublisher
        self._sharedMemory = sharedMemory

        self.observedEvent = Event()
        self.observerLock = Lock()
        # start observing
        self.work([])

    @staticmethod
    def eventObserver(instance: 'MultiprocessObserver'):
        while True:
            instance.observedEvent.wait()
            with instance.observerLock:
                instance._eventObserver(instance._sharedMemory)
            instance.observedEvent.clear()

    @staticmethod
    def eventPublisher(instance: 'MultiprocessObserver', *args, **kwargs):
        with instance.observerLock:
            instance._eventPublisher(instance._sharedMemory, *args, **kwargs)

        instance.observedEvent.set()


def killWorkers():
    """
    kills all workers when program tears down.
    """
    workerPills = []
    for worker in _WORKERS:
        workerPills.append((worker, worker.workerFuture, worker.killWorker()))

    index: int
    for index, (worker, workerFuture, pill) in enumerate(workerPills):
        try:
            workerFuture.result(timeout=2)
        except TimeoutError as e:
            logging.error(f"Error: Worker {worker} didn't finish.")
            raise e
        if not pill.isSwallowed():
            # Todo: in single process scenario this always prints for some or all workers. Why?
            tprint(f"Worker died of other causes. {_WORKERS[index]}")

