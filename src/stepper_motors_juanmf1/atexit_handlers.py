import atexit
import signal
import threading

from stepper_motors_juanmf1 import BlockingQueueWorker
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint, flush_streams

# Register the cleanup function using atexit
atexit.register(flush_streams)
atexit.register(BlockingQueueWorker.killWorkers)


# Define a function to handle the interrupt signal (Ctrl+C)
def interrupt_handler(signum, frame):
    tprint(f"Received signal {signum}. Cleaning up before exit.")
    BlockingQueueWorker.killWorkers()
    flush_streams()
    # Get the default action for signum and execute it
    default_action = signal.getsignal(signum)
    default_action(signum, frame)
    exit(signum)

def kill_all_threads(signum, frame):
    for thread in threading.enumerate():
        if "MainThread" not in thread.name:
            thread._Thread__stop()
    flush_streams()
    # Get the default action for signum and execute it
    default_action = signal.getsignal(signum)
    default_action(signum, frame)

# Register the interrupt handler for the interrupt signal (Ctrl+C) or kill
signal.signal(signal.SIGINT, kill_all_threads)
signal.signal(signal.SIGTERM,  kill_all_threads)