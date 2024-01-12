import atexit
import signal

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

# Register the interrupt handler for the interrupt signal (Ctrl+C) or kill
signal.signal(signal.SIGINT, interrupt_handler)
signal.signal(signal.SIGTERM, interrupt_handler)