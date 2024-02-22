import atexit
import signal
from RPi import GPIO
from multiprocessing import current_process

from stepper_motors_juanmf1 import BlockingQueueWorker
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint, flush_streams

# Define a function to handle the interrupt signal (Ctrl+C)
def interrupt_handler(signum, frame):
    tprint(f"Received signal {signum}. Cleaning up before exit.")
    BlockingQueueWorker.killWorkers()
    GPIO.cleanup()
    flush_streams()
    # exit the process
    process = current_process()
    if process._popen is not None:
        # Accessing protected because processing does not make any safety check here.
        process.terminate()
    default_action = signal.getsignal(signum)
    default_action(signum, frame)
    exit(0)

# Register the cleanup function using atexit
atexit.register(interrupt_handler)

# Register the interrupt handler for the interrupt signal (Ctrl+C) or kill
signal.signal(signal.SIGINT, interrupt_handler)
signal.signal(signal.SIGTERM,  interrupt_handler)
