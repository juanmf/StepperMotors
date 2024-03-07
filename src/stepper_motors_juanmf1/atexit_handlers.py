import os
import atexit
import signal
import time
import multiprocessing

from RPi import GPIO
from multiprocessing import current_process

from stepper_motors_juanmf1 import BlockingQueueWorker
from stepper_motors_juanmf1.ControllerFactory import MultiProcessingControllerFactory
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint, flush_streams

def terminateProcess(process):
    print(process.pid, signal.SIGKILL)
    process.terminate()
    time.sleep(0.2)
    os.kill(process.pid, signal.SIGKILL)

# Define a function to handle the interrupt signal (Ctrl+C)
def interrupt_handler(signum, frame):
    tprint(f"Received signal {signum}. Cleaning up before exit.")
    try:
        for process, proxyController in MultiProcessingControllerFactory.runningProcesses:
            terminateProcess(process)

        for process in multiprocessing.active_children():
            terminateProcess(process)


        process = current_process()
        # exit the process
        if process._popen is not None:
            # Accessing protected because processing does not make any safety check here.
            process.terminate()

        GPIO.cleanup()
        flush_streams()
        time.sleep(2)
    finally:
        os.kill(os.getpid(), signal.SIGKILL)
        exit(0)

# Register the cleanup function using atexit
atexit.register(interrupt_handler)

# Register the interrupt handler for the interrupt signal (Ctrl+C) or kill
signal.signal(signal.SIGINT, interrupt_handler)
signal.signal(signal.SIGTERM,  interrupt_handler)
