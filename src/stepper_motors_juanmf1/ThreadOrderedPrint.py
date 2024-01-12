import time
from io import StringIO
import threading
import _thread
import sys
import atexit
import signal

from src.stepper_motors_juanmf1 import BlockingQueueWorker

_printStreams={}
_interpreter=None
_globalPrintLock = threading.Lock()

def tprint(*args, sep=' ', end='\n'):
    thread_name, thread_id = get_current_thread_info()
    key = f"{thread_id}-{thread_name}"
    file = _printStreams.get(key, new_stream)
    print(*args, sep=sep, end=end, file=file)
    with _globalPrintLock:
        _printStreams.setdefault(thread_id, file)


def flush_streams():
    global _printStreams
    for key, stream in enumerate(_printStreams):
        # Print the contents to stdout
        out = ("\n@start thread dump ========================================\n"
               + key
               + "\n===========================================================\n"
               + stream.getvalue()
               + "\n@end thread dump ==========================================\n")
        print(out)
    with _globalPrintLock:
        _printStreams = {}


def new_stream():
    return StringIO()


def get_current_thread_info():
    if sys.version_info >= (3, 9):
        current_thread = threading.current_thread()
        thread_name = current_thread.name
        thread_id = current_thread.ident
    else:
        thread_name = threading.current_thread().name
        thread_id = _thread.get_ident()

    return thread_name, thread_id
