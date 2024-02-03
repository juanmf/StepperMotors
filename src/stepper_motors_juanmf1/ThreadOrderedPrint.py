import logging
import time
from datetime import datetime
import threading
import _thread
import sys
import traceback
import os
os.environ['PYTHONUNBUFFERED'] = '1'

logging.basicConfig(level=logging.INFO)
_printStreams = {}
_interpreter = None
_globalPrintLock = threading.Lock()


def tprint(*args, sep=' ', end='\n'):
    # Micros
    timestamp = time.time_ns() // 1000
    message = sep.join(map(str, args)) + end
    thread_name, thread_id = get_current_thread_info()
    with _globalPrintLock:
        if thread_name not in _printStreams:
            _printStreams[thread_name] = {}

        _printStreams[thread_name][timestamp] = message


def flush_streams():
    global _printStreams
    out = ""
    for thread_name, messages in _printStreams.items():
        # Print the contents to stdout
        out += (f"\n@start thread dump {thread_name} ========================================\n"
               + "==========================================================================\n"
               + "\n".join([f"[{datetime.fromtimestamp(timestamp / 1e6).strftime('%H:%M:%S.%f')}] "
                            f"{message}" for timestamp, message in messages.items()])
               + f"\n@end thread dump {thread_name} =========================================\n")
    out += ("\n===================================================================================\n"
            + "Thread prints Dump Complete =======================================================\n")

    try:
        logging.info(out)
    except RuntimeError as e:
        print(f"Logging error {e, traceback.format_exc()}")

    with _globalPrintLock:
        _printStreams = {}


def get_current_thread_info():
    if sys.version_info >= (3, 9):
        current_thread = threading.current_thread()
        thread_name = current_thread.name
        thread_id = current_thread.ident
    else:
        thread_name = threading.current_thread().name
        thread_id = _thread.get_ident()

    return thread_name, thread_id
