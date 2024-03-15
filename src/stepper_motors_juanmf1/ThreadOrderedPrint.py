import logging
import time
from datetime import datetime
import threading
import _thread
import sys
import traceback
import os
from multiprocess.synchronize import process


os.environ['PYTHONUNBUFFERED'] = '1'

logging.basicConfig(level=logging.INFO)
_printStreams = {}
_interpreter = None
_globalPrintLock = threading.Lock()


def tprint(*args, sep=' ', end='\n', flush=False):
    """
    @param args: What to print.
    @param sep: what separates arguments.
    @param end: end of line.
    @param flush: If True, immediately flushes this thread alone, use `flush_streams()` for all.
    """
    # Micros
    global _printStreams
    timestamp = time.time_ns() // 1000
    message = sep.join(map(str, args)) + end
    thread_name, thread_id = get_current_thread_info()
    with _globalPrintLock:
        if thread_name not in _printStreams:
            _printStreams[thread_name] = {}

        _printStreams[thread_name][timestamp] = message
    if flush:
        flush_streams({thread_name: _printStreams.pop(thread_name)})


def flush_current_thread_only():
    global _printStreams
    thread_name, thread_id = get_current_thread_info()
    if thread_name in _printStreams:
        flush_streams({thread_name: _printStreams.pop(thread_name)})


def flush_streams_if_not_empty():
    global _printStreams
    if _printStreams:
        flush_streams()


def flush_streams(toPrint=None, stoppingApp=False):
    global _printStreams
    localPrintItems = _printStreams if toPrint is None else toPrint

    out = ""
    with _globalPrintLock:
        for thread_name, messages in localPrintItems.items():
            # Print the contents to stdout
            out += (f"\n@start thread dump {process.current_process().name} => {thread_name} ========================================\n"
                   + "==========================================================================\n"
                   + "\n".join([f"[{datetime.fromtimestamp(timestamp / 1e6).strftime('%H:%M:%S.%f')}] "
                                f"{message}" for timestamp, message in messages.items()])
                   + f"\n@end thread dump {thread_name} =========================================\n")
        out += ("\n===================================================================================\n"
                + "Thread prints Dump Complete =======================================================\n")

    try:
        if not stoppingApp:
            logging.info(out)
        else:
            os.write(sys.stdout.fileno(), out.encode())
    except RuntimeError as e:
        print(f"Logging error {e, traceback.format_exc()}")
    finally:
        if localPrintItems is _printStreams:
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
