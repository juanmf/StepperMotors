import time
from io import StringIO
import threading
import _thread
import sys
from datetime import datetime


_printStreams = {}
_interpreter = None
_globalPrintLock = threading.Lock()


def tprint(*args, sep=' ', end='\n'):
    # Complete timestamp would be "%Y-%m-%d %H:%M:%S.%f"
    timestamp = datetime.now().strftime("%H:%M:%S.%f")
    prefix = f"[{timestamp}]"
    message = sep.join(map(str, args))

    thread_name, thread_id = get_current_thread_info()
    key = f"{thread_id}_{thread_name}"
    file = _printStreams[key] if key in _printStreams else StringIO()
    print(f"{prefix} {message}", end=end, file=file)
    with _globalPrintLock:
        _printStreams[key] = file


def flush_streams():
    global _printStreams
    for key, stream in _printStreams.items():
        # Print the contents to stdout
        out = (f"\n@start thread dump {key} ========================================\n"
               + "======================================================================\n"
               + stream.getvalue()
               + "\n@end thread dump =====================================================\n")
        print(out)
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
