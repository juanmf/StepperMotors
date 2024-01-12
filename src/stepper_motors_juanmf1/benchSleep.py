import time
import numpy as np

from stepper_motors_juanmf1.ThreadOrderedPrint import tprint


def mt(s):
    start = time.time()
    time.sleep(s)
    end = time.time()
    tprint(f"slept {end - start} seconds")
    return end - start


def benchSleep(s):
    diff = []
    slept = []
    for i in range(30):
        slept.append(mt(s))
        diff.append(slept[-1] - s)
    meandiff = np.mean(diff)
    stddiff = np.std(diff)
    meanslept = np.mean(slept)
    stdslept = np.std(slept)
    tprint(
        f"Actual sleep differs by net avg {meandiff} or {100 * meandiff / s}%; with std dev {stddiff} or {100 * stddiff / s}%; ")
    tprint(
        f"Actual sleep differs by net avg {meanslept} or {100 * meanslept / s}%; with std dev {stdslept} or {100 * stdslept / s}%; ")
