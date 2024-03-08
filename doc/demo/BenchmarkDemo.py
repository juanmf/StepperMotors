import signal
import sys
from sshkeyboard import stop_listening

from stepper_motors_juanmf1.Benchmark import Benchmark
from stepper_motors_juanmf1.ControllerFactory import DynamicControllerFactory
from stepper_motors_juanmf1.StepperMotor import Nema17_42Ncm_17HS4401
from stepper_motors_juanmf1.ThreadOrderedPrint import tprint


def main():
    args = sys.argv[1:]
    if len(args) > 0 and args[0] == "bench":
        tprint("Benchmarking azimuth Motor")
        # Benchmark.initBenchmark(PG35S_D48_HHC2(True), 25, 8)
        factory = DynamicControllerFactory()
        motor = Nema17_42Ncm_17HS4401(True)
        builder = factory.getInteractiveDriverBuilder(motor,
                                                      directionGpioPin=13,
                                                      stepGpioPin=19,
                                                      sleepGpioPin=12,
                                                      minSpeedDelta=Benchmark.MIN_PPS_DELTA,
                                                      minPps=Benchmark.ABSOLUTE_MIN_PPS)
        # Could use any driver
        driver = builder.buildDRV8825Driver()
        Benchmark.initBenchmark(driver)


# Preserve the original signal handlers
original_sigint_handler = signal.getsignal(signal.SIGINT)
original_sigterm_handler = signal.getsignal(signal.SIGTERM)


# Define your custom signal handler
def my_interrupt_handler(signum, frame):
    # Call your custom handler
    stop_listening()
    # Then original signal handlers first
    if original_sigint_handler is not None:
        original_sigint_handler(signum, frame)
    if original_sigterm_handler is not None:
        original_sigterm_handler(signum, frame)


if __name__ == '__main__':
    # Register your custom interrupt handler
    signal.signal(signal.SIGINT, my_interrupt_handler)
    signal.signal(signal.SIGTERM, my_interrupt_handler)
    main()
