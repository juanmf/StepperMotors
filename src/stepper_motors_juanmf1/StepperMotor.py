import threading



class StepperMotor:
    """
    Abstract class for Stepper motor, a StepperMotor implementations are placeholder for motor specifics
    Actual behavior is implemented elsewhere (see Controller and AccelerationStrategy).

    Interesting reads
      https://us.metoree.com/categories/stepping-motor/
      https://forum.arduino.cc/t/stepper-motor-basics/275223
    """
    TORQUE_UNIT_N_M = "N-m"
    # 1 "N-m" = 141.612 "oz-in"
    TORQUE_UNIT_OZ_IN = "oz-in"
    # 1 N-m = 100 cN-m
    TORQUE_UNIT_CN_M = "cN-m"
    # 1 "N-m x 10^-4" = 0.0001 N-m used in some Torque charts.
    TORQUE_UNIT_CN_CM = "cN-cm"

    TORQUE_EQUIVALENCE = {TORQUE_UNIT_N_M: 1,
                          TORQUE_UNIT_OZ_IN: 141.612,
                          TORQUE_UNIT_CN_M: 0.01,
                          TORQUE_UNIT_CN_CM: 0.0001}

    # Must be overriden
    MIN_PPS = None
    # Must be a map pps -> torque
    TORQUE_CHARACTERISTICS = None
    TORQUE_UNIT = None
    SPR = None

    def __init__(self, pps, sleepTime, minPps, spr):
        super().__init__()
        # Sets GPIO pins
        # Todo: assess removal.
        self.settingsLock = threading.Lock()
        self.sleepTime = sleepTime
        # Pulses per second
        self.pps = pps
        # Steps per revolution
        if self.MIN_PPS is None:
            self.MIN_PPS = minPps
        self.minSleepTime = 1 / self.MIN_PPS
        self.spr = spr

    def readSettings(self):
        with self.settingsLock:
            pps = self.pps
            sleepTime = self.sleepTime
        return pps, sleepTime

    def changeSettings(self, pps, sleepTime):
        with self.settingsLock:
            self.pps = pps
            self.sleepTime = sleepTime


class PG35S_D48_HHC2(StepperMotor):
    """
    Motor PG35S-D48-HHC2
    https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/571/PG35S-D48-HHC2_Standard.pdf
    https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/15/PG35S-D48-HHC2_Spec.pdf
    https://nmbtc.com/parts/pg35s-d48-hhc2/
    https://www.eevblog.com/forum/reviews/solder-pasteflux-manual-syringe-applicator/?action=dlattach;attach=174358
    pull out torque: 200pps ... 392.3 mNm MIN
    max no load response: 850pps
    max slew speed: 1500 pps
    driver: peak current 500mA
    driver: rated current 250mA
    driver: voltage 24V
    driver mode constant current
    Life 3000h
    """

    MIN_PPS = None
    MAX_TORQUE_PPS = None
    # pps -> N-m x 10^-4
    TORQUE_UNIT = StepperMotor.TORQUE_UNIT_CN_CM

    """
    Gathered with module Benchmark
    format: (PPS, increment) Speeds up the fastest from 200 to 1044 PPS.
    TORQUE_CHARACTERISTICS = [(200, 300), (500, 200), (700, 100), (800, 50), (850, 25), (875, 25), (900, 1)]
    """
    # TORQUE_CHARACTERISTICS = [(180.0, 320), (500.0, 240), (740.0, 205), (945.0, 0)]
    # TORQUE_CHARACTERISTICS = [(200, 150), (350, 200), (550, 100), (650, 100), (750, 80), (830, 20), (850, 0)]
    # TORQUE_CHARACTERISTICS = [(200, 100), (300, 150), (450, 100), (550, 0)]
    TORQUE_CHARACTERISTICS = [(200, 150), (350, 0)]

    """
    Steps per Revolution with 1.8 deg per step
    """
    SPR = 360 // 0.212  # Steps per Revolution, 0.212 is angle per step in deg

    # A Pulse implies full period. so signal to GPIO doubles this freq.
    # use benchSleep to find RPI overhead near the specified sleep time (the lower sleep time the greater the overhead):
    # >>> benchSleep(1/(900)*.73)
    # slept 0.0003504753112792969 seconds
    # slept 0.0003421306610107422 seconds
    # Actual sleep differs by net avg 9.664361317952476e-05 or 39.71655336144853%; \
    #     with std dev 2.3774077176703874e-06 or 0.9770168702755017%;
    # Actually slept on avg 0.00033751328786214195 or 138.72309406582076%; \
    #     with std dev 5.701971382635937e-06 or 2.3435969513505697%;
    # LOADED_MAX_PPS = 1300 # DatasheetMax is 1500
    # LOADED_SLEEP_HIGH_LOW = 0.00030 # Benchmarked for 1500PPS was: 0.0002433

    # Max tested functional speed was 1067
    LOADED_MAX_PPS = 400  # DatasheetMax is 1500
    LOADED_SLEEP = 1 / 400

    # target sleep: 1/(2*850) = 0.000588
    # >>> benchSleep(0.000496)
    # slept 0.0006048679351806641 seconds
    # slept 0.0005941390991210938 seconds
    # Actual sleep differs by net avg 9.250990549723315e-05 or 18.50198109944663%; \
    #     with std dev 2.652120187789779e-06 or 0.5304240375579559%;
    # Actual sleep differs by net avg 0.0005880355834960938 or 118.55556118872857%; \
    #     with std dev 3.174699532262383e-06 or 0.6400603895690288%;
    # NOLOAD_MAX_PPS = 750 # 850
    # NOLOAD_SLEEP_HIGH_LOW = 0.00058 # 0.000496

    NOLOAD_MAX_PPS = 800  # 850
    NOLOAD_SLEEP = 1 / 800  # 0.000496

    PPS_MAP = {True: LOADED_MAX_PPS, False: NOLOAD_MAX_PPS}
    SLEEP_TIME_MAP = {True: LOADED_SLEEP, False: NOLOAD_SLEEP}

    def __init__(self, loaded: bool = True, minPps=210):
        super().__init__(self.PPS_MAP[loaded], self.SLEEP_TIME_MAP[loaded], minPps, spr=self.SPR)


class GenericStepper(StepperMotor):

    def __init__(self, *, maxPps, minPps=150, spr=200):
        super().__init__(maxPps, 1 / maxPps, minPps, spr)
