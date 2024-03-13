class StepperMotor:
    """

    Abstract class for Stepper motor, a StepperMotor implementations are placeholder for motor specifics
    Actual behavior is implemented elsewhere (see Controller and AccelerationStrategy).

    Interesting reads
      https://www.orientalmotor.com/stepper-motors/technology/speed-torque-curves-for-stepper-motors.html
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
    TORQUE_CURVE = None
    TORQUE_UNIT = None
    SPR = None

    def __init__(self, *, minPps, maxPps, minSleepTime, maxSleepTime, spr, torqueCurve=None):
        """

        @param minPps: Ideally Min speed in PPS at which the motor shows continuity between steps.
        @param maxPps: Max PPS the motor can keep up with, note that it changes with load.
        @param minSleepTime: Sleep time between steps. Normally 1 / maxPps, in seconds
        @param maxSleepTime: Sleep time between steps. Normally 1 / minPps, in seconds
        @param spr: Steps per revolution. reciprocal to angle per step: (360 / anglePerStep) e.g. 1.8deg => 200
        @param torqueCurve: With Benchmark module find the max increase in PPS at each speed, used by
                            CustomAccelerationPerPps to reach max speed in minimum number of steps, under CURRENT load.
                            Format: [(minPPS, PPSIncrease1), (minPPS+PPSIncrease1, PPSIncrease2), ..., (maxPPS, 0)]
                            Note: CustomAccelerationPerPps ignores self.minPps & self.maxPps
        """
        # Sets GPIO pins
        # Todo: assess removal.
        # [Min/Max] Pulses per second
        self.minPps = minPps
        self.maxPps = maxPps

        # In Seconds
        self.maxSleepTime = maxSleepTime
        self.minSleepTime = minSleepTime
        
        # StepsPerRevolution
        self.spr = spr
        
        # For use with CustomAccelerationPerPps, to find out with Benchmark module. 
        self.torqueCurve = torqueCurve
            
    def getSpr(self):
        return self.spr
    
    def getMinPps(self):
        return self.minPps
    
    def getMaxPps(self):
        return self.maxPps

    def getMinSleepTime(self):
        return self.minSleepTime

    def getMaxSleepTime(self):
        return self.maxSleepTime

    def getTorqueCurve(self):
        return self.maxSleepTime


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

    MIN_PPS = 210
    # pps -> N-m x 10^-4
    TORQUE_UNIT = StepperMotor.TORQUE_UNIT_CN_CM

    """
    Gathered with module Benchmark
    format: (PPS, increment) Speeds up the fastest from 200 to 1044 PPS.
    TORQUE_CURVE = [(200, 300), (500, 200), (700, 100), (800, 50), (850, 25), (875, 25), (900, 1)]
    """
    # TORQUE_CURVE = [(180.0, 320), (500.0, 240), (740.0, 205), (945.0, 0)]
    # TORQUE_CURVE = [(200, 150), (350, 200), (550, 100), (650, 100), (750, 80), (830, 20), (850, 0)]
    # TORQUE_CURVE = [(200, 100), (300, 150), (450, 100), (550, 0)]
    TORQUE_CURVE = [(150, 160), (310, 160), (470, 20), (490, 0)]

    """
    Steps per Revolution with 1.8 deg per step
    """
    SPR = int(360 / 0.212)  # Steps per Revolution, 0.212 is angle per step in deg

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

    def __init__(self, loaded: bool = True, minPps=None):
        minPps = minPps if minPps else self.MIN_PPS
        
        super().__init__(maxPps=self.PPS_MAP[loaded], 
                         minSleepTime=PG35S_D48_HHC2.SLEEP_TIME_MAP[loaded],
                         maxSleepTime=1 / minPps,
                         minPps=minPps,
                         spr=PG35S_D48_HHC2.SPR,
                         torqueCurve=PG35S_D48_HHC2.TORQUE_CURVE)


#  Soon:
# STEPPERONLINE 0.9deg Nema 17 Stepper Motor Bipolar 1.5A 30Ncm
# Nema 17 Motor 42BYGH 1.8 Degree Body 38MM 4-Lead Wire 1.5A 42N.cm (60oz.in)
# STEPPERONLINE Nema 17 Stepper Motor Bipolar 2A 59Ncm(84oz.in) 48mm Body 4-Lead W


class Nema23_3Nm_23HS45_4204S(StepperMotor):
    """
    STEPPERONLINE High Torque Nema 23 CNC Stepper Motor 114mm 425oz.in/3Nm CNC Mill Lathe Router
    https://m.media-amazon.com/images/I/71D9bTPATQL.pdf
    https://m.media-amazon.com/images/I/A1DqXXSNDVL.pdf
    """
    # 30RPM * 200 / 60 Min speed in 71D9bTPATQL.pdf
    MIN_PPS = 100

    # pps -> N-m x 10^-4
    TORQUE_UNIT = StepperMotor.TORQUE_UNIT_N_M
    TORQUE = 3

    """
    Gathered with module Benchmark
    format: (PPS, increment) Speeds up the fastest from 200 to 1044 PPS.
    """
    TORQUE_CURVE = [(100, 150), (250, 0)]

    """
    Steps per Revolution with 1.8 deg per step
    """
    SPR = int(360 / 1.8)

    # Max tested functional speed was 1067
    LOADED_MAX_PPS = 4000  # DatasheetMax is 1500
    LOADED_SLEEP = 1 / 4000

    NOLOAD_MAX_PPS = 2000  # 850
    NOLOAD_SLEEP = 1 / 2000  # 0.000496

    PPS_MAP = {True: LOADED_MAX_PPS, False: NOLOAD_MAX_PPS}
    SLEEP_TIME_MAP = {True: LOADED_SLEEP, False: NOLOAD_SLEEP}

    def __init__(self, loaded: bool = True, minPps=None):
        minPps = minPps if minPps else self.MIN_PPS

        super().__init__(maxPps=self.PPS_MAP[loaded],
                         minSleepTime=Nema23_3Nm_23HS45_4204S.SLEEP_TIME_MAP[loaded],
                         maxSleepTime=1 / minPps,
                         minPps=minPps,
                         spr=Nema23_3Nm_23HS45_4204S.SPR,
                         torqueCurve=Nema23_3Nm_23HS45_4204S.TORQUE_CURVE)

class Nema17_59Ncm_17HS19_2004S1(StepperMotor):
    """
    Nema 17 Stepper Motor Bipolar 2A 59Ncm(84oz.in) 48mm Body 4-Lead W/ 1m Cable and Connector
    https://m.media-amazon.com/images/I/91zyUMD1hWL.pdf
    https://www.omc-stepperonline.com/download/17HS19-2004S1_Torque_Curve.pdf
    Brand	STEPPERONLINE
    Voltage	12 Volts
    Horsepower	1.4 hp
    Product Dimensions	1.65"W x 1.65"H
    Material	lead
    Item Weight	0.88 Pounds
    Manufacturer	OSM Technology Co.,Ltd.
    Part Number	17HS19-2004S1
    Item Weight	14.1 ounces
    Country of Origin	China
    Item model number	17HS19-2004S
    Size	1.65"x1.65"
    """
    # Based on torque curve, 400 PPS in half step => 200PPS on Full step.
    MIN_PPS = 200

    # Ncm = cNM
    TORQUE_UNIT = StepperMotor.TORQUE_UNIT_CN_M
    TORQUE = 59

    """
    Gathered with module Benchmark
    format: (PPS, increment) Speeds up the fastest from 200 to 1044 PPS.
    """
    TORQUE_CURVE = [(200, 150), (350, 0)]

    """
    Steps per Revolution with 1.8 deg per step
    """
    SPR = int(360 / 1.8)

    # Max tested functional speed was 1067
    LOADED_MAX_PPS = 2500  # DatasheetMax is 1500
    LOADED_SLEEP = 1 / 2500

    NOLOAD_MAX_PPS = 1000  # 850
    NOLOAD_SLEEP = 1 / 1000  # 0.000496

    PPS_MAP = {True: LOADED_MAX_PPS, False: NOLOAD_MAX_PPS}
    SLEEP_TIME_MAP = {True: LOADED_SLEEP, False: NOLOAD_SLEEP}

    def __init__(self, loaded: bool = True, minPps=None):
        minPps = minPps if minPps else self.MIN_PPS

        super().__init__(maxPps=self.PPS_MAP[loaded],
                         minSleepTime=Nema17_59Ncm_17HS19_2004S1.SLEEP_TIME_MAP[loaded],
                         maxSleepTime=1 / minPps,
                         minPps=minPps,
                         spr=Nema17_59Ncm_17HS19_2004S1.SPR,
                         torqueCurve=Nema17_59Ncm_17HS19_2004S1.TORQUE_CURVE)


class Nema17_42Ncm_17HS4401(StepperMotor):
    """
    Nema 17 Motor 42BYGH 1.8 Degree Body 38MM 4-Lead Wire1.5A 42N.cm (60oz.in)
    https://roboticx.ps/wp-content/uploads/2016/12/HB_Stepper_Motor_E.pdf

    Electrical Specification:
    Product type:Bipolar 42 Stepper Motor
    Step Angle: 1.8 deg.
    Rated Current/phase: 1.5A
    Holding Torque:42N.cm (60oz.in)

    General Specification:
    Step angle accuracy: + - 5%(full step,not load)
    Resistance accuracy: + - 10%
    Inductance accuracy: + - 20%
    Temperature rise: 80deg Max(rated current,2 phase on)
    Ambient temperature ----------20deg ~+50deg
    Insulation resistance:100M Min,500VDC
    Insultion Strength--------500VAC for one minute.
    """

    MIN_PPS = 200

    # Ncm = cNM
    TORQUE_UNIT = StepperMotor.TORQUE_UNIT_CN_M
    TORQUE = 59

    """
    Gathered with module Benchmark
    format: (PPS, increment) Speeds up the fastest from 200 to 1044 PPS.
    """
    TORQUE_CURVE = [(200, 150), (350, 150), (500, 150), (650, 150), (800, 100), (900, 50), (950, 50), (1000, 0)]

    """
    Steps per Revolution with 1.8 deg per step
    """
    SPR = int(360 / 1.8)

    # Max tested functional speed was 1067
    LOADED_MAX_PPS = 2500  # DatasheetMax is 1500
    LOADED_SLEEP = 1 / 2500

    NOLOAD_MAX_PPS = 1000  # 850
    NOLOAD_SLEEP = 1 / 1000  # 0.000496

    PPS_MAP = {True: LOADED_MAX_PPS, False: NOLOAD_MAX_PPS}
    SLEEP_TIME_MAP = {True: LOADED_SLEEP, False: NOLOAD_SLEEP}

    def __init__(self, loaded: bool = True, minPps=None):
        minPps = minPps if minPps else self.MIN_PPS

        super().__init__(maxPps=self.PPS_MAP[loaded],
                         minSleepTime=Nema17_42Ncm_17HS4401.SLEEP_TIME_MAP[loaded],
                         maxSleepTime=1 / minPps,
                         minPps=minPps,
                         spr=Nema17_42Ncm_17HS4401.SPR,
                         torqueCurve=Nema17_42Ncm_17HS4401.TORQUE_CURVE)


class Stepper_28BYJ_48(StepperMotor):
    """
    https://www.mouser.com/datasheet/2/758/stepd-01-data-sheet-1143075.pdf
    Rated voltage ： 5VDC
    Number of Phase 4
    Speed Variation Ratio 1/64
    Stride Angle 5.625° /64
    Frequency 100Hz
    DC resistance 50Ω±7%(25℃)
    Idle In-traction Frequency > 600Hz
    Idle Out-traction Frequency > 1000Hz
    In-traction Torque >34.3mN.m(120Hz)
    Self-positioning Torque >34.3mN.m
    Friction torque 600-1200 gf.cm
    Pull in torque 300 gf.cm
    Insulated resistance >10MΩ(500V)
    Insulated electricity power 600VAC/1mA/1s
    Insulation grade A
    Rise in Temperature <40K(120Hz)
    Noise <35dB(120Hz,No load,10cm)
    Model 28BYJ-48 – 5V
    """
    SPR = 2048
    MAX_PPS = 420
    """
    Pulse per second in Full step mode, It's steps per second.
    """
    MIN_PPS = 10

    def __init__(self, *, minPps=MIN_PPS, maxPps=MAX_PPS, minSleepTime=(1/MAX_PPS), maxSleepTime=(1/MIN_PPS), spr=SPR,
                 torqueCurve=None):
        super().__init__(maxPps=maxPps,
                         minSleepTime=minSleepTime,
                         maxSleepTime=maxSleepTime,
                         minPps=minPps,
                         spr=spr,
                         torqueCurve=torqueCurve)


class GenericStepper(StepperMotor):

    def __init__(self, *, minPps, maxPps, minSleepTime, maxSleepTime, spr=200, torqueCurve=None):
        super().__init__(maxPps=maxPps,
                         minSleepTime=minSleepTime,
                         maxSleepTime=maxSleepTime,
                         minPps=minPps,
                         spr=spr,
                         torqueCurve=torqueCurve)

    class Builder:
        def __init__(self, stepperMotor=None):
            self.minPps = None
            self.maxPps = None
            self.minSleepTime = None
            self.maxSleepTime = None
            self.spr = None
            self.torqueCurve = None
            if stepperMotor:
                self.copy(stepperMotor)

        def build(self):
            return GenericStepper(maxPps=self.maxPps,
                                  minSleepTime=self.minSleepTime,
                                  maxSleepTime=self.maxSleepTime,
                                  minPps=self.minPps,
                                  spr=self.spr,
                                  torqueCurve=self.torqueCurve)

        def copy(self, stepperMotor):
            self.minPps = stepperMotor.getMinPps()
            self.maxPps = stepperMotor.getMaxPps()
            self.minSleepTime = stepperMotor.getMinSleepTime()
            self.maxSleepTime = stepperMotor.getMaxSleepTime()
            self.spr = stepperMotor.getSpr()
            self.torqueCurve = list(stepperMotor.getTorqueCurve())
            return self

        def withSpr(self, spr):
            self.spr = spr
            return self

        def withMinPps(self, minPps):
            self.minPps = minPps
            if self.maxSleepTime is None:
                self.maxSleepTime = 1 / minPps
            return self

        def withMaxPps(self, maxPps):
            self.maxPps = maxPps
            if self.minSleepTime is None:
                self.minSleepTime = 1 / maxPps
            return self

        def withMinSleepTime(self, minSleepTime):
            self.minSleepTime = minSleepTime
            return self

        def withMaxSleepTime(self, maxSleepTime):
            self.maxSleepTime = maxSleepTime
            return self
