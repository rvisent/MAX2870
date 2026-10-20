# MAX2870
Arduino library for the MAX2870 wideband frequency synthesizer (23.5 MHz to 6 GHz, Maxim / Analog Devices), with a demo sketch and a PC control panel. Tested on Arduino Due with the common low-cost MAX2870 boards.

+ **Exact frequency setting**: the best FRAC/MOD pair is found with an integer-only continued fraction algorithm; the actual frequency and its error are reported after every setting.
+ **Sweeps without unlocking**: a VCO lookup table, calibrated in about 2 s from the VTUNE ADC of the chip, lets sweeps keep the VCO instead of running the 200 us VCO autoselect at every step.
+ **Register 6 readback**: active VCO and VTUNE ADC, checked automatically at startup.
+ **Ready to use**: the demo sketch accepts text commands on the serial port (frequency, power, sweeps, calibration, register dump), and a Python/tkinter control panel drives it.
+ **Practical notes** on the low-cost boards: unused output termination, the -4 dBm level, the readback wiring.
+ No additional libraries required.

Quick start: wire the board to an Arduino Due (LE 48, lock detect 50, CE 52, SPI header; MUX_OUT to MISO for the readback), install the library (see [Installation](#installation)), upload File > Examples > MAX2870 > my2870, then run `python tools/max2870_gui.py` and press CAL once to build the VCO table.

## Introduction

This library supports the MAX2870 from Maxim (now Analog Devices) on Arduinos. The chip is a wideband (23.4375 MHz to 6 GHz) Phase-Locked Loop (PLL) and Voltage Controlled Oscillator (VCO), covering a very wide frequency range
under digital control. Just add an external PLL loop filter, Reference frequency source and a power supply for a very useful 
frequency generator for applications as a Local Oscillator or Sweep Generator.  

The chip generates the frequency using a programmable Fractional-N and Integer-N Phase-Locked Loop (PLL) and Voltage Controlled Oscillator (VCO) with an external loop filter and frequency reference. The chip is controlled by 
a SPI interface, which is controlled by a microcontroller such as the Arduino.

The library provides an SPI control interface for the MAX2870, and also provides functions to calculate and set the frequency, which greatly simplifies the integration of this chip into a design. Frequencies are given in Hz as "long long" (64 bit) values. For any requested frequency the library selects the output divider, INT, and the FRAC/MOD pair giving the closest achievable frequency (MOD <= 4095); integer-N mode is used automatically when FRAC is 0. Code was tested on an Arduino Due; it should compile on other architectures, but 64 bit arithmetic is slow on 8 bit Arduinos. The library also exposes all of the PLL variables, such as FRAC, MOD and INT, so they can be examined as needed.  

Doesn't require any additional library.

A low phase noise stable oscillator is required for this module. Typically, an Ovenized Crystal Oscillator (OCXO) in the 10 MHz to 100 MHz range is used.  

## Features

+ Frequency Range: 23.4375 MHz to 6 GHz
+ Output Level: -4 dBm to 5 dBm (in 3 dB steps) 
+ In-Band Phase Noise: -95 dBc/Hz (10 kHz from 4.227 Ghz carrier)
+ PLL Modes: Fraction-N and Integer-N (set automatically)
+ Frequency setting with 1 Hz input resolution: the actual frequency is the closest one achievable with MOD <= 4095, the error is at most fPFD/8190 divided by the output divider (e.g. about 1.2 kHz with fPFD = 10 MHz above 3 GHz), usually much less. The actual frequency and the error are available after each setting
+ Signal On/Off control
+ All MAX2870_R[] registers can be accessed and manipulated

## Demo sketch and PC control panel

The demo sketch [my2870](examples/my2870) (pins for Arduino Due: LE 48, lock detect 50, CE 52) accepts commands on the serial port at 115200 baud, one per line, case insensitive:

| Command | Function |
|---|---|
| `S` or `STATUS` | print status |
| `F freq_Hz [power]` | set frequency, power 0 (off) or 1-4 (-4/-1/+2/+5 dBm); the sketch starts at level 2 |
| `FD delta_Hz` | change frequency by delta from the last requested frequency |
| `P power` | set main output power level |
| `REF ref_Hz R [NONE\|HALF\|DOUBLE]` | set reference, R divider and doubler/halver, then reapply the frequency |
| `CP current_mA` | charge pump current |
| `PD INV\|NONINV` | phase detector polarity |
| `CE ON\|OFF` | chip enable |
| `SWEEP start_Hz stop_Hz step_Hz dwell_ms [power] [LOG\|LOGALL]` | repeated sweep, stopped by any received line. With LOG the lock detect pin is monitored during the dwell time and a line `L f=<Hz> n=<unlock pulses> low=<total unlock us> lock=<us to final lock> vco=<VCO, -1 autoselect> [UNLOCKED]` is printed for anomalous steps: unlocked at the end, more than 150 us unlocked or more than 3 unlock pulses (LOGALL: every step) |
| `CAL [step_MHz] [AUTO\|LOCK\|TUNE]` | calibrate the VCO table (RF outputs muted), then print it as `T vco=<n> min=<MHz> max=<MHz> kind=<LOCK\|TUNE>` lines; AUTO (default) uses TUNE if the register 6 readback works |
| `VTAB [vco min_MHz max_MHz [LOCK\|TUNE]]` | print the VCO table, or set one entry (to restore a saved table) |
| `VMODE AUTO\|TABLE [margin_MHz]` | VCO selection: autoselect at every frequency change, or from the table (see below; default margin 60 MHz for a LOCK table, 5 for a TUNE table) |
| `VAS ON\|OFF` | VCO autoselect on, or off keeping the current VCO (manual mode) |
| `VCO n` | manual VCO selection 0-63 |
| `FRAC ON\|OFF` | force fractional mode also when FRAC = 0 (recommended for sweeps) |
| `FT freq_Hz [watch_ms]` | set the frequency and measure the lock time on the lock detect pin |
| `VSCAN freq_Hz [settle_ms [watch_ms]]` | lock test of all 64 VCOs at one frequency |
| `REG` / `REG n hex` / `REG W` | dump R0-R5 and the R6 readback / write one register / write all registers |
| `BURST on_us off_us count [AUX]` | on/off bursts, count 0 = until any received line |
| `FREQ_DIRECT R INT MOD FRAC outdiv FRACMODE(0\|1)` | set PLL parameters directly |
| `HELP` | list of commands |

Every command is answered by one line `OK` or `ERR <code> <message>`. Commands changing the synthesizer state first print a status line:

`S f=<actual Hz> req=<requested Hz> err=<req-actual Hz> ref=<Hz> r=<R> refmode=<NONE|HALF|DOUBLE> pfd=<Hz> int=<N> frac=<F> mod=<M> div=<outdiv> pwr=<0-4> aux=<0-4> cp=<mA> pd=<INV|NONINV> ce=<0|1> lock=<0|1> vmode=<AUTO|TABLE|MANUAL> vco=<VCO, -1 = unknown> adc=<VTUNE ADC 0-7, -1 = no readback> ffrac=<0|1>`

Lines starting with `#` are informational, `R` lines are register dumps, `L` lines the sweep lock log, `T` lines the VCO table, `V` lines the VSCAN results.

### VCO table mode

With the VCO autoselect (VAS) every write of register 0 starts a new VCO selection (10/fBS = 200 us) during which the loop is open: in a sweep the PLL unlocks at every step, and the autoselect can choose a VCO working with VTUNE near its limits, or alternate between overlapping sub-bands at consecutive steps. Following the datasheet (Applications Information, VCO manual selection), `CAL` builds a table of the 64 VCO sub-bands, then `VMODE TABLE` selects the VCO from the table: the most centered one for a new frequency, and in sweeps the current one is kept while it is at least `margin_MHz` from its limits.

Two calibration methods, chosen at run time (`CAL AUTO`, default):

+ TUNE (needs the register 6 readback, MUX_OUT connected to MISO): with the VCO selected manually, the VTUNE ADC of register 6 is read while the frequency is stepped up; the table holds, for every VCO, the range where VTUNE is 0.7-2.1 V (ADC 010 or 101). The scan stops as soon as VTUNE exceeds 2.1 V, so the VCO is never forced beyond its range. On the test board: 2 s, full coverage of 3000-6000 MHz, good ranges 28-179 MHz wide (median 115). In a 100 kHz step sweep VTUNE stayed within 0.7-2.1 V in 995 of 1001 steps, while with the autoselect it reached 0.5-0.7 V in 36 steps and the VCO alternated between three sub-bands.
+ LOCK (lock detect pin only): the lock range of every VCO. Forcing a VCO beyond the upper limit of its range can leave the PLL stuck (neither the autoselect nor rewriting the registers recover it): the calibration recovers it programming the registers with CE low, as at power up (25 ms); VCOs whose limits differ by more than 90 MHz from the neighbouring sub-bands of the same VCO core are measured again. About 35 s. The lower part of a lock range has VTUNE below 0.5 V and the output degrades within about 40 MHz from the upper limit, hence the 60 MHz default margin.

The control panel saves the table to `tools/max2870_vco_table.txt` and loads it into the board at every connection.

Notes from the test board: at the lowest output level (-4 dBm, APWR = 00) some VCO sub-bands were noisy, and in integer mode the PLL locked about 1 MHz off frequency with some VCOs; from -1 dBm up everything worked, so the sketch starts at level 2. The register 6 readback works with the SPI clock at 4 MHz. With only one output of a pair used, terminate the other one (e.g. RF_OUT_A- when RF_OUT_A+ is used) in 50 ohm, as the datasheet asks: left open, on the test board (with 5.6 nH chokes added in series with the 50 ohm pull-ups) it caused a 6-10 dB dip of the output power at 4.3-4.7 GHz.

[tools/max2870_gui.py](tools/max2870_gui.py) is a PC control panel for the demo sketch (Python 3 with tkinter and pyserial: `pip install pyserial`, then `python max2870_gui.py`): frequency entry (MHz by default, or with unit Hz/k/M/G) with step buttons, mouse wheel and slider, power level, reference/R/doubler, charge pump current, phase detector polarity, CE, sweep, lock indicator with periodic status refresh, and a console for raw commands.

### Other projects

The library and the demo are also used in [synth2870](https://github.com/rvisent/synth2870), a 2-18 GHz source with a variable gain amplifier and a YIG tuned multiplier driven by the MAX2870 (output power levelled from a measured table, automated power measurements).

## Library Use

The library is briefly documented below. 

An example program using the library is provided in [examples/my2870](examples/my2870); once the library is installed it appears in the Arduino IDE under File > Examples > MAX2870.

MAX2870(uint32_t SPIclock = 500000UL): constructor, SPIclock is the SPI clock in Hz (e.g. "MAX2870 vfo(1000000UL);" for 1 MHz).

init(SSpin, LockPinNumber, Lock_Pin_Used, CEpin, CE_Pin_Used): initialize the MAX2870 with SPI SS pin, lock pin and true/false for lock pin use and CE pin use - CE pin is typically LOW (disabled) on reset if used; depending on your board, this pin along with the RF Power Down pin may have a pullup or pulldown resistor fitted and certain boards have the RF Power Down pin (low active) on the header. Registers are written twice with a 25 ms pause, as recommended in the datasheet, with RF outputs disabled. Note: presently lock and CE pins are only configured by the library, they are actually used only in the demo file my2870.ino.

ReadR()/ReadInt()/ReadFraction()/ReadMod(): returns a uint16_t value for the currently programmed register

ReadOutDivider()/ReadOutDivider_PowerOf2()/ReadRDIV2()/ReadRefDoubler()/ReadPowerLevel()/ReadAuxPowerLevel(): returns a uint8_t value for the currently programmed register. ReadOutDivider() is automatically converted from a binary exponent to an actual division ratio and ReadOutDivider_PowerOf2() is the 2 power exponent used for the actual MAX2870 register.

ReadPFDfreq(): returns a uint32_t for the PFD frequency in Hz (rounded).

setrf(uint32_t f, uint16_t r, uint8_t ReferenceDivisionType): set the reference frequency and reference divider R and reference frequency division type (MAX2870_REF_(UNDIVIDED/HALF/DOUBLE)) - default is 10 MHz/1/undivided. Also sets the band select clock divider (band select clock 50 kHz as required by the datasheet, BS limited to 1023 for fPFD above 51.15 MHz) and the lock detect speed. Registers are written to the chip by the next setf(). Returns an error code.

setf(long long freq, uint8_t PowerLevel=1, uint8_t AuxPowerLevel=0, uint8_t AuxFrequencyDivider=MAX2870_AUX_FUNDAMENTAL): set the frequency (in Hz), power level/auxiliary power level (0:off; 1-4 in 3dB steps from -4 dBm) and mode for auxiliary frequency output (MAX2870_AUX_DIVIDED/FUNDAMENTAL). Returns an error code; on error no register is changed.

sweepStep(long long freq): as setf, but changes only the frequency. Slightly faster, useful for sweeps. Returns an error code as setf.

setfDirect(R_divider, INT_value, MOD_value, FRAC_value, RF_DIVIDER_value, FRACTIONAL_MODE): RF divider value is (1/2/4/8/16/32/64/128) and fractional mode is a true/false bool. These parameters will not be checked for invalid values. ReadCurrentFrequency() is not updated.

setPowerLevel/setAuxPowerLevel(PowerLevel): set the power level (0 to disable or 1-4) and write to the MAX2870 in one operation. Returns an error code.

WriteSweepValues(const uint32_t *regs): high speed straight MAX2870 register write, usable e.g. for fast frequency changes from stored register sets (*regs is uint32_t and size is as per MAX2870_RegsToWrite).

ReadSweepValues(uint32_t *regs): high speed read of the MAX2870 register copies (*regs is uint32_t and size is as per MAX2870_RegsToWrite).

ReadCurrentFrequency(): actual output frequency in Hz set by the last successful setf() or sweepStep(), returns a long long.

ReadFrequencyError(): requested minus actual frequency in Hz for the last successful setf() or sweepStep(), returns an int32_t.

setCPcurrent(float Current): set charge pump current in mA (0.32 to 5.12 mA in 0.32 mA steps, valid for RSET = 5.1 kOhm).

setPDpolarity(MAX2870_LOOP_TYPE_INVERTING/NONINVERTING): set phase detector polarity for your VCO loop filter.

writeRegister(n): write only register n (0-5) from MAX2870_R[]. Writing R0 starts the VCO autoselect when enabled.

setMuxOut(mode): set the MUX_OUT pin mode (MAX2870_MUX_TRISTATE, MAX2870_MUX_DIGITAL_LOCK, MAX2870_MUX_READBACK or any MUX[3:0] value) writing R5 and R2 only.

ReadRegister6()/ReadActiveVCO()/ReadTuneADC(): read back register 6 through MUX_OUT (requires MAX2870_MUX_READBACK and MUX_OUT connected to MISO): active VCO 0-63 and 3 bit VTUNE ADC (000/111 out of lock, 001/110 near the tuning limits). ReadRegister6Raw() returns the 40 bits sampled after the read command, for debug; MAX2870_R6_SKIP is the number of clocks before bit 31.

setVAS(enable, vco): enable the VCO autoselect, or disable it using the given VCO; ReadVAS()/ReadManualVCO() return the current setting.

setForceFrac(force): keep fractional mode also when FRAC = 0 (no charge pump/lock detect setting changes between steps).

checkReadback(): true if the register 6 readback works (two manual VCOs are selected and read back); leaves MUX_OUT in readback mode.

calibrateVCO(stepMHz=10, settleUs=1000, watchUs=2000, method=MAX2870_CAL_AUTO): build the VCO table with MAX2870_CAL_TUNE (VTUNE ADC, needs the readback), MAX2870_CAL_LOCK (lock detect pin given to init(), LD set as digital lock detect; uses the CE pin, if given to init(), to recover a stuck PLL) or MAX2870_CAL_AUTO (TUNE if checkReadback() succeeds). Returns the number of VCOs found, -1 if the method cannot be used; `recoveries` and `rechecks` report the stuck PLL recoveries and the VCOs measured again (LOCK). ReadVCOTable(vco, &min, &max)/setVCOTable(vco, min, max)/ReadVCOTableKind()/setVCOTableKind(kind)/VCOTableValid(): table access (MHz of fVCO), e.g. to save and restore it.

useVCOTable(use, marginMHz=-1): select the VCO from the table in setf() (most centered VCO) and sweepStep() (current VCO kept while at least marginMHz from its limits; -1 = default for the table kind: 60 MHz LOCK, 5 TUNE); frequencies not covered by the table use the autoselect. selectVCO(fvcoMHz, current) returns the VCO that would be chosen. testLock(vco, fvcoMHz, settleUs, watchUs) tests a single VCO.


Default settings which may need to be changed as required BEFORE execution of MAX2870 library functions (defaults listed):

Phase Detector Polarity (Register 2/Bit 6 = 1): Positive (passive or noninverting active loop filter)

SPI rate: 500 kHz, can be changed on the constructor. On Arduino Due the minimum is 84 MHz/255 = 329 kHz.

Error codes:

Common to all of the following subroutines:

MAX2870_ERROR_NONE


setf:

MAX2870_ERROR_POWER_LEVEL

MAX2870_ERROR_AUX_POWER_LEVEL

MAX2870_ERROR_AUX_FREQ_DIVIDER

and all the sweepStep error codes


sweepStep:

MAX2870_ERROR_RF_FREQUENCY

MAX2870_ERROR_ZERO_PFD_FREQUENCY

MAX2870_ERROR_N_RANGE

MAX2870_ERROR_N_RANGE_FRAC

MAX2870_ERROR_PFD_EXCEEDED_WITH_FRACTIONAL_MODE


setrf:

MAX2870_ERROR_DOUBLER_EXCEEDED

MAX2870_ERROR_R_RANGE

MAX2870_ERROR_REF_FREQUENCY

MAX2870_ERROR_REF_MULTIPLIER_TYPE

MAX2870_ERROR_PFD_LIMITS


setPowerLevel/setAuxPowerLevel:

MAX2870_ERROR_POWER_LEVEL


setPDpolarity:

MAX2870_ERROR_POLARITY_INVALID


## Installation
Download the ZIP from GitHub (Code > Download ZIP) and add it with Sketch > Include Library > Add .ZIP Library in the Arduino IDE, or copy (or clone) this repository into the `libraries` folder of your sketchbook as `MAX2870`. For development, a directory link keeps the IDE on the working copy without copying files, e.g. on Windows: `mklink /J "<sketchbook>\libraries\MAX2870" "<path of this repository>"`. Alternatively copy the two files from `src/` into your sketch folder.

## Version history

v1.0.0 First release

v1.0.1 Double buffering of RF frequency divider implemented by default

v1.1.0 Added current frequency read function

v1.1.1 Corrected issue with conversion in ReadCurrentFreq

v1.1.2 Add setPowerLevel function which can be used for frequency bursts

v1.1.3 Added direct entry of frequency parameters for precalculated frequencies of the highest possible precision

v1.1.4 Added configuration of charge pump current and phase detector polarity

v2.0 use of "long long" instead of "BigNumber", also doesn't use BitFieldManipulation and BeyondByte libraries, but internal implementations to increase execution speed. Uses Farey algorithm for fast fractional optimizer. Tested on Arduino Due.

v2.1 SPI clock passed to the constructor is now actually used (previous versions always ran at the SPI library default, 4 MHz on Due). FRAC/MOD computed with an integer-only best rational approximation (continued fractions): always optimal, at most a few tens of iterations, no floating point. setrf() sets the band select clock divider (fBS = 50 kHz as required by the datasheet) and the lock detect speed according to fPFD. Output frequency is computed exactly and rounded to 1 Hz. Register 6 readback, VCO autoselect on/off with manual VCO selection, VCO lookup table calibrated with the VTUNE ADC or the lock detect pin (sweeps without VCO autoselect, no unlock at every step), optional forced fractional mode. Demo sketch with a machine readable serial protocol and a Python control panel.

## References

+ Maxim MAX2870 datasheet
