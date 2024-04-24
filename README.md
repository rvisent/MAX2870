# MAX2870
Arduino Library for the MAX2870 Wideband Frequency Synthesizer chip

v1.0.0 First release

v1.0.1 Double buffering of RF frequency divider implemented by default

v1.1.0 Added current frequency read function

v1.1.1 Corrected issue with conversion in ReadCurrentFreq

v1.1.2 Add setPowerLevel function which can be used for frequency bursts

v1.1.3 Added direct entry of frequency parameters for precalculated frequencies of the highest possible precision

v1.1.4 Added configuration of charge pump current and phase detector polarity

v2.0 use of "long long" instead of "BigNumber", also doesn't use BitFieldManipulation and BeyondByte libraries, but internal implementations to increase execution speed. Uses Farey algorithm for fast fractional optimizer. Tested on Arduino Due.

## Introduction

This library supports the MAX2870 from Maxim on Arduinos. The chip is a wideband (23.475 MHz to 6 GHz) Phase-Locked Loop (PLL) and Voltage Controlled Oscillator (VCO), covering a very wide range frequency range
under digital control. Just add an external PLL loop filter, Reference frequency source and a power supply for a very useful 
frequency generator for applications as a Local Oscillator or Sweep Generator.  

The chip generates the frequency using a programmable Fractional-N and Integer-N Phase-Locked Loop (PLL) and Voltage Controlled Oscillator (VCO) with an external loop filter and frequency reference. The chip is controlled by 
a SPI interface, which is controlled by a microcontroller such as the Arduino.

The library provides an SPI control interface for the MAX2870, and also provides functions to calculate and set the frequency, which greatly simplifies the integration of this chip into a design. Input frequency and part of the calculations are performed using "long long" (64 bit) variables to allow 1 Hz resolution. This may cause issues on smaller Arduinos, code was tested on an Arduino Due. The library also exposes all of the PLL variables, such as FRAC, Mod and INT, so they examined as needed.  

Doesn't require any additional library.

A low phase noise stable oscillator is required for this module. Typically, an Ovenized Crystal Oscillator (OCXO) in the 10 MHz to 100 MHz range is used.  

## Features

+ Frequency Range: 23.475 MHz to 6 GHz
+ Output Level: -4 dBm to 5 dBm (in 3 dB steps) 
+ In-Band Phase Noise: -95 dBc/Hz (10 kHz from 4.227 Ghz carrier)
+ PLL Modes: Fraction-N and Integer-N (set automatically)
+ Step Frequency: 1 kHz to 100 MHz  
+ Signal On/Off control
+ All MAX2870_R[] registers can be accessed and manipulated

## Library Use

The library is briefly documented below. 

An example program using the library is provided in the testcode directory [my2870](testcode/my2870). Compile from Arduino IDE by putting on the same directory the two files from the src folder.

init(SSpin, LockPinNumber, Lock_Pin_Used, CEpin, CE_Pin_Used): initialize the MAX2870 with SPI SS pin, lock pin and true/false for lock pin use and CE pin use - CE pin is typically LOW (disabled) on reset if used; depending on your board, this pin along with the RF Power Down pin may have a pullup or pulldown resistor fitted and certain boards have the RF Power Down pin (low active) on the header. Note: presently lock and CE pins are not used in the library, but only on the demo file my2870.ino.

ReadR()/ReadInt()/ReadFraction()/ReadMod(): returns a uint16_t value for the currently programmed register

ReadOutDivider()/ReadOutDivider_PowerOf2()/ReadRDIV2()/ReadRefDoubler()/ReadPowerLevel()/ReadAuxPowerLevel(): returns a uint8_t value for the currently programmed register. ReadOutDivider() is automatically converted from a binary exponent to an actual division ratio and ReadOutDivider_PowerOf2() is the 2 power exponent used for the actual MAX2870 register.

ReadPFDfreq(): returns a uint32_t for the PFD value.

setf(long long freq, uint8_t PowerLevel=1, uint8_t AuxPowerLevel=0, uint8_t AuxFrequencyDivider=1): set the frequency (in Hz) power level/auxiliary power level (0:off; 1-4 in 3dBm steps from -5dBm), mode for auxiliary frequency output (MAX2870_AUX_DIVIDED/FUNDAMENTAL). Returns an error or warning code.

setrf(uint32_t f, uint16_t r, uint8_t ReferenceDivisionType): set the reference frequency and reference divider R and reference frequency division type (MAX2870_REF_(UNDIVIDED/HALF/DOUBLE)) - default is 10 MHz/1/undivided - returns an error code.

sweepStep(long long freq): as setf, but allows only changing the frequency. Slightly faster, useful for sweeps. Returns error or warning as setf.

setfDirect(R_divider, INT_value, MOD_value, FRAC_value, RF_DIVIDER_value, FRACTIONAL_MODE): RF divider value is (1/2/4/8/16/32/64) and fractional mode is a true/false bool. These paramaters will not be checked for invalid values.

setPowerLevel/setAuxPowerLevel(PowerLevel): set the power level (0 to disable or 1-4) and write to the MAX2870 in one operation. Returns an error code.

WriteSweepRegs(const uint32_t *regs): high speed straight MAX2870 register write, usable e.g. for fast frequency changes from stored register sets (*regs is uint32_t and size is as per MAX2870_RegsToWrite.

ReadSweepRegs(uint32_t *regs): high speed read of MAX2870 registers (*regs is uint32_t and size is as per MAX2870_RegsToWrite.

ReadCurrentFrequency(): calculation of currently programmed frequency, returns a long long.

setCPcurrent(float Current): set charge pump current in mA.

setPDpolarity(INVERTING/NONINVERTING): set phase detector polarity for your VCO loop filter.


Default settings which may need to be changed as required BEFORE execution of MAX2870 library functions (defaults listed):

Phase Detector Polarity (Register 2/Bit 6 = 1): Positive (passive or noninverting active loop filter)
SPI rate: default 500 kHz on MAX2870() constructor, can be changed by specifying on constructor (e.g. "MAX2870 vfo(1000000UL);" for 1 MHz).

Error codes:

Common to all of the following subroutines:

MAX2870_ERROR_NONE


setf:

MAX2870_ERROR_RF_FREQUENCY

MAX2870_ERROR_POWER_LEVEL

MAX2870_ERROR_AUX_POWER_LEVEL

MAX2870_ERROR_AUX_FREQ_DIVIDER

MAX2870_ERROR_ZERO_PFD_FREQUENCY

MAX2870_ERROR_MOD_RANGE

MAX2870_ERROR_FRAC_RANGE

MAX2870_ERROR_N_RANGE

MAX2870_ERROR_PFD_EXCEEDED_WITH_FRACTIONAL_MODE


sweepStep:

MAX2870_ERROR_RF_FREQUENCY

MAX2870_ERROR_AUX_FREQ_DIVIDER

MAX2870_ERROR_ZERO_PFD_FREQUENCY

MAX2870_ERROR_MOD_RANGE

MAX2870_ERROR_FRAC_RANGE

MAX2870_ERROR_N_RANGE

MAX2870_ERROR_PFD_EXCEEDED_WITH_FRACTIONAL_MODE


setrf:

MAX2870_ERROR_DOUBLER_EXCEEDED

MAX2870_ERROR_R_RANGE

MAX2870_ERROR_REF_FREQUENCY

MAX2870_ERROR_REF_MULTIPLIER_TYPE


setrf:

MAX2870_ERROR_PFD_LIMITS


setPDpolarity:

MAX2870_ERROR_POLARITY_INVALID


Warning codes, setf and sweepStep:

MAX2870_WARNING_FREQUENCY_ERROR

## Installation
Copy the `src/` directory to your Arduino sketchbook directory  (named the directory `my2870`). You can also install the MAX2870 files separatly as a library.

## References

+ Maxim MAX2870 datasheet
