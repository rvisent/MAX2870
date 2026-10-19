/*!
   @file MAX2870.h

   This is part of the Arduino Library for the MAX2870 PLL wideband frequency synthesier

*/

#ifndef MAX2870_H
#define MAX2870_H
#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

#define MAX2870_PFD_MAX   105000000UL      ///< Maximum Frequency for Phase Detector (Integer-N)
#define MAX2870_PFD_MAX_FRAC   50000000UL  ///< Maximum Frequency for Phase Detector (Fractional-N)
#define MAX2870_PFD_MIN   125000UL        ///< Minimum Frequency for Phase Detector
#define MAX2870_REFIN_MAX   200000000UL   ///< Maximum Reference Frequency
#define MAX2870_REFIN_MIN   10000000UL   ///< Minimum Reference Frequency
#define MAX2870_REF_FREQ_DEFAULT 10000000UL  ///< Default Reference Frequency
#define MAX2870_BS_FREQ_MAX 50000UL         ///< Maximum Band Select Clock Frequency

#define MAX2870_AUX_DIVIDED 0
#define MAX2870_AUX_FUNDAMENTAL 1
#define MAX2870_REF_UNDIVIDED 0
#define MAX2870_REF_HALF 1
#define MAX2870_REF_DOUBLE 2
#define MAX2870_LOOP_TYPE_INVERTING 0
#define MAX2870_LOOP_TYPE_NONINVERTING 1

// MUX_OUT pin modes (MUX[3:0], register 2 bits 28:26 + register 5 bit 18)
#define MAX2870_MUX_TRISTATE 0
#define MAX2870_MUX_DIGITAL_LOCK 6
#define MAX2870_MUX_READBACK 12  // MUX_OUT is the serial output of register 6

// register 6 readback: number of clocks after LE rising edge before bit 31 is sampled
#define MAX2870_R6_SKIP 2

// VCO calibration methods and table kinds
#define MAX2870_CAL_AUTO 0  // TUNE if the register 6 readback works, else LOCK
#define MAX2870_CAL_LOCK 1  // table = lock range of each VCO (lock detect pin)
#define MAX2870_CAL_TUNE 2  // table = range with VTUNE 0.7-2.1 V (register 6 ADC = 010 or 101)

// VCO table: default distance from the VCO range limits kept in sweeps. Lock range table: on the
// test board the output degrades within about 40 MHz from the upper limit, while the most centered
// VCO is always at least 70 MHz from both limits. VTUNE table: the range is already the good one, 5 MHz keep VTUNE below 2.1 V
#define MAX2870_VCO_MARGIN_MHZ 60
#define MAX2870_VCO_MARGIN_TUNE_MHZ 5
// VCO calibration range (fVCO, MHz): wider than the 3000-6000 MHz specification to find the edges
#define MAX2870_VCO_CAL_MIN_MHZ 2800
#define MAX2870_VCO_CAL_MAX_MHZ 6200
// minimum lock range to accept a VCO (rejects isolated false lock indications)
#define MAX2870_VCO_MIN_RANGE_MHZ 30
// maximum plausible lock range of a VCO (wider: lock detect stuck, the VCO is tested again)
#define MAX2870_VCO_MAX_RANGE_MHZ 800
// consistency check: VCO limits farther than this from the neighbouring sub-bands are measured
// again (test board: real irregularities up to about 70 MHz, wrong measurements 115-300 MHz)
#define MAX2870_VCO_CONSISTENCY_MHZ 90
#define MAX2870_VCO_MAX_RECHECKS 10
// stuck PLL recovery: pause between the register writes with CE low (power up sequence)
#define MAX2870_RECOVER_DELAY_MS 25

// common to all of the following subroutines
#define MAX2870_ERROR_NONE 0

// SetStepFreq
#define MAX2870_ERROR_STEP_FREQUENCY_EXCEEDS_PFD 1

// setf
#define MAX2870_ERROR_RF_FREQUENCY 2
#define MAX2870_ERROR_POWER_LEVEL 3
#define MAX2870_ERROR_AUX_POWER_LEVEL 4
#define MAX2870_ERROR_AUX_FREQ_DIVIDER 5
#define MAX2870_ERROR_ZERO_PFD_FREQUENCY 6
#define MAX2870_ERROR_MOD_RANGE 7
#define MAX2870_ERROR_FRAC_RANGE 8
#define MAX2870_ERROR_N_RANGE 9
#define MAX2870_ERROR_N_RANGE_FRAC 10
#define MAX2870_ERROR_RF_FREQUENCY_AND_STEP_FREQUENCY_HAS_REMAINDER 11
#define MAX2870_ERROR_PFD_EXCEEDED_WITH_FRACTIONAL_MODE 12
#define MAX2870_ERROR_PRECISION_FREQUENCY_CALCULATION_TIMEOUT 13
//#define MAX2870_WARNING_FREQUENCY_ERROR 14

// setrf
#define MAX2870_ERROR_DOUBLER_EXCEEDED 15
#define MAX2870_ERROR_R_RANGE 16
#define MAX2870_ERROR_REF_FREQUENCY 17
#define MAX2870_ERROR_REF_MULTIPLIER_TYPE 18

// setf and setrf
#define MAX2870_ERROR_PFD_AND_STEP_FREQUENCY_HAS_REMAINDER 19
#define MAX2870_ERROR_PFD_LIMITS 20

// setPDpolarity
#define MAX2870_ERROR_POLARITY_INVALID 21

#define MAX2870_RegsToWrite 6UL // for high speed sweep

// ReadCurrentFrequency
#define MAX2870_DIGITS 10
#define MAX2870_DECIMAL_PLACES 6
#define MAX2870_ReadCurrentFrequency_ArraySize (MAX2870_DIGITS + MAX2870_DECIMAL_PLACES + 2) // including decimal point and null terminator

/*!
   @brief MAX2870 chip device driver

   This class provides the overall interface for MAX2870 chip. It is used
   to define the SPI connection, initalize the chip on power up, disable/enable
   frequency generation, and set the frequency and reference frequency.

   The PLL values and register values can also be set directly with this class,
   and current settings for the chip and PLL can be read.

   As a simple frequency generator, once the target frequency and desired channel step
   value is set, the library will perform the required calculations to
   set the PLL and other values, and determine the mode (Frac-N or Int-N)
   for the PLL loop. This greatly simplifies the use of the MAX2870 chip.

   The MAX2870 datasheet should be consulted to understand the correct
   register settings. While a number of checks are provided in the library,
   not all values are checked for allowed settings, so YMMV.

*/
class MAX2870
{
  public:
    /*!
       Constructor
       creates an object and sets the SPI parameters.
       see the Arduino SPI library for the parameter values.
       @param pin the SPI Slave Select Pin to use
       @param mode the SPI Mode (see SPI mode define values)
       @param speed the SPI Serial Speed (see SPI speed values)
       @param order the SPI bit order (see SPI bit order values)
    */
    uint8_t MAX2870_PIN_SS = 10;   ///< Ard Pin for SPI Slave Select

    MAX2870(uint32_t SPIclock = 500000UL); // SPI clock in Hz
    void WriteRegs();

    uint16_t ReadR();
    uint16_t ReadInt();
    uint16_t ReadFraction();
    uint16_t ReadMod();
    uint8_t ReadOutDivider();
    uint8_t ReadOutDivider_PowerOf2();
    uint8_t ReadRDIV2();
    uint8_t ReadRefDoubler();
    uint8_t ReadPowerLevel();
    uint8_t ReadAuxPowerLevel();
    uint32_t ReadPFDfreq();
    int32_t ReadFrequencyError();
    
    void WriteBF_dword(uint8_t offset, uint8_t nBits, uint32_t *pdw, uint16_t insData);

    void init(uint8_t SSpin, uint8_t LockPinNumber, bool Lock_Pin_Used, uint8_t CEpin, bool CE_Pin_Used);
    int setf(long long freq, uint8_t PowerLevel=1, uint8_t AuxPowerLevel=0, uint8_t AuxFrequencyDivider=1); // set freq and power levels and output mode with option for precision frequency setting with tolerance in Hz
    int sweepStep(long long freq);
    void setfDirect(uint16_t R_divider, uint16_t INT_value,uint16_t MOD_value,uint16_t FRAC_value, uint8_t RF_DIVIDER_value, bool FRACTIONAL_MODE);
    int setrf(uint32_t f, uint16_t r, uint8_t ReferenceDivisionType) ; // set reference freq and reference divider (default is 10 MHz with divide by 1)
    int setPowerLevel(uint8_t PowerLevel);
    int setAuxPowerLevel(uint8_t PowerLevel);

    void WriteSweepValues(const uint32_t *regs);
    void ReadSweepValues(uint32_t *regs);
    long long ReadCurrentFrequency(void);
    int setCPcurrent(float Current);
    int setPDpolarity(uint8_t PDpolarity);

    // single register write (R0 write triggers VCO autoselect if enabled)
    void writeRegister(uint8_t n);
    // MUX_OUT pin mode, MAX2870_MUX_READBACK required by ReadRegister6()
    void setMuxOut(uint8_t mode);
    // register 6 readback (needs MUX_OUT connected to MISO)
    uint32_t ReadRegister6();
    uint64_t ReadRegister6Raw(); // 40 bits sampled after the read command, for debug
    uint8_t ReadActiveVCO();     // VCO and sub-band selected, 0-63
    uint8_t ReadTuneADC();       // 3 bit VTUNE ADC: 000/111 out of lock, 001/110 near limits, 010/101 ok
    // VCO autoselect: enable, or disable keeping the given VCO (0-63)
    void setVAS(bool enable, uint8_t vco = 0);
    bool ReadVAS();
    uint8_t ReadManualVCO();
    // keep fractional mode also when FRAC = 0 (no loop setting changes in sweeps)
    void setForceFrac(bool force);

    // true if the register 6 readback works (MUX_OUT connected to MISO): two manual VCOs are
    // selected and read back. MUX_OUT is left in readback mode.
    bool checkReadback();

    // VCO lookup table (VCO manual selection, see datasheet Applications Information).
    // calibrateVCO() with method:
    //  MAX2870_CAL_TUNE: for every VCO, the range where VTUNE is 0.7-2.1 V, read with the register 6
    //    ADC (needs the readback). The VCO is never forced beyond its range: fast and clean.
    //  MAX2870_CAL_LOCK: the lock range of every VCO testing the lock detect pin (LD = digital lock
    //    detect, pin given to init()).
    //  MAX2870_CAL_AUTO: TUNE if checkReadback() succeeds, else LOCK.
    // RF outputs are muted during the calibration. Returns the number of VCOs found, -1 if the
    // method cannot be used (no lock pin / no readback).
    int calibrateVCO(uint16_t stepMHz = 10, uint16_t settleUs = 1000, uint16_t watchUs = 2000,
                     uint8_t method = MAX2870_CAL_AUTO);
    bool ReadVCOTable(uint8_t vco, uint16_t *minMHz, uint16_t *maxMHz); // false if VCO not found
    void setVCOTable(uint8_t vco, uint16_t minMHz, uint16_t maxMHz);     // e.g. restore a saved table
    uint8_t ReadVCOTableKind();                                         // MAX2870_CAL_LOCK or _TUNE
    void setVCOTableKind(uint8_t kind);
    bool VCOTableValid();
    // use the table in setf() (most centered VCO) and sweepStep() (keep the VCO while at least
    // marginMHz from its limits): no VCO autoselect, no unlock at every frequency change.
    // marginMHz < 0: default for the table kind
    void useVCOTable(bool use, int16_t marginMHz = -1);
    bool ReadUseVCOTable();
    int8_t selectVCO(uint16_t fvcoMHz, int8_t current); // -1 if no VCO covers fvcoMHz
    uint16_t recoveries = 0;          // stuck PLL recoveries in the last calibration
    uint8_t rechecks = 0;             // VCOs measured again by the consistency check
    // program fVCO with a manual VCO, true if the lock detect pin stays high for watchUs after settleUs
    // (prepare: lock first with the autoselect at fvcoMHz, see the .cpp)
    bool testLock(uint8_t vco, uint16_t fvcoMHz, uint16_t settleUs, uint16_t watchUs, bool prepare = true);

    SPISettings MAX2870_SPI;

    int32_t MAX2870_FrequencyError = 0;
    // power on defaults
    uint32_t MAX2870_reffreq = MAX2870_REF_FREQ_DEFAULT;
    // R[4] modified to start with RF outputs disabled (original 0x6180B23C)
    uint32_t MAX2870_R[6] {0x007D0000, 0x2000FFF9, 0x18006E42, 0x0000000B, /*0x6180B23C*/0x6180B204, 0x00400005};
    long long MAX2870_currentF;

  private:
    bool forceFrac = false;
    uint8_t ldPin = 255;              // lock detect pin, 255 = not used
    uint8_t cePin = 255;              // chip enable pin, 255 = not used
    void recoverPLL();
    uint16_t vcoMin[64], vcoMax[64];  // VCO lock ranges in MHz, 0 = not found
    bool vcoTableValid = false;
    uint8_t vcoTableKind = MAX2870_CAL_LOCK;
    bool vcoTableUse = false;
    uint8_t vcoMargin = MAX2870_VCO_MARGIN_MHZ;
    void applyVCO(long long freq, uint8_t divSel, bool keepCurrent);
    void programVCOfreq(uint16_t fvcoMHz);
    uint16_t calStepMHz, calSettleUs, calWatchUs;
    bool calOk;
    bool calTest(uint8_t vco, uint16_t fvcoMHz);
    bool calTestUp(uint8_t vco, uint16_t fvcoMHz);
    bool calibrateOne(uint8_t v, uint16_t fStart, uint16_t *pLo, uint16_t *pHi);
    uint16_t vcoDeviation(uint8_t v);
    int calibrateLock();
    int calibrateTune();
    uint8_t tuneTest(uint8_t v, uint16_t fvcoMHz, bool *locked);
    void writeReg(uint32_t value);
    void getRefRatio(uint32_t *pRefNum, uint32_t *pRefDen);
    void bestRational(uint32_t num, uint32_t den, uint16_t maxDen, uint16_t *pNum, uint16_t *pDen);
    int calcPLL(long long freq, uint16_t *pN, uint16_t *pF, uint16_t *pM, uint8_t *pDivSel, long long *pActual);
    void applyPLL(uint16_t N, uint16_t F, uint16_t M, uint8_t divSel);
    void updatePFDsettings();
};

#endif
