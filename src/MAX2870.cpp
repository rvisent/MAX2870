/*!

   @file MAX2870.cpp

   @mainpage MAX2870 Arduino library driver for Wideband Frequency Synthesizer

   @section intro_sec Introduction

   The MAX2870 chip is a wideband freqency synthesizer integrated circuit that can generate frequencies
   from 23.475 MHz to 6 GHz. It incorporates a PLL (Fraction-N and Integer-N modes) and VCO, along with
   prescalers, dividers and multipiers.  The users add a PLL loop filter and reference frequency to
   create a frequency generator with a very wide range, that is tuneable in settable frequency steps.

   The MAX2870 chip provides an SPI interface for setting the device registers that control the
   frequency and output levels, along with several IO pins for gathering chip status and
   enabling/disabling output and power modes.

   The MAX2870 library provides an Arduino API for accessing the features of the ADF chip.

   The basic PLL equations for the MAX2870 are:

   \f$ RF_{out} = f_{PFD} \times (INT +(\frac{FRAC}{MOD})) \f$

   where:

   \f$ f_{PFD} = REF_{IN} \times \left[ \frac{(1 + D)}{( R \times (1 + T))} \right]  \f$

   \f$ D = \textrm{RD2refdouble, ref doubler flag}\f$

   \f$ R = \textrm{RCounter, ref divider}\f$

   \f$ T = \textrm{RD1Rdiv2, ref divide by 2 flag}\f$




   @section dependencies Dependencies

   @section author Author

   Bryce Cherry
   2024apr07 modified by Roberto Visentin to use "long long" and Farey algorithm

*/

#include "MAX2870.h"

MAX2870::MAX2870(uint32_t SPIclock)
{
  SPISettings MAX2870_SPI(SPIclock, MSBFIRST, SPI_MODE0); // RV changed from 10 MHz to 500 kHz due to filtered
  MAX2870_currentF = 0;
}

void MAX2870::WriteRegs()
{
  SPI.beginTransaction(MAX2870_SPI);
  SPI.setClockDivider(255); // 329 kHz not working with SPISettings... (?)
  for (int i = 5 ; i >= 0 ; i--) { // sequence according to the MAX2870 datasheet
    uint8_t *pReg = (uint8_t *)(MAX2870_R+i);
    digitalWrite(MAX2870_PIN_SS, LOW);
    delayMicroseconds(1);
    //BeyondByte.writeDword(0, MAX2870_R[i], 4, BeyondByte_SPI, MSBFIRST);
    SPI.transfer(pReg[3]);  // transfer one DWORD from MSB
    SPI.transfer(pReg[2]);
    SPI.transfer(pReg[1]);
    SPI.transfer(pReg[0]);
    delayMicroseconds(1);
    digitalWrite(MAX2870_PIN_SS, HIGH);
    delayMicroseconds(1);
  }
  SPI.endTransaction();
}

void MAX2870::WriteSweepValues(const uint32_t *regs) {
  for (int i = 0; i < MAX2870_RegsToWrite; i++) {
    MAX2870_R[i] = regs[i];
  }
  WriteRegs();
}

void MAX2870::ReadSweepValues(uint32_t *regs) {
  for (int i = 0; i < MAX2870_RegsToWrite; i++) {
    regs[i] = MAX2870_R[i];
  }
}

uint16_t MAX2870::ReadR() {
  return uint16_t(MAX2870_R[2] >> 14) & 1023;
}

uint16_t MAX2870::ReadInt() {
  return uint16_t(MAX2870_R[0] >> 15);
}

uint16_t MAX2870::ReadFraction() {
  return uint16_t(MAX2870_R[0] >> 3) & 4095;
}

uint16_t MAX2870::ReadMod() {
  return uint16_t(MAX2870_R[1] >> 3) & 4095;
}

uint8_t MAX2870::ReadOutDivider() {
  return 1 << ReadOutDivider_PowerOf2();
}

uint8_t MAX2870::ReadOutDivider_PowerOf2() {
  return uint8_t(MAX2870_R[4] >> 20) & 7;
}

uint8_t MAX2870::ReadRDIV2() {
  return (MAX2870_R[2] & 0x1000000L) ? 1:0;
}

uint8_t MAX2870::ReadRefDoubler() {
  return (MAX2870_R[2] & 0x2000000L) ? 1:0;
}

uint32_t MAX2870::ReadPFDfreq() {
  uint32_t value = MAX2870_reffreq;
  uint16_t temp = ReadR();
  if (temp == 0) { // avoid division by zero
    return 0;
  }
  value /= temp;
  if (ReadRDIV2() != 0) {
    value /= 2;
  }
  if (ReadRefDoubler() != 0) {
    value *= 2;
  }
  return value;
}

int32_t MAX2870::ReadFrequencyError() {
  return MAX2870_FrequencyError;
}

long long MAX2870::ReadCurrentFrequency(void)
{
  return MAX2870_currentF;

/*
  uint16_t CurrentR = ReadR();
  // external reference optionally *2 or /2
  uint32_t intRef = MAX2870_reffreq*(1+ReadRefDoubler())/(1+ReadRDIV2());
  // INT part of fvco
  long long fvco = (long long)intRef*ReadInt()/CurrentR;
  // FRAC part of fvco
  fvco += (long long)intRef*ReadFraction()/((uint32_t)CurrentR*ReadMod());
  return fvco/ReadOutDivider();
*/
}

uint8_t MAX2870::ReadPowerLevel()
{
  // output off?
  if (!(MAX2870_R[4] & 0b100000))
    return 0;
  return (uint8_t(MAX2870_R[4] >> 3) & 3)+1;
}

uint8_t MAX2870::ReadAuxPowerLevel()
{
  // output off?
  if (!(MAX2870_R[4] & 0b100000000))
    return 0;
  return (uint8_t(MAX2870_R[4] >> 6) & 3)+1;
}

void MAX2870::WriteBF_dword(uint8_t offset, uint8_t nBits, uint32_t *pdw, uint16_t insData)
{
  uint32_t mask = ((1L<<nBits)-1)<<offset;
  *pdw = *pdw & ~mask | (insData<<offset) & mask;
}

void MAX2870::farey(float x, uint16_t max_denominator, uint16_t *pNum, uint16_t *pDen)
{
  uint16_t lower_num = 0, lower_den = 1;
  uint16_t upper_num = 1, upper_den = 1;
  uint16_t result_num = 0, result_den = 1;
  float value;

  while (1)
  {
    result_num = lower_num + upper_num;
    result_den = lower_den + upper_den;
    value = (float)result_num / result_den;

    if (result_den > max_denominator)
    {
      // If the denominator exceeds max_denominator, choose among previous best results
      float lower_value = (float)lower_num / lower_den;
      float upper_value = (float)upper_num / upper_den;
      if (x - lower_value < upper_value - x)
      {
        result_num = lower_num;
        result_den = lower_den;
      }
      else
      {
        result_num = upper_num;
        result_den = upper_den;
      }
      break;
    }
    else if (value == x)
    {
        // If the value matches exactly, break
        break;
    }
    else if (value < x)
    {
        lower_num = result_num;
        lower_den = result_den;
    }
      else
    {
        upper_num = result_num;
        upper_den = result_den;
    }
  }

  // copy result to external vars
  *pNum = result_num;
  *pDen = result_den;
}

void MAX2870::init(uint8_t SSpin, uint8_t LockPinNumber, bool Lock_Pin_Used, uint8_t CEpinNumber, bool CE_Pin_Used)
{
  MAX2870_PIN_SS = SSpin;
  pinMode(MAX2870_PIN_SS, OUTPUT) ;
  digitalWrite(MAX2870_PIN_SS, HIGH) ;
  if (CE_Pin_Used == true) {
    pinMode(CEpinNumber, OUTPUT) ;
  }
  if (Lock_Pin_Used == true) {
    pinMode(LockPinNumber, INPUT_PULLUP) ;
  }
  SPI.begin();

  // initialize chip as recommended in the datasheet
  // RF outputs are disabled on default values
  SPI.beginTransaction(MAX2870_SPI);
  SPI.setClockDivider(255); // 329 kHz not working with SPISettings... (?)
  for (int j = 0 ; j < 2 ; j++) {
    for (int i=5; i>=0; i--)
    {
      uint8_t *pReg = (uint8_t *)(MAX2870_R+i);
      digitalWrite(MAX2870_PIN_SS, LOW);
      delayMicroseconds(1);
      SPI.transfer(pReg[3]);
      SPI.transfer(pReg[2]);
      SPI.transfer(pReg[1]);
      SPI.transfer(pReg[0]);
      delayMicroseconds(1);
      digitalWrite(MAX2870_PIN_SS, HIGH);
      delayMicroseconds(1);
    }
    delay(25);  // at least 20 ms delay
  }
  SPI.endTransaction();
}

int  MAX2870::setf(long long freq, uint8_t PowerLevel, uint8_t AuxPowerLevel, uint8_t AuxFrequencyDivider) {
  //  calculate settings from freq
  if (PowerLevel < 0 || PowerLevel > 4) return MAX2870_ERROR_POWER_LEVEL;
  if (AuxPowerLevel < 0 || AuxPowerLevel > 4) return MAX2870_ERROR_AUX_POWER_LEVEL;
  if (AuxFrequencyDivider != MAX2870_AUX_DIVIDED && AuxFrequencyDivider != MAX2870_AUX_FUNDAMENTAL) return MAX2870_ERROR_AUX_FREQ_DIVIDER;
  if (ReadPFDfreq() == 0) return MAX2870_ERROR_ZERO_PFD_FREQUENCY;

  uint32_t ReferenceFrequency = MAX2870_reffreq;
  ReferenceFrequency /= ReadR();

  if (freq > 6000000000LL || freq < 23437500L)
    return MAX2870_ERROR_RF_FREQUENCY;

  uint8_t MAX2870_outdiv = 1 ;
  uint8_t MAX2870_RfDivSel = 0 ;
  uint32_t MAX2870_N_Int;
  uint16_t MAX2870_Mod = 2;
  uint16_t MAX2870_Frac = 0;

  uint16_t CurrentR = ReadR();
  uint8_t RDIV2 = ReadRDIV2();
  uint8_t RefDoubler = ReadRefDoubler();
  uint32_t intRef = MAX2870_reffreq*(1+RefDoubler)/(1+RDIV2);
  uint32_t PFDfreq = intRef/CurrentR; // not used for coeff. computation due to truncation
  
  // select the output divider
  MAX2870_outdiv = (uint8_t)(6000000000LL / (freq+1)); // +1, so e.g. 3000 MHz gets divided by 1
  if (MAX2870_outdiv >= 128) { MAX2870_outdiv = 128; MAX2870_RfDivSel=7; }
  else if (MAX2870_outdiv >= 64) { MAX2870_outdiv = 64; MAX2870_RfDivSel=6; }
  else if (MAX2870_outdiv >= 32) { MAX2870_outdiv = 32; MAX2870_RfDivSel=5; }
  else if (MAX2870_outdiv >= 16) { MAX2870_outdiv = 16; MAX2870_RfDivSel=4; }
  else if (MAX2870_outdiv >= 8) { MAX2870_outdiv = 8; MAX2870_RfDivSel=3; }
  else if (MAX2870_outdiv >= 4) { MAX2870_outdiv = 4; MAX2870_RfDivSel=2; }
  else if (MAX2870_outdiv >= 2) { MAX2870_outdiv = 2; MAX2870_RfDivSel=1; }
  else { MAX2870_outdiv = 1; MAX2870_RfDivSel=0; }

  long long fvcoTarget = freq*MAX2870_outdiv;

  // compute N_int
  MAX2870_N_Int = (uint16_t)(fvcoTarget*CurrentR/intRef);
  // compute actual frequency in int mode
  long long fvco = (long long)intRef*MAX2870_N_Int/CurrentR;
  // compute residual F/M (0..1)
  float res = float(fvcoTarget - fvco)*CurrentR/intRef;
  // put in best fraction form
  farey(res, 4095, &MAX2870_Frac, &MAX2870_Mod);
  // checks
  if (MAX2870_Mod == 1)
  {
    MAX2870_Mod = 2;  // minimum
    if (MAX2870_Frac == 1)
    {
      // best approx was 1/1, so we revert to 0/2 and increment N
      MAX2870_Frac = 0;
      MAX2870_N_Int++;
      fvco = (long long)intRef*MAX2870_N_Int/CurrentR;
    }
  }
  if (MAX2870_Frac >= MAX2870_Mod)
    MAX2870_Frac = MAX2870_Mod-1; // maximum
  
  if (MAX2870_Mod > 4095) {
    return MAX2870_ERROR_MOD_RANGE;
  }

  // compute actual frequency
  fvco += (long long)intRef*MAX2870_Frac/((uint32_t)CurrentR*MAX2870_Mod);
  MAX2870_currentF = fvco/MAX2870_outdiv;
  MAX2870_FrequencyError = (int32_t)(freq - MAX2870_currentF);

  if (MAX2870_Frac == 0 && (MAX2870_N_Int < 16  || MAX2870_N_Int > 65535)) {
    return MAX2870_ERROR_N_RANGE;
  }

  if (MAX2870_Frac != 0 && (MAX2870_N_Int < 19  || MAX2870_N_Int > 4091)) {
    return MAX2870_ERROR_N_RANGE_FRAC;
  }

  if (MAX2870_Frac != 0 && PFDfreq > MAX2870_PFD_MAX_FRAC) {
    return MAX2870_ERROR_PFD_EXCEEDED_WITH_FRACTIONAL_MODE;
  }

  MAX2870_R[0] = uint32_t(MAX2870_Frac) << 3;
  MAX2870_R[0] |= uint32_t(MAX2870_N_Int) << 15;

  if (MAX2870_Frac == 0) {
    MAX2870_R[0] |= 0x80000000L; // integer-n mode
    WriteBF_dword(29, 2, MAX2870_R+1, 0); // Charge Pump Linearity disable for INT mode
    WriteBF_dword(31, 1, MAX2870_R+1, 1); // CP clamp enable (improved INT phase noise)
    WriteBF_dword(8, 1, MAX2870_R+2, 1); // Lock Detect Function, int-n mode
    WriteBF_dword(24, 1, MAX2870_R+5, 0); // F01 not used in INT mode
  }
  else {
    WriteBF_dword(29, 2, MAX2870_R+1, 1); // Charge Pump Linearity enable for FRAC mode
    WriteBF_dword(31, 1, MAX2870_R+1, 0); // CP clamp disable in FRAC mode
    WriteBF_dword(8, 1, MAX2870_R+2, 0); // Lock Detect Function, frac-n mode
    WriteBF_dword(24, 1, MAX2870_R+5, 1); // F01 auto switch to INT if F==0
  }
  if (PFDfreq > 32000000L)
    WriteBF_dword(31, 1, MAX2870_R+2, 1); // lock detect speed
  else
    WriteBF_dword(31, 1, MAX2870_R+2, 0); // lock detect speed
  // (0x01, 15, 12, 1) phase
  WriteBF_dword(3, 12, MAX2870_R+1, MAX2870_Mod);
  // (0x02, 3,1,0) counter reset
  // (0x02, 4,1,0) cp3 state
  // (0x02, 5,1,0) power down
  if (PFDfreq > 32000000UL) { // lock detect speed adjustment
    WriteBF_dword(31, 1, MAX2870_R+2, 1); // Lock Detect Speed
  }
  else  {
    WriteBF_dword(31, 1, MAX2870_R+2, 0); // Lock Detect Speed
  }
  // (0x02, 13,1,0) dbl buf
  // (0x02, 26,3,0) //  muxout, not used
  // (0x02, 29,2,0) low noise and spurs mode
  // (0x03, 15,2,1) clk div mode
  // (0x03, 17,1,0) reserved
  // (0x03, 18,6,0) reserved
  // (0x03, 24,1,0) VAS response to temperature drift
  // (0x03, 25,1,0) VAS state machine
  // (0x03, 26,6,0) VCO and VCO sub-band manual selection
  if (PowerLevel == 0) {
    WriteBF_dword(5, 1, MAX2870_R+4, 0);  // output disabled
  }
  else {
    PowerLevel--;
    WriteBF_dword(5, 1, MAX2870_R+4, 1);  // output enabled
    WriteBF_dword(3, 2, MAX2870_R+4, PowerLevel);
  }
  if (AuxPowerLevel == 0) {
    WriteBF_dword(8, 1, MAX2870_R+4, 0);
  }
  else {
    AuxPowerLevel--;
    WriteBF_dword(6, 2, MAX2870_R+4, AuxPowerLevel);
    WriteBF_dword(8, 1, MAX2870_R+4, 1);
    WriteBF_dword(9, 1, MAX2870_R+04, AuxFrequencyDivider);
  }
  // (0x04, 10,1,0) reserved
  // (0x04, 11,1,0) reserved
  // (0x04, 12,8,1) Band Select Clock Divider
  WriteBF_dword(20, 3, MAX2870_R+4, MAX2870_RfDivSel); // rf divider select
  // (0x04, 23,8,0) reserved
  // (0x04, 24,2,1) Band Select Clock Divider MSBs
  // (0x04, 26,6,1) reserved
  // (0x05, 3,15,0) reserved
  // (0x05, 18,1,0) MUXOUT pin mode
  // (0x05, 22,2,1) lock pin function
  // (0x05, 25,7,0) reserved
  WriteRegs();

  return MAX2870_ERROR_NONE; // ok
}

// simplified setf with only f changing
int MAX2870::sweepStep(long long freq) {
  if (freq > 6000000000LL || freq < 23437500L)
    return MAX2870_ERROR_RF_FREQUENCY;

  uint8_t MAX2870_outdiv = 1 ;
  uint8_t MAX2870_RfDivSel = 0 ;
  uint32_t MAX2870_N_Int;
  uint16_t MAX2870_Mod = 2;
  uint16_t MAX2870_Frac = 0;

  uint16_t CurrentR = ReadR();
  uint8_t RDIV2 = ReadRDIV2();
  uint8_t RefDoubler = ReadRefDoubler();
  uint32_t intRef = MAX2870_reffreq*(1+RefDoubler)/(1+RDIV2);
  uint32_t PFDfreq = intRef/CurrentR; // not used for coeff. computation due to truncation
  
  // select the output divider
  MAX2870_outdiv = (uint8_t)(6000000000LL / (freq+1)); // +1, so e.g. 3000 MHz gets divided by 1
  if (MAX2870_outdiv >= 128) { MAX2870_outdiv = 128; MAX2870_RfDivSel=7; }
  else if (MAX2870_outdiv >= 64) { MAX2870_outdiv = 64; MAX2870_RfDivSel=6; }
  else if (MAX2870_outdiv >= 32) { MAX2870_outdiv = 32; MAX2870_RfDivSel=5; }
  else if (MAX2870_outdiv >= 16) { MAX2870_outdiv = 16; MAX2870_RfDivSel=4; }
  else if (MAX2870_outdiv >= 8) { MAX2870_outdiv = 8; MAX2870_RfDivSel=3; }
  else if (MAX2870_outdiv >= 4) { MAX2870_outdiv = 4; MAX2870_RfDivSel=2; }
  else if (MAX2870_outdiv >= 2) { MAX2870_outdiv = 2; MAX2870_RfDivSel=1; }
  else { MAX2870_outdiv = 1; MAX2870_RfDivSel=0; }

  long long fvcoTarget = freq*MAX2870_outdiv;

  // compute N_int
  MAX2870_N_Int = (uint16_t)(fvcoTarget*CurrentR/intRef);
  // compute actual frequency in int mode
  long long fvco = (long long)intRef*MAX2870_N_Int/CurrentR;
  // compute residual F/M (0..1)
  float res = float(fvcoTarget - fvco)*CurrentR/intRef;
  // put in best fraction form
  farey(res, 4095, &MAX2870_Frac, &MAX2870_Mod);
  // checks
  if (MAX2870_Mod == 1)
  {
    MAX2870_Mod = 2;  // minimum
    if (MAX2870_Frac == 1)
    {
      // best approx was 1/1, so we revert to 0/2 and increment N
      MAX2870_Frac = 0;
      MAX2870_N_Int++;
      fvco = (long long)intRef*MAX2870_N_Int/CurrentR;
    }
  }
  if (MAX2870_Frac >= MAX2870_Mod)
    MAX2870_Frac = MAX2870_Mod-1; // maximum
  
  if (MAX2870_Mod > 4095) {
    return MAX2870_ERROR_MOD_RANGE;
  }

  // compute actual frequency
  fvco += (long long)intRef*MAX2870_Frac/((uint32_t)CurrentR*MAX2870_Mod);
  MAX2870_currentF = fvco/MAX2870_outdiv;
  MAX2870_FrequencyError = (int32_t)(freq - MAX2870_currentF);

  if (MAX2870_Frac == 0 && (MAX2870_N_Int < 16  || MAX2870_N_Int > 65535)) {
    return MAX2870_ERROR_N_RANGE;
  }

  if (MAX2870_Frac != 0 && (MAX2870_N_Int < 19  || MAX2870_N_Int > 4091)) {
    return MAX2870_ERROR_N_RANGE_FRAC;
  }

  if (MAX2870_Frac != 0 && PFDfreq > MAX2870_PFD_MAX_FRAC) {
    return MAX2870_ERROR_PFD_EXCEEDED_WITH_FRACTIONAL_MODE;
  }

  // we need to update only registers involved in a frequency change
  MAX2870_R[0] = uint32_t(MAX2870_Frac) << 3;
  MAX2870_R[0] |= uint32_t(MAX2870_N_Int) << 15;

  if (MAX2870_Frac == 0) {
    MAX2870_R[0] |= 0x80000000L; // integer-n mode
    WriteBF_dword(29, 2, MAX2870_R+1, 0); // Charge Pump Linearity disable for INT mode
    WriteBF_dword(31, 1, MAX2870_R+1, 1); // CP clamp enable (improved INT phase noise)
    WriteBF_dword(8, 1, MAX2870_R+2, 1); // Lock Detect Function, int-n mode
    WriteBF_dword(24, 1, MAX2870_R+5, 0); // F01 not used in INT mode
  }
  else {
    WriteBF_dword(29, 2, MAX2870_R+1, 1); // Charge Pump Linearity enable for FRAC mode
    WriteBF_dword(31, 1, MAX2870_R+1, 0); // CP clamp disable in FRAC mode
    WriteBF_dword(8, 1, MAX2870_R+2, 0); // Lock Detect Function, frac-n mode
    WriteBF_dword(24, 1, MAX2870_R+5, 1); // F01 auto switch to INT if F==0
  }
  WriteBF_dword(3, 12, MAX2870_R+1, MAX2870_Mod);
  WriteBF_dword(20, 3, MAX2870_R+4, MAX2870_RfDivSel); // rf divider select
  WriteRegs();

  return MAX2870_ERROR_NONE; // ok
}

int MAX2870::setrf(uint32_t f, uint16_t r, uint8_t ReferenceDivisionType)
{
  if (f > 30000000UL && ReferenceDivisionType == MAX2870_REF_DOUBLE) return MAX2870_ERROR_DOUBLER_EXCEEDED;
  if (r > 1023 || r < 1) return MAX2870_ERROR_R_RANGE;
  if (f < MAX2870_REFIN_MIN || f > MAX2870_REFIN_MAX) return MAX2870_ERROR_REF_FREQUENCY;
  if (ReferenceDivisionType != MAX2870_REF_UNDIVIDED && ReferenceDivisionType != MAX2870_REF_HALF && ReferenceDivisionType != MAX2870_REF_DOUBLE) return MAX2870_ERROR_REF_MULTIPLIER_TYPE;

  uint32_t newF;
  if (ReferenceDivisionType == MAX2870_REF_HALF) {
    newF = f/2/r;
  }
  else if (ReferenceDivisionType == MAX2870_REF_DOUBLE) {
    newF = f*2/r;
  }
  else
    newF = f/r;
  
  if ( newF > MAX2870_PFD_MAX || newF < MAX2870_PFD_MIN ) return MAX2870_ERROR_PFD_LIMITS;

  MAX2870_reffreq = f ;
  WriteBF_dword(14, 10, MAX2870_R+2, r);
  if (ReferenceDivisionType == MAX2870_REF_DOUBLE) {
    WriteBF_dword(24, 2, MAX2870_R+2, 0b00000010);
  }
  else if (ReferenceDivisionType == MAX2870_REF_HALF) {
    WriteBF_dword(24, 2, MAX2870_R+2, 0b00000001);
  }
  else {
    WriteBF_dword(24, 2, MAX2870_R+2, 0b00000000);
  }
  return MAX2870_ERROR_NONE;
}

void MAX2870::setfDirect(uint16_t R_divider, uint16_t INT_value, uint16_t MOD_value, uint16_t FRAC_value, uint8_t RF_DIVIDER_value, bool FRACTIONAL_MODE) {
  switch (RF_DIVIDER_value) {
    case 1:
      RF_DIVIDER_value = 0;
      break;
    case 2:
      RF_DIVIDER_value = 1;
      break;
    case 4:
      RF_DIVIDER_value = 2;
      break;
    case 8:
      RF_DIVIDER_value = 3;
      break;
    case 16:
      RF_DIVIDER_value = 4;
      break;
    case 32:
      RF_DIVIDER_value = 5;
      break;
    case 64:
      RF_DIVIDER_value = 6;
      break;
    case 128:
      RF_DIVIDER_value = 7;
      break;
  }
  WriteBF_dword(14, 10, MAX2870_R+2, R_divider);
  WriteBF_dword(15, 16, MAX2870_R, INT_value);
  WriteBF_dword(3, 12, MAX2870_R+1, MOD_value);
  WriteBF_dword(3, 12, MAX2870_R, FRAC_value);
  WriteBF_dword(20, 3, MAX2870_R+4, RF_DIVIDER_value);
  if (FRACTIONAL_MODE == false) {
    WriteBF_dword(31, 1, MAX2870_R, 1); // integer-n mode
    WriteBF_dword(29, 2, MAX2870_R+1, 0); // Charge Pump Linearity
    WriteBF_dword(31, 1, MAX2870_R+1, 1); // Charge Pump Output Clamp
    WriteBF_dword(8, 1, MAX2870_R+2, 1); // Lock Detect Function, int-n mode
    WriteBF_dword(24, 1, MAX2870_R+5, 1); // integer-n mode
  }
  else {
    WriteBF_dword(31, 1, MAX2870_R, 0); // frac-n mode
    WriteBF_dword(29, 2, MAX2870_R+1, 1); // Charge Pump Linearity
    WriteBF_dword(31, 1, MAX2870_R+1, 0); // Charge Pump Output Clamp
    WriteBF_dword(8, 1, MAX2870_R+2, 0); // Lock Detect Function, frac-n mode
    WriteBF_dword(24, 1, MAX2870_R+5, 0); // frac-n mode
  }
  WriteRegs();
}

int MAX2870::setPowerLevel(uint8_t PowerLevel) {
  if (PowerLevel < 0 && PowerLevel > 4) return MAX2870_ERROR_POWER_LEVEL;
  if (PowerLevel == 0) {
    WriteBF_dword(5, 1, MAX2870_R+4, 0);
  }
  else {
    PowerLevel--;
    WriteBF_dword(5, 1, MAX2870_R+4, 1);
    WriteBF_dword(3, 2, MAX2870_R+4, PowerLevel);
  }
  WriteRegs();
  return MAX2870_ERROR_NONE;
}

int MAX2870::setAuxPowerLevel(uint8_t PowerLevel) {
  if (PowerLevel < 0 && PowerLevel > 4) return MAX2870_ERROR_POWER_LEVEL;
  if (PowerLevel == 0) {
    WriteBF_dword(8, 1, MAX2870_R+4, 0);
  }
  else {
    PowerLevel--;
    WriteBF_dword(6, 2, MAX2870_R+4, PowerLevel);
    WriteBF_dword(8, 1, MAX2870_R+4, 1);
  }
  WriteRegs();
  return MAX2870_ERROR_NONE;
}

int MAX2870::setCPcurrent(float Current) {
  if (Current < 0.32) {
    Current = 0.32;
  }
  if (Current > 5.12) {
    Current = 5.12;
  }
  Current /= 0.32;
  Current -= 0.5; // 0 = 0.32 mA per step rounded
  uint8_t CPcurrent = Current;
  WriteBF_dword(9, 4, MAX2870_R+2, CPcurrent);
  WriteRegs();
  return MAX2870_ERROR_NONE;
}

int MAX2870::setPDpolarity(uint8_t PDpolarity) {
  if (PDpolarity == MAX2870_LOOP_TYPE_INVERTING || PDpolarity == MAX2870_LOOP_TYPE_NONINVERTING) {
    WriteBF_dword(6, 1, MAX2870_R+2, PDpolarity);
    WriteRegs();
    return MAX2870_ERROR_NONE;
  }
  else {
    return MAX2870_ERROR_POLARITY_INVALID;
  }
}
