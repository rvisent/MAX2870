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
   2026oct12 v2.1: fixed SPI clock setting, integer best rational approximation of FRAC/MOD,
             automatic band select clock divider and lock detect speed, register 6 readback, VCO autoselect control

*/

#include "MAX2870.h"

MAX2870::MAX2870(uint32_t SPIclock) : MAX2870_SPI(SPIclock, MSBFIRST, SPI_MODE0)
{
  MAX2870_currentF = 0;
}

void MAX2870::writeReg(uint32_t value)
{
  digitalWrite(MAX2870_PIN_SS, LOW);
  delayMicroseconds(1);
  SPI.transfer((uint8_t)(value >> 24)); // MSB first
  SPI.transfer((uint8_t)(value >> 16));
  SPI.transfer((uint8_t)(value >> 8));
  SPI.transfer((uint8_t)value);
  delayMicroseconds(1);
  digitalWrite(MAX2870_PIN_SS, HIGH); // data latched on LE rising edge
  delayMicroseconds(1);
}

void MAX2870::WriteRegs()
{
  SPI.beginTransaction(MAX2870_SPI);
  for (int i = 5 ; i >= 0 ; i--) { // sequence according to the MAX2870 datasheet
    writeReg(MAX2870_R[i]);
  }
  SPI.endTransaction();
}

void MAX2870::writeRegister(uint8_t n)
{
  if (n > 5)
    return;
  SPI.beginTransaction(MAX2870_SPI);
  writeReg(MAX2870_R[n]);
  SPI.endTransaction();
}

void MAX2870::setMuxOut(uint8_t mode)
{
  WriteBF_dword(26, 3, MAX2870_R+2, mode & 7);
  WriteBF_dword(18, 1, MAX2870_R+5, (mode >> 3) & 1);
  // R0 is not written, so the VCO autoselect is not triggered
  writeRegister(5);
  writeRegister(2);
}

// Read sequence (datasheet): with MUX = 1100 write 32 bits ending with address 110,
// raise LE, then register 6 is shifted out MSB first on MUX_OUT (changing on CLK
// falling edges) after one more clock period. 40 bits are sampled after LE.
uint64_t MAX2870::ReadRegister6Raw()
{
  uint64_t raw = 0;
  SPI.beginTransaction(MAX2870_SPI);
  writeReg(0x00000006);
  for (int i = 0; i < 5; i++)
    raw = (raw << 8) | SPI.transfer(0x00);
  SPI.endTransaction();
  return raw;
}

uint32_t MAX2870::ReadRegister6()
{
  // bits 31:3 (29 bits) start MAX2870_R6_SKIP clocks after LE
  uint32_t value = (uint32_t)(ReadRegister6Raw() >> (40 - MAX2870_R6_SKIP - 29)) & 0x1FFFFFFFUL;
  return (value << 3) | 6;
}

uint8_t MAX2870::ReadActiveVCO()
{
  return (ReadRegister6() >> 3) & 63;
}

uint8_t MAX2870::ReadTuneADC()
{
  return (ReadRegister6() >> 20) & 7;
}

void MAX2870::setVAS(bool enable, uint8_t vco)
{
  if (enable) {
    WriteBF_dword(25, 1, MAX2870_R+3, 0); // VAS_SHDN = 0
  }
  else {
    WriteBF_dword(26, 6, MAX2870_R+3, vco & 63); // manual VCO
    WriteBF_dword(25, 1, MAX2870_R+3, 1); // VAS_SHDN = 1
  }
  writeRegister(3);
}

bool MAX2870::ReadVAS()
{
  return (MAX2870_R[3] & (1UL << 25)) == 0;
}

uint8_t MAX2870::ReadManualVCO()
{
  return (MAX2870_R[3] >> 26) & 63;
}

void MAX2870::setForceFrac(bool force)
{
  forceFrac = force;
}

// program fVCO = fvcoMHz with the given manual VCO, wait settleUs, then true if the lock detect
// pin stays high for watchUs
// set the PLL registers copies for fVCO = fvcoMHz, output divider unchanged
void MAX2870::programVCOfreq(uint16_t fvcoMHz)
{
  uint32_t refNum, refDen;
  getRefRatio(&refNum, &refDen);
  unsigned long long fvcoScaled = (unsigned long long)fvcoMHz * 1000000ULL * refDen;
  unsigned long long N = fvcoScaled / refNum;
  uint16_t F, M;
  bestRational((uint32_t)(fvcoScaled % refNum), refNum, 4095, &F, &M);
  if (M == 1) {
    if (F == 1)
      N++;
    F = 0;
    M = 2;
  }
  applyPLL((uint16_t)N, F, M, ReadOutDivider_PowerOf2());
}

// A VCO forced beyond its upper limit can take several ms to lock again once a valid VCO is
// selected, while the autoselect always recovers quickly. With prepare = true the PLL is first
// locked at fvcoMHz with the autoselect, then the VCO under test is selected: every test starts
// from the same clean state (needed after a failed test).
bool MAX2870::testLock(uint8_t vco, uint16_t fvcoMHz, uint16_t settleUs, uint16_t watchUs, bool prepare)
{
  programVCOfreq(fvcoMHz);
  if (prepare) {
    WriteBF_dword(25, 1, MAX2870_R+3, 0); // VAS on
    WriteRegs();
    delayMicroseconds(1000); // autoselect (10/fBS = 200 us) and lock
    // within the VCO range the autoselect always locks: if not, the PLL is stuck
    if (fvcoMHz >= 3000 && fvcoMHz <= 6000 && digitalRead(ldPin) == LOW) {
      delayMicroseconds(1000);
      if (digitalRead(ldPin) == LOW) {
        recoverPLL();
        WriteRegs();
        delayMicroseconds(1000);
      }
    }
  }
  WriteBF_dword(26, 6, MAX2870_R+3, vco);
  WriteBF_dword(25, 1, MAX2870_R+3, 1); // VAS off
  if (prepare) {
    writeRegister(3);
    writeRegister(0);
  }
  else {
    WriteRegs();
  }
  delayMicroseconds(settleUs);
  unsigned long t0 = micros();
  while (micros() - t0 < watchUs) {
    if (digitalRead(ldPin) == LOW)
      return false;
  }
  return true;
}

// Recovery from a stuck PLL (observed after forcing VCOs beyond their range: lock detect stays
// low, or high, and neither the autoselect nor rewriting the registers help): program the
// registers with the chip disabled, as at power up, then enable it again.
void MAX2870::recoverPLL()
{
  recoveries++;
  if (cePin != 255) {
    digitalWrite(cePin, LOW);
    WriteRegs();
    delay(MAX2870_RECOVER_DELAY_MS);
    WriteRegs();
    digitalWrite(cePin, HIGH);
  }
  else { // no CE pin: software shutdown (SHDN, register 2 bit 5)
    WriteBF_dword(5, 1, MAX2870_R+2, 1);
    WriteRegs();
    delay(MAX2870_RECOVER_DELAY_MS);
    WriteBF_dword(5, 1, MAX2870_R+2, 0);
  }
  WriteRegs();
  delay(1);
}

// calibration test: prepare (lock with the autoselect first) only after a failed test
bool MAX2870::calTest(uint8_t vco, uint16_t fvcoMHz)
{
  calOk = testLock(vco, fvcoMHz, calSettleUs, calWatchUs, !calOk);
  return calOk;
}

// test above the lower limit: tests above the lock range leave the PLL stuck, so recover after
// every failure, and confirm the failure from a clean state (a test can fail spuriously)
bool MAX2870::calTestUp(uint8_t vco, uint16_t fvcoMHz)
{
  if (calTest(vco, fvcoMHz))
    return true;
  recoverPLL();
  calOk = testLock(vco, fvcoMHz, calSettleUs, calWatchUs, true);
  if (calOk)
    return true;
  recoverPLL();
  return false;
}

// lock range of one VCO, searching upwards from fStart (MHz). false if not found
bool MAX2870::calibrateOne(uint8_t v, uint16_t fStart, uint16_t *pLo, uint16_t *pHi)
{
  uint8_t retries = 0;
  uint16_t f = fStart;
  while (f <= MAX2870_VCO_CAL_MAX_MHZ) {
    if (!calTest(v, f)) {
      f += calStepMHz;
      continue;
    }
    uint16_t lo = f, hi = f;
    if (f == fStart) { // already locked at the start: extend downwards
      while (lo >= MAX2870_VCO_CAL_MIN_MHZ + calStepMHz && calTest(v, lo - calStepMHz))
        lo -= calStepMHz;
    }
    while (hi + calStepMHz <= MAX2870_VCO_CAL_MAX_MHZ && calTestUp(v, hi + calStepMHz))
      hi += calStepMHz;
    if (hi - lo < MAX2870_VCO_MIN_RANGE_MHZ) {
      f = hi + calStepMHz; // isolated false lock indication, look further up
      continue;
    }
    // refine the limits to 1 MHz (a: unlocked, b: locked)
    if (lo >= MAX2870_VCO_CAL_MIN_MHZ + calStepMHz) {
      uint16_t a = lo - calStepMHz, b = lo;
      while (b - a > 1) {
        uint16_t m = (a + b) / 2;
        if (calTest(v, m)) b = m; else a = m;
      }
      lo = b;
    }
    if (hi + calStepMHz <= MAX2870_VCO_CAL_MAX_MHZ) {
      uint16_t a = hi, b = hi + calStepMHz;
      while (b - a > 1) {
        uint16_t m = (a + b) / 2;
        if (calTestUp(v, m)) a = m; else b = m;
      }
      hi = a;
    }
    if (hi - lo > MAX2870_VCO_MAX_RANGE_MHZ && retries < 2) {
      // implausible range: lock detect stuck high, recover and test this VCO again
      recoverPLL();
      retries++;
      calOk = false;
      f = fStart;
      continue;
    }
    *pLo = lo;
    *pHi = hi;
    return true;
  }
  return false;
}

// Distance (MHz) of the limits of VCO v from the values expected from the neighbouring
// sub-bands of the same VCO core (16 sub-bands per core): interpolation between v-1 and v+1,
// or extrapolation at the ends of the core. Limits at the calibration range ends are not checked.
uint16_t MAX2870::vcoDeviation(uint8_t vco)
{
  int v = vco; // signed: v - 1 and v - 2 can be negative
  uint16_t worst = 0;
  for (uint8_t i = 0; i < 2; i++) {
    uint16_t *lim = (i == 0) ? vcoMin : vcoMax;
#define VCO_USABLE(u) ((u) >= 0 && (u) < 64 && (u) / 16 == v / 16 && vcoMin[u] != 0 && \
                       lim[u] > MAX2870_VCO_CAL_MIN_MHZ + 5 && lim[u] < MAX2870_VCO_CAL_MAX_MHZ - 10)
    if (!VCO_USABLE(v))
      continue;
    int32_t expected;
    if (VCO_USABLE(v - 1) && VCO_USABLE(v + 1))
      expected = ((int32_t)lim[v - 1] + lim[v + 1]) / 2;
    else if (VCO_USABLE(v + 1) && VCO_USABLE(v + 2))
      expected = 2 * (int32_t)lim[v + 1] - lim[v + 2];
    else if (VCO_USABLE(v - 1) && VCO_USABLE(v - 2))
      expected = 2 * (int32_t)lim[v - 1] - lim[v - 2];
    else
      continue;
#undef VCO_USABLE
    uint16_t d = abs((int32_t)lim[v] - expected);
    if (d > worst)
      worst = d;
  }
  return worst;
}

bool MAX2870::checkReadback()
{
  setMuxOut(MAX2870_MUX_READBACK);
  uint32_t r3 = MAX2870_R[3];
  bool ok = true;
  const uint8_t testVCO[2] = {21, 42}; // 010101, 101010
  for (uint8_t i = 0; i < 2 && ok; i++) {
    WriteBF_dword(26, 6, MAX2870_R+3, testVCO[i]);
    WriteBF_dword(25, 1, MAX2870_R+3, 1); // VAS off
    writeRegister(3);
    delayMicroseconds(50);
    ok = (ReadActiveVCO() == testVCO[i]);
  }
  MAX2870_R[3] = r3;
  writeRegister(3);
  return ok;
}

int MAX2870::calibrateVCO(uint16_t stepMHz, uint16_t settleUs, uint16_t watchUs, uint8_t method)
{
  if (stepMHz == 0)
    return -1;
  uint32_t saved[6];
  ReadSweepValues(saved);
  WriteBF_dword(5, 1, MAX2870_R+4, 0); // RF outputs muted
  WriteBF_dword(8, 1, MAX2870_R+4, 0);
  WriteRegs();
  if (method == MAX2870_CAL_AUTO)
    method = checkReadback() ? MAX2870_CAL_TUNE : MAX2870_CAL_LOCK;
  else if (method == MAX2870_CAL_TUNE && !checkReadback())
    method = 255;
  if (method == MAX2870_CAL_LOCK && ldPin == 255)
    method = 255;
  calStepMHz = stepMHz;
  calSettleUs = settleUs;
  calWatchUs = watchUs;
  calOk = false;
  recoveries = 0;
  rechecks = 0;

  int found = -1;
  if (method == MAX2870_CAL_TUNE)
    found = calibrateTune();
  else if (method == MAX2870_CAL_LOCK)
    found = calibrateLock();
  if (found >= 0) {
    vcoTableValid = (found > 0);
    vcoTableKind = method;
  }

  WriteSweepValues(saved); // restore registers, outputs and frequency
  delay(2);
  for (int i = 0; i < 3 && ldPin != 255 && ReadVAS() && digitalRead(ldPin) == LOW; i++) {
    recoverPLL();
    delay(2);
  }
  return found;
}

// program fVCO with manual VCO v, wait the settle time, return the VTUNE ADC and the lock state
uint8_t MAX2870::tuneTest(uint8_t v, uint16_t fvcoMHz, bool *locked)
{
  programVCOfreq(fvcoMHz);
  WriteBF_dword(26, 6, MAX2870_R+3, v);
  WriteBF_dword(25, 1, MAX2870_R+3, 1); // VAS off
  WriteRegs();
  delayMicroseconds(calSettleUs);
  *locked = (ldPin == 255) || (digitalRead(ldPin) == HIGH);
  return ReadTuneADC();
}

// VTUNE calibration: for every VCO the range with VTUNE 0.7-2.1 V (ADC 010 or 101). The scan goes
// upwards and stops as soon as VTUNE exceeds 2.1 V, so the VCO is never forced beyond its range.
int MAX2870::calibrateTune()
{
#define TUNE_GOOD(a, l) ((l) && ((a) == 2 || (a) == 5))
  int found = 0;
  uint16_t fStart = MAX2870_VCO_CAL_MIN_MHZ;
  for (uint8_t v = 0; v < 64; v++) {
    vcoMin[v] = vcoMax[v] = 0;
    bool lk;
    uint16_t f = fStart;
    uint8_t a = tuneTest(v, f, &lk);
    // start below the good range (VTUNE < 0.7 V): the VCO ranges are not strictly monotonic
    while (a >= 2 && f >= MAX2870_VCO_CAL_MIN_MHZ + calStepMHz) {
      f -= calStepMHz;
      a = tuneTest(v, f, &lk);
    }
    uint16_t lo = 0, hi = 0;
    uint16_t above = 0; // first frequency above the good range
    bool unlockedAbove = false;
    while (f <= MAX2870_VCO_CAL_MAX_MHZ) {
      a = tuneTest(v, f, &lk);
      if (TUNE_GOOD(a, lk)) {
        if (lo == 0)
          lo = f;
        hi = f;
      }
      else if (a >= 6 || (lo != 0 && !lk)) {
        above = f;
        unlockedAbove = !lk;
        break;
      }
      f += calStepMHz;
    }
    if (unlockedAbove)
      recoverPLL(); // the VCO went beyond its range: the PLL could be stuck
    if (lo == 0)
      continue;
    // refine the limits to 1 MHz
    if (lo >= MAX2870_VCO_CAL_MIN_MHZ + calStepMHz) {
      uint16_t x = lo - calStepMHz, y = lo; // x: below, y: good
      while (y - x > 1) {
        uint16_t m = (x + y) / 2;
        a = tuneTest(v, m, &lk);
        if (TUNE_GOOD(a, lk)) y = m; else x = m;
      }
      lo = y;
    }
    if (above != 0) {
      uint16_t x = hi, y = above; // x: good, y: above
      while (y - x > 1) {
        uint16_t m = (x + y) / 2;
        a = tuneTest(v, m, &lk);
        if (TUNE_GOOD(a, lk)) x = m; else y = m;
      }
      hi = x;
    }
    vcoMin[v] = lo;
    vcoMax[v] = hi;
    found++;
    fStart = lo;
  }
#undef TUNE_GOOD
  return found;
}

// lock range calibration with the lock detect pin
int MAX2870::calibrateLock()
{
  int found = 0;
  uint16_t fStart = MAX2870_VCO_CAL_MIN_MHZ;
  for (uint8_t v = 0; v < 64; v++) {
    uint16_t lo, hi;
    if (calibrateOne(v, fStart, &lo, &hi)) {
      vcoMin[v] = lo;
      vcoMax[v] = hi;
      found++;
      fStart = lo; // VCO frequency increases with the VCO number
    }
    else {
      vcoMin[v] = vcoMax[v] = 0;
    }
  }
  vcoTableValid = (found > 0);

  // Consistency check: measure again (up to twice) the VCOs whose limits differ too much from
  // the neighbouring sub-bands, keeping the most consistent measurement
  uint8_t tries[64] = {0};
  for (uint8_t n = 0; n < MAX2870_VCO_MAX_RECHECKS; n++) {
    int8_t worstV = -1;
    uint16_t worstDev = MAX2870_VCO_CONSISTENCY_MHZ;
    for (uint8_t v = 0; v < 64; v++) {
      uint16_t d = vcoDeviation(v);
      if (tries[v] < 2 && d > worstDev) {
        worstDev = d;
        worstV = v;
      }
    }
    if (worstV < 0)
      break;
    uint8_t v = worstV;
    tries[v]++;
    rechecks++;
    uint16_t oldMin = vcoMin[v], oldMax = vcoMax[v];
    uint16_t start = (v > 0 && vcoMin[v - 1] != 0) ? vcoMin[v - 1] : MAX2870_VCO_CAL_MIN_MHZ;
    if (oldMin != 0 && oldMin < start)
      start = oldMin;
    if (start > MAX2870_VCO_CAL_MIN_MHZ + 50)
      start -= 50;
    else
      start = MAX2870_VCO_CAL_MIN_MHZ;
    recoverPLL();
    calOk = false;
    uint16_t lo, hi;
    if (calibrateOne(v, start, &lo, &hi)) {
      vcoMin[v] = lo;
      vcoMax[v] = hi;
      if (vcoDeviation(v) > worstDev) { // worse than before: keep the old values
        vcoMin[v] = oldMin;
        vcoMax[v] = oldMax;
      }
    }
  }
  return found;
}

bool MAX2870::ReadVCOTable(uint8_t vco, uint16_t *minMHz, uint16_t *maxMHz)
{
  if (vco > 63)
    return false;
  *minMHz = vcoMin[vco];
  *maxMHz = vcoMax[vco];
  return vcoMin[vco] != 0;
}

void MAX2870::setVCOTable(uint8_t vco, uint16_t minMHz, uint16_t maxMHz)
{
  if (vco > 63)
    return;
  vcoMin[vco] = minMHz;
  vcoMax[vco] = maxMHz;
  if (minMHz != 0)
    vcoTableValid = true;
}

bool MAX2870::VCOTableValid()
{
  return vcoTableValid;
}

uint8_t MAX2870::ReadVCOTableKind()
{
  return vcoTableKind;
}

void MAX2870::setVCOTableKind(uint8_t kind)
{
  if (kind == MAX2870_CAL_LOCK || kind == MAX2870_CAL_TUNE)
    vcoTableKind = kind;
}

void MAX2870::useVCOTable(bool use, int16_t marginMHz)
{
  vcoTableUse = use;
  if (marginMHz < 0)
    marginMHz = (vcoTableKind == MAX2870_CAL_TUNE) ? MAX2870_VCO_MARGIN_TUNE_MHZ : MAX2870_VCO_MARGIN_MHZ;
  vcoMargin = marginMHz;
  if (!use)
    WriteBF_dword(25, 1, MAX2870_R+3, 0); // back to VCO autoselect at the next write
}

bool MAX2870::ReadUseVCOTable()
{
  return vcoTableUse && vcoTableValid;
}

// keep current VCO if fvcoMHz is at least vcoMargin from its limits, otherwise the most centered one
int8_t MAX2870::selectVCO(uint16_t fvcoMHz, int8_t current)
{
  if (!vcoTableValid)
    return -1;
  if (current >= 0 && current < 64 && vcoMin[current] != 0 &&
      fvcoMHz >= vcoMin[current] + vcoMargin && fvcoMHz + vcoMargin <= vcoMax[current])
    return current;
  int8_t best = -1;
  int32_t bestScore = -1;
  for (int8_t v = 0; v < 64; v++) {
    if (vcoMin[v] == 0 || fvcoMHz < vcoMin[v] || fvcoMHz > vcoMax[v])
      continue;
    int32_t score = min(fvcoMHz - vcoMin[v], vcoMax[v] - fvcoMHz);
    if (score > bestScore) {
      bestScore = score;
      best = v;
    }
  }
  return best;
}

// select the VCO from the table (R3 updated, written with the other registers)
void MAX2870::applyVCO(long long freq, uint8_t divSel, bool keepCurrent)
{
  if (!vcoTableUse || !vcoTableValid)
    return;
  uint16_t fvcoMHz = (uint16_t)(((freq << divSel) + 500000LL) / 1000000LL);
  int8_t current = (keepCurrent && !ReadVAS()) ? (int8_t)ReadManualVCO() : -1;
  int8_t v = selectVCO(fvcoMHz, current);
  if (v < 0) {
    WriteBF_dword(25, 1, MAX2870_R+3, 0); // not in the table: VCO autoselect
  }
  else {
    WriteBF_dword(26, 6, MAX2870_R+3, v);
    WriteBF_dword(25, 1, MAX2870_R+3, 1);
  }
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

// fPFD = refNum/refDen exactly, refNum = REFIN*(1+D), refDen = R*(1+T)
void MAX2870::getRefRatio(uint32_t *pRefNum, uint32_t *pRefDen) {
  *pRefNum = MAX2870_reffreq*(1+ReadRefDoubler());
  *pRefDen = (uint32_t)ReadR()*(1+ReadRDIV2());
}

uint32_t MAX2870::ReadPFDfreq() {
  uint32_t refNum, refDen;
  getRefRatio(&refNum, &refDen);
  if (refDen == 0) { // avoid division by zero
    return 0;
  }
  return (refNum + refDen/2)/refDen; // rounded
}

int32_t MAX2870::ReadFrequencyError() {
  return MAX2870_FrequencyError;
}

long long MAX2870::ReadCurrentFrequency(void)
{
  return MAX2870_currentF;
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
  uint32_t mask = ((1UL<<nBits)-1)<<offset;
  *pdw = (*pdw & ~mask) | (((uint32_t)insData<<offset) & mask);
}

// Best rational approximation pNum/pDen of num/den (0 <= num < den) with pDen <= maxDen.
// Continued fraction expansion: the best approximation is either the last convergent
// within maxDen or the largest semiconvergent following it. Integer arithmetic only,
// number of iterations is O(log(den)).
void MAX2870::bestRational(uint32_t num, uint32_t den, uint16_t maxDen, uint16_t *pNum, uint16_t *pDen)
{
  // convergents h(k-2)/k(k-2) = p0/q0, h(k-1)/k(k-1) = p1/q1
  uint32_t p0 = 0, q0 = 1, p1 = 1, q1 = 0;
  uint32_t n = num, d = den;

  while (d != 0) {
    uint32_t a = n/d;
    // stop if next convergent denominator q0 + a*q1 would exceed maxDen (checked without overflow)
    if (q1 != 0 && a > (maxDen - q0)/q1)
      break;
    uint32_t p2 = p0 + a*p1;
    uint32_t q2 = q0 + a*q1;
    p0 = p1; q0 = q1;
    p1 = p2; q1 = q2;
    uint32_t r = n - a*d;
    n = d;
    d = r;
  }

  if (d != 0) {
    // not exact: compare last convergent with the largest admissible semiconvergent
    uint32_t k = (maxDen - q0)/q1;
    uint32_t ps = p0 + k*p1;
    uint32_t qs = q0 + k*q1;
    // |p/q - num/den| compared as |p*den - num*q| / q, cross multiplied
    int64_t e1 = (int64_t)p1*den - (int64_t)num*q1;
    int64_t es = (int64_t)ps*den - (int64_t)num*qs;
    if (e1 < 0) e1 = -e1;
    if (es < 0) es = -es;
    if ((uint64_t)es*q1 < (uint64_t)e1*qs) {
      p1 = ps;
      q1 = qs;
    }
  }

  *pNum = (uint16_t)p1;
  *pDen = (uint16_t)q1;
}

// compute PLL parameters for the requested frequency, no register changed
int MAX2870::calcPLL(long long freq, uint16_t *pN, uint16_t *pF, uint16_t *pM, uint8_t *pDivSel, long long *pActual)
{
  if (freq > 6000000000LL || freq < 23437500L)
    return MAX2870_ERROR_RF_FREQUENCY;

  uint32_t refNum, refDen;
  getRefRatio(&refNum, &refDen);
  if (refNum == 0 || refDen == 0)
    return MAX2870_ERROR_ZERO_PFD_FREQUENCY;

  // output divider: largest power of 2 keeping fVCO <= 6 GHz (3 GHz is divided by 1)
  uint8_t divSel = 0;
  while (divSel < 7 && ((freq+1) << (divSel+1)) <= 6000000000LL)
    divSel++;

  // fVCO/fPFD = fVCO*refDen/refNum = N + rem/refNum, exact
  unsigned long long fvcoScaled = (unsigned long long)(freq << divSel) * refDen;
  unsigned long long N = fvcoScaled / refNum;
  uint32_t rem = (uint32_t)(fvcoScaled % refNum);

  uint16_t F, M;
  bestRational(rem, refNum, 4095, &F, &M);
  if (M == 1) {
    if (F == 1)
      N++;  // best approximation was 1/1
    F = 0;
    M = 2;  // minimum
  }

  bool fracMode = (F != 0 || forceFrac);
  if (!fracMode && (N < 16 || N > 65535))
    return MAX2870_ERROR_N_RANGE;
  if (fracMode && (N < 19 || N > 4091))
    return MAX2870_ERROR_N_RANGE_FRAC;
  if (fracMode && refNum > (unsigned long long)MAX2870_PFD_MAX_FRAC*refDen)
    return MAX2870_ERROR_PFD_EXCEEDED_WITH_FRACTIONAL_MODE;

  // actual output frequency = fPFD*(N + F/M)/outdiv, rounded to 1 Hz
  unsigned long long den = (unsigned long long)refDen*M << divSel;
  *pActual = (long long)(((unsigned long long)refNum*(N*M + F) + den/2) / den);
  *pN = (uint16_t)N;
  *pF = F;
  *pM = M;
  *pDivSel = divSel;
  return MAX2870_ERROR_NONE;
}

// write PLL parameters to the register copies (not to the chip)
void MAX2870::applyPLL(uint16_t N, uint16_t F, uint16_t M, uint8_t divSel)
{
  MAX2870_R[0] = uint32_t(F) << 3;
  MAX2870_R[0] |= uint32_t(N) << 15;

  if (F == 0 && !forceFrac) {
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
    // F01: auto switch to INT if F==0, unless fractional mode is forced
    WriteBF_dword(24, 1, MAX2870_R+5, forceFrac ? 0 : 1);
  }
  // (0x01, 15, 12, 1) phase
  WriteBF_dword(3, 12, MAX2870_R+1, M);
  WriteBF_dword(20, 3, MAX2870_R+4, divSel); // rf divider select
}

// settings depending only on fPFD: band select clock divider and lock detect speed
void MAX2870::updatePFDsettings()
{
  uint32_t refNum, refDen;
  getRefRatio(&refNum, &refDen);
  if (refDen == 0)
    return;

  // band select clock fBS = fPFD/BS must be set to 50 kHz (datasheet: BS rounded to nearest)
  unsigned long long bsDen = (unsigned long long)refDen*MAX2870_BS_FREQ_MAX;
  uint32_t BS = (uint32_t)((refNum + bsDen/2)/bsDen);
  if (BS < 1)
    BS = 1;
  if (BS > 1023)
    BS = 1023; // field limit, fBS > 50 kHz only if fPFD > 51.15 MHz
  WriteBF_dword(12, 8, MAX2870_R+4, BS & 0xFF); // BS LSBs
  WriteBF_dword(24, 2, MAX2870_R+4, BS >> 8);   // BS MSBs

  // lock detect speed
  if (refNum > (unsigned long long)32000000UL*refDen)
    WriteBF_dword(31, 1, MAX2870_R+2, 1);
  else
    WriteBF_dword(31, 1, MAX2870_R+2, 0);
}

void MAX2870::init(uint8_t SSpin, uint8_t LockPinNumber, bool Lock_Pin_Used, uint8_t CEpinNumber, bool CE_Pin_Used)
{
  MAX2870_PIN_SS = SSpin;
  pinMode(MAX2870_PIN_SS, OUTPUT) ;
  digitalWrite(MAX2870_PIN_SS, HIGH) ;
  if (CE_Pin_Used == true) {
    pinMode(CEpinNumber, OUTPUT) ;
    cePin = CEpinNumber;
  }
  if (Lock_Pin_Used == true) {
    pinMode(LockPinNumber, INPUT_PULLUP) ;
    ldPin = LockPinNumber;
  }
  SPI.begin();

  // initialize chip as recommended in the datasheet
  // RF outputs are disabled on default values
  for (int j = 0 ; j < 2 ; j++) {
    WriteRegs();
    delay(25);  // at least 20 ms delay
  }
}

int  MAX2870::setf(long long freq, uint8_t PowerLevel, uint8_t AuxPowerLevel, uint8_t AuxFrequencyDivider) {
  if (PowerLevel > 4) return MAX2870_ERROR_POWER_LEVEL;
  if (AuxPowerLevel > 4) return MAX2870_ERROR_AUX_POWER_LEVEL;
  if (AuxFrequencyDivider != MAX2870_AUX_DIVIDED && AuxFrequencyDivider != MAX2870_AUX_FUNDAMENTAL) return MAX2870_ERROR_AUX_FREQ_DIVIDER;

  uint16_t N, F, M;
  uint8_t divSel;
  long long fActual;
  int err = calcPLL(freq, &N, &F, &M, &divSel, &fActual);
  if (err != MAX2870_ERROR_NONE)
    return err;

  MAX2870_currentF = fActual;
  MAX2870_FrequencyError = (int32_t)(freq - fActual);
  applyPLL(N, F, M, divSel);
  applyVCO(freq, divSel, false); // most centered VCO if the table is used

  // (0x02, 3,1,0) counter reset
  // (0x02, 4,1,0) cp3 state
  // (0x02, 5,1,0) power down
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
    WriteBF_dword(9, 1, MAX2870_R+4, AuxFrequencyDivider);
  }
  // (0x04, 10,1,0) reserved
  // (0x04, 11,1,0) reserved
  // (0x04, 12,8,x) Band Select Clock Divider, set by setrf
  // (0x04, 23,8,0) reserved
  // (0x04, 24,2,x) Band Select Clock Divider MSBs, set by setrf
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
  uint16_t N, F, M;
  uint8_t divSel;
  long long fActual;
  int err = calcPLL(freq, &N, &F, &M, &divSel, &fActual);
  if (err != MAX2870_ERROR_NONE)
    return err;

  MAX2870_currentF = fActual;
  MAX2870_FrequencyError = (int32_t)(freq - fActual);
  applyPLL(N, F, M, divSel);
  applyVCO(freq, divSel, true); // keep the current VCO if far enough from its limits
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
  updatePFDsettings();
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
  updatePFDsettings();
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
