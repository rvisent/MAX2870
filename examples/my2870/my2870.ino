/*

  MAX2870 demo by Bryce Cherry, version by RV

  Serial protocol: 115200 baud, one command per line (LF or CR), case insensitive, fields separated by spaces.
  Every command ends with exactly one line "OK" or "ERR <code> <message>".
  Commands changing the synthesizer state print a status line before OK:
    S f=<actual Hz> req=<requested Hz> err=<req-actual Hz> ref=<Hz> r=<R> refmode=<NONE|HALF|DOUBLE> pfd=<Hz>
      int=<N> frac=<F> mod=<M> div=<outdiv> pwr=<0-4> aux=<0-4> cp=<mA> pd=<INV|NONINV> ce=<0|1> lock=<0|1>
      vmode=<AUTO|TABLE|MANUAL> vco=<VCO 0-63, -1 = unknown> adc=<VTUNE ADC 0-7, -1 = no readback>
      ffrac=<0|1>
  Lines starting with '#' are informational, "R ..." register dumps, "L ..." sweep lock log,
  "T ..." VCO table, "V ..." VCO lock scan. Empty lines are ignored.

  Commands:
  S | STATUS                         print status
  F freq_Hz [power(0-4)]             set frequency (power: 0 off, 1-4 = -4/-1/+2/+5 dBm; avoid 1 on the
                                     test board: some VCOs noisy or off frequency at -4 dBm)
  FD delta_Hz                        change frequency by delta from the last requested frequency
  P power(0-4)                       set main output power level
  REF ref_Hz R [NONE|HALF|DOUBLE]    set reference frequency, R divider and doubler/halver, then reapply frequency
  CP current_mA                      set charge pump current (0.32-5.12 mA with RSET = 5.1 kOhm)
  PD INV|NONINV                      phase detector polarity
  CE ON|OFF                          enable/disable MAX2870
  SWEEP start_Hz stop_Hz step_Hz dwell_ms [power(0-4)] [LOG|LOGALL]
                                     repeated sweep until any line is received, then OK
                                     LOG: print a line for steps with anomalous lock (unlocked at the end of the
                                     dwell time, more than 150 us unlocked or more than 3 unlock pulses),
                                     LOGALL: print a line for every step
                                     L f=<Hz> n=<unlock pulses> low=<total unlock us> lock=<us to final lock>
                                       vco=<VCO, -1 unknown> adc=<VTUNE ADC, -1 no readback> [UNLOCKED]
  CAL [step_MHz] [AUTO|LOCK|TUNE]    calibrate the VCO table (RF outputs muted), then print it:
                                     T vco=<n> min=<MHz> max=<MHz> kind=<LOCK|TUNE>
                                     TUNE: range with VTUNE 0.7-2.1 V (register 6 readback), LOCK: lock
                                     range (lock detect pin), AUTO (default): TUNE if the readback works
  VTAB [vco min_MHz max_MHz [LOCK|TUNE]]
                                     print the VCO table, or set one entry (e.g. to restore a saved table)
  VMODE AUTO|TABLE [margin_MHz]      VCO selection: autoselect at every frequency change, or from the table
                                     (no autoselect, the VCO is kept in sweeps while margin_MHz from its limits;
                                     default 60 MHz for a LOCK table, 5 for a TUNE table)
  VAS ON|OFF                         VCO autoselect on, or off keeping the current VCO (manual mode)
  VCO n(0-63)                        manual VCO selection
  VSCAN freq_Hz [settle_ms [watch_ms]]
                                     lock test of every VCO at freq_Hz: V f=<Hz> vco=<n> n=<unlock pulses> low=<us> [LOCK]
  FT freq_Hz [watch_ms]              set frequency and measure the lock time on the lock detect pin
  FRAC ON|OFF                        force fractional mode also for integer N (no loop setting changes)
  REG                                dump registers R0-R5 (copies) and R6 (readback)
  REG n(0-5) hex_value               write one register (address bits are forced), REG W writes all
  BURST on_us off_us count [AUX]     on/off bursts on the current frequency, count 0 = until any line is received
  FREQ_DIRECT R INT MOD FRAC outdiv FRACMODE(0|1)
                                     set PLL parameters directly (not checked)
  HELP                               list of commands

*/

#include "MAX2870.h"

MAX2870 vfo(4000000UL);  // SPI clock 4 MHz

const byte SSpin = 48; // LE
const byte LockPin = 50;
const byte CEpin = 52;

const unsigned long SerialPortRate = 115200;

// sketch level error codes, library codes are below 100
#define ERR_UNKNOWN_COMMAND 100
#define ERR_BAD_ARGUMENTS 101
#define ERR_LINE_TOO_LONG 102
#define ERR_OUT_OF_RANGE 103
#define ERR_NO_VCO_TABLE 104
#define ERR_NO_LOCK_PIN 105
#define ERR_NO_READBACK 106

const int CommandSize = 80;
char Command[CommandSize];
const int MaxArgs = 8;
char *Arg[MaxArgs];
int ArgCount;

long long RequestedFreq = 0; // last requested frequency, base for FD
bool CEstate = false;
bool ForceFrac = false;
bool ReadbackOK = false; // register 6 readback works (checked at startup)

// lock detect statistics over a time window
struct LockStats {
  uint16_t pulses;        // number of unlock pulses
  unsigned long lowTime;  // total unlocked time, us
  unsigned long lockTime; // time of the last unlock->lock transition, us from start
  bool locked;            // state at the end of the window
};

// discard any input still arriving, e.g. the rest of the line used to stop a sweep
void FlushSerialInput() {
  unsigned long last = millis();
  while (millis() - last < 5) {
    if (Serial.available() > 0) {
      Serial.read();
      last = millis();
    }
  }
}

void printHex32(uint32_t value) {
  for (int shift = 28; shift >= 0; shift -= 4)
    Serial.print((value >> shift) & 15, HEX);
}

void printLL(long long value) {
  if (value < 0) {
    Serial.print('-');
    value = -value;
  }
  unsigned long high = (unsigned long)(value / 1000000000LL);
  unsigned long low = (unsigned long)(value % 1000000000LL);
  if (high > 0) {
    Serial.print(high);
    for (unsigned long d = 100000000UL; d > 1 && low < d; d /= 10)
      Serial.print('0');
  }
  Serial.print(low);
}

const __FlashStringHelper *ErrorText(int code) {
  switch (code) {
    case MAX2870_ERROR_RF_FREQUENCY: return F("RF frequency out of range");
    case MAX2870_ERROR_POWER_LEVEL: return F("Power level incorrect");
    case MAX2870_ERROR_AUX_POWER_LEVEL: return F("Auxiliary power level incorrect");
    case MAX2870_ERROR_AUX_FREQ_DIVIDER: return F("Auxiliary frequency divider incorrect");
    case MAX2870_ERROR_ZERO_PFD_FREQUENCY: return F("PFD frequency is zero");
    case MAX2870_ERROR_MOD_RANGE: return F("MOD is out of range");
    case MAX2870_ERROR_FRAC_RANGE: return F("FRAC is out of range");
    case MAX2870_ERROR_N_RANGE: return F("N is out of range");
    case MAX2870_ERROR_N_RANGE_FRAC: return F("N is out of range in fractional mode");
    case MAX2870_ERROR_PFD_EXCEEDED_WITH_FRACTIONAL_MODE: return F("PFD exceeds 50 MHz in fractional mode");
    case MAX2870_ERROR_DOUBLER_EXCEEDED: return F("Reference frequency with doubler exceeded");
    case MAX2870_ERROR_R_RANGE: return F("R divider is out of range");
    case MAX2870_ERROR_REF_FREQUENCY: return F("Reference frequency is out of range");
    case MAX2870_ERROR_REF_MULTIPLIER_TYPE: return F("Reference multiplier type is incorrect");
    case MAX2870_ERROR_PFD_LIMITS: return F("PFD frequency is out of range");
    case MAX2870_ERROR_POLARITY_INVALID: return F("Phase detector polarity invalid");
    case ERR_UNKNOWN_COMMAND: return F("Unknown command");
    case ERR_BAD_ARGUMENTS: return F("Bad or missing arguments");
    case ERR_LINE_TOO_LONG: return F("Command line too long");
    case ERR_OUT_OF_RANGE: return F("Argument out of range");
    case ERR_NO_VCO_TABLE: return F("VCO table not available, use CAL");
    case ERR_NO_LOCK_PIN: return F("Lock detect pin not available");
    case ERR_NO_READBACK: return F("Register 6 readback not available");
    default: return F("Error");
  }
}

void Reply(int code) {
  if (code == MAX2870_ERROR_NONE) {
    Serial.println(F("OK"));
  }
  else {
    Serial.print(F("ERR "));
    Serial.print(code);
    Serial.print(' ');
    Serial.println(ErrorText(code));
  }
}

void PrintStatus() {
  Serial.print(F("S f=")); printLL(vfo.ReadCurrentFrequency());
  Serial.print(F(" req=")); printLL(RequestedFreq);
  Serial.print(F(" err=")); Serial.print(vfo.ReadFrequencyError());
  Serial.print(F(" ref=")); Serial.print(vfo.MAX2870_reffreq);
  Serial.print(F(" r=")); Serial.print(vfo.ReadR());
  Serial.print(F(" refmode="));
  if (vfo.ReadRDIV2() != 0)
    Serial.print(F("HALF"));
  else if (vfo.ReadRefDoubler() != 0)
    Serial.print(F("DOUBLE"));
  else
    Serial.print(F("NONE"));
  Serial.print(F(" pfd=")); Serial.print(vfo.ReadPFDfreq());
  Serial.print(F(" int=")); Serial.print(vfo.ReadInt());
  Serial.print(F(" frac=")); Serial.print(vfo.ReadFraction());
  Serial.print(F(" mod=")); Serial.print(vfo.ReadMod());
  Serial.print(F(" div=")); Serial.print(vfo.ReadOutDivider());
  Serial.print(F(" pwr=")); Serial.print(vfo.ReadPowerLevel());
  Serial.print(F(" aux=")); Serial.print(vfo.ReadAuxPowerLevel());
  Serial.print(F(" cp=")); Serial.print(0.32*(((vfo.MAX2870_R[2] >> 9) & 15)+1), 2);
  Serial.print(F(" pd=")); Serial.print((vfo.MAX2870_R[2] & 0x40) ? F("NONINV") : F("INV"));
  Serial.print(F(" ce=")); Serial.print(CEstate ? 1 : 0);
  Serial.print(F(" lock=")); Serial.print(digitalRead(LockPin) == HIGH ? 1 : 0);
  Serial.print(F(" vmode="));
  if (vfo.ReadUseVCOTable())
    Serial.print(F("TABLE"));
  else if (vfo.ReadVAS())
    Serial.print(F("AUTO"));
  else
    Serial.print(F("MANUAL"));
  Serial.print(F(" vco=")); Serial.print(CurrentVCO());
  Serial.print(F(" adc=")); Serial.print(ReadbackOK ? (int)vfo.ReadTuneADC() : -1);
  Serial.print(F(" ffrac=")); Serial.print(ForceFrac ? 1 : 0);
  Serial.println();
}

// VCO in use: read back from register 6 if possible, else the manual VCO (-1 = autoselect)
int CurrentVCO() {
  if (ReadbackOK)
    return vfo.ReadActiveVCO();
  return vfo.ReadVAS() ? -1 : vfo.ReadManualVCO();
}

const __FlashStringHelper *TableKindName(uint8_t kind) {
  return kind == MAX2870_CAL_TUNE ? F("TUNE") : F("LOCK");
}

void PrintVCOTableEntry(uint8_t v, uint16_t lo, uint16_t hi) {
  Serial.print(F("T vco=")); Serial.print(v);
  Serial.print(F(" min=")); Serial.print(lo);
  Serial.print(F(" max=")); Serial.print(hi);
  Serial.print(F(" kind=")); Serial.println(TableKindName(vfo.ReadVCOTableKind()));
}

void PrintVCOTable() {
  for (uint8_t v = 0; v < 64; v++) {
    uint16_t lo, hi;
    if (vfo.ReadVCOTable(v, &lo, &hi))
      PrintVCOTableEntry(v, lo, hi);
  }
}

// MHz of fVCO between 3000 and 6000 MHz not covered by the VCO table
int VCOTableGaps() {
  int gaps = 0;
  for (uint16_t f = 3000; f <= 6000; f++)
    if (vfo.selectVCO(f, -1) < 0)
      gaps++;
  return gaps;
}

void PrintRegisters() {
  Serial.print(F("R"));
  for (int i = 0; i < 6; i++) {
    Serial.print(F(" r")); Serial.print(i); Serial.print('=');
    printHex32(vfo.MAX2870_R[i]);
  }

  uint64_t raw = vfo.ReadRegister6Raw();
  uint32_t r6 = vfo.ReadRegister6();
  Serial.print(F(" r6=")); printHex32(r6);
  Serial.print(F(" raw40=")); Serial.print((uint32_t)(raw >> 32) & 0xFF, HEX); printHex32((uint32_t)raw);
  Serial.print(F(" por=")); Serial.print((r6 >> 23) & 1);
  Serial.print(F(" adc=")); Serial.print((r6 >> 20) & 7);
  Serial.print(F(" vco=")); Serial.println((r6 >> 3) & 63);
}

void PrintHelp() {
  Serial.println(F("# S | STATUS"));
  Serial.println(F("# F freq_Hz [power(0-4)]"));
  Serial.println(F("# FD delta_Hz"));
  Serial.println(F("# P power(0-4)"));
  Serial.println(F("# REF ref_Hz R [NONE|HALF|DOUBLE]"));
  Serial.println(F("# CP current_mA"));
  Serial.println(F("# PD INV|NONINV"));
  Serial.println(F("# CE ON|OFF"));
  Serial.println(F("# SWEEP start_Hz stop_Hz step_Hz dwell_ms [power(0-4)] [LOG|LOGALL]  (any line stops)"));
  Serial.println(F("# CAL [step_MHz] [AUTO|LOCK|TUNE]"));
  Serial.println(F("# VTAB [vco min_MHz max_MHz [LOCK|TUNE]]"));
  Serial.println(F("# VMODE AUTO|TABLE [margin_MHz]"));
  Serial.println(F("# VSCAN freq_Hz [settle_ms [watch_ms]]"));
  Serial.println(F("# VAS ON|OFF"));
  Serial.println(F("# VCO n(0-63)"));
  Serial.println(F("# FRAC ON|OFF"));
  Serial.println(F("# REG  |  REG n(0-5) hex_value  |  REG W"));
  Serial.println(F("# BURST on_us off_us count [AUX]  (count 0 = until any line)"));
  Serial.println(F("# FREQ_DIRECT R INT MOD FRAC outdiv FRACMODE(0|1)"));
}

void SetCE(bool on) {
  CEstate = on;
  digitalWrite(CEpin, on ? HIGH : LOW);
}

// set frequency keeping the current aux output settings
int SetFrequency(long long freq, uint8_t power) {
  int err = vfo.setf(freq, power, vfo.ReadAuxPowerLevel(), (vfo.MAX2870_R[4] >> 9) & 1);
  if (err == MAX2870_ERROR_NONE)
    RequestedFreq = freq;
  return err;
}

void DelayMicros(unsigned long us) {
  if (us <= 16383) {
    delayMicroseconds(us);
  }
  else {
    delay(us / 1000);
    delayMicroseconds(us % 1000);
  }
}

// watch the lock detect pin for durationUs (stops early if a serial line arrives)
void MonitorLock(unsigned long durationUs, LockStats *st) {
  unsigned long t0 = micros();
  unsigned long now = t0;
  unsigned long lowStart = t0;
  bool level = digitalRead(LockPin);
  st->pulses = level ? 0 : 1;
  st->lowTime = 0;
  st->lockTime = 0;
  while ((now = micros()) - t0 < durationUs && Serial.available() == 0) {
    bool l = digitalRead(LockPin);
    if (l != level) {
      if (!l) {
        st->pulses++;
        lowStart = now;
      }
      else {
        st->lowTime += now - lowStart;
        st->lockTime = now - t0;
      }
      level = l;
    }
  }
  if (!level)
    st->lowTime += now - lowStart;
  st->locked = level;
}

int CmdSweep() {
  if (ArgCount < 5 || ArgCount > 7)
    return ERR_BAD_ARGUMENTS;
  long long start = atoll(Arg[1]);
  long long stop = atoll(Arg[2]);
  long long step = atoll(Arg[3]);
  long dwell = atol(Arg[4]);
  uint8_t power = vfo.ReadPowerLevel();
  uint8_t logMode = 0; // 0 none, 1 anomalies, 2 all
  for (int i = 5; i < ArgCount; i++) {
    if (strcmp(Arg[i], "LOG") == 0)
      logMode = 1;
    else if (strcmp(Arg[i], "LOGALL") == 0)
      logMode = 2;
    else if (isdigit(Arg[i][0]))
      power = atoi(Arg[i]);
    else
      return ERR_BAD_ARGUMENTS;
  }
  if (start >= stop || step <= 0 || dwell < 0 || power > 4)
    return ERR_OUT_OF_RANGE;
  int err = SetFrequency(start, power);
  if (err != MAX2870_ERROR_NONE)
    return err;
  Serial.println(F("# sweeping, send any line to stop"));
  while (Serial.available() == 0) {
    for (long long f = start; f <= stop && Serial.available() == 0; f += step) {
      err = vfo.sweepStep(f);
      if (err != MAX2870_ERROR_NONE)
        break;
      RequestedFreq = f;
      LockStats st;
      MonitorLock(dwell*1000UL, &st);
      if (logMode > 0) {
        // short lock detect glitches (tens of us) at the frequency change are normal
        int adc = ReadbackOK ? vfo.ReadTuneADC() : -1;
        // VTUNE outside 0.5-2.5 V (ADC 000 or 111) is also an anomaly
        bool anomaly = !st.locked || st.lowTime > 150 || st.pulses > 3 || adc == 0 || adc == 7;
        if (logMode == 2 || anomaly) {
          Serial.print(F("L f=")); printLL(f);
          Serial.print(F(" n=")); Serial.print(st.pulses);
          Serial.print(F(" low=")); Serial.print(st.lowTime);
          Serial.print(F(" lock=")); Serial.print(st.lockTime);
          Serial.print(F(" vco=")); Serial.print(CurrentVCO());
          Serial.print(F(" adc=")); Serial.print(adc);
          if (!st.locked)
            Serial.print(F(" UNLOCKED"));
          Serial.println();
        }
      }
    }
    if (err != MAX2870_ERROR_NONE)
      break;
  }
  FlushSerialInput();
  PrintStatus();
  return err;
}

int CmdBurst() {
  if (ArgCount < 4 || ArgCount > 5)
    return ERR_BAD_ARGUMENTS;
  unsigned long onTime = atol(Arg[1]);
  unsigned long offTime = atol(Arg[2]);
  unsigned long count = atol(Arg[3]);
  bool aux = (ArgCount == 5 && strcmp(Arg[4], "AUX") == 0);
  if (ArgCount == 5 && !aux)
    return ERR_BAD_ARGUMENTS;

  uint32_t onRegs[MAX2870_RegsToWrite];
  uint32_t offRegs[MAX2870_RegsToWrite];
  vfo.ReadSweepValues(onRegs);
  if (aux)
    vfo.WriteBF_dword(8, 1, vfo.MAX2870_R+4, 0); // aux output off
  else
    vfo.WriteBF_dword(5, 1, vfo.MAX2870_R+4, 0); // main output off
  vfo.ReadSweepValues(offRegs);

  Serial.println(F("# bursting, send any line to stop"));
  while (Serial.available() == 0) {
    vfo.WriteSweepValues(onRegs);
    DelayMicros(onTime);
    vfo.WriteSweepValues(offRegs);
    DelayMicros(offTime);
    if (count > 0 && --count == 0)
      break;
  }
  vfo.WriteSweepValues(onRegs); // restore
  FlushSerialInput();
  PrintStatus();
  return MAX2870_ERROR_NONE;
}

int Execute() {
  const char *cmd = Arg[0];

  if (strcmp(cmd, "S") == 0 || strcmp(cmd, "STATUS") == 0) {
    PrintStatus();
    return MAX2870_ERROR_NONE;
  }

  if (strcmp(cmd, "HELP") == 0) {
    PrintHelp();
    return MAX2870_ERROR_NONE;
  }

  if (strcmp(cmd, "F") == 0 || strcmp(cmd, "FD") == 0) {
    if (ArgCount < 2 || ArgCount > 3 || (cmd[1] == 'D' && ArgCount != 2))
      return ERR_BAD_ARGUMENTS;
    long long freq = atoll(Arg[1]);
    if (cmd[1] == 'D')
      freq += RequestedFreq;
    uint8_t power = (ArgCount == 3) ? atoi(Arg[2]) : vfo.ReadPowerLevel();
    int err = SetFrequency(freq, power);
    if (err == MAX2870_ERROR_NONE) {
      delay(5); // let the VCO autoselect and the loop settle before reading VCO/ADC
      PrintStatus();
    }
    return err;
  }

  if (strcmp(cmd, "P") == 0) {
    if (ArgCount != 2)
      return ERR_BAD_ARGUMENTS;
    int power = atoi(Arg[1]);
    if (power < 0 || power > 4)
      return ERR_OUT_OF_RANGE;
    vfo.setPowerLevel(power);
    PrintStatus();
    return MAX2870_ERROR_NONE;
  }

  if (strcmp(cmd, "REF") == 0) {
    if (ArgCount < 3 || ArgCount > 4)
      return ERR_BAD_ARGUMENTS;
    uint8_t mode = MAX2870_REF_UNDIVIDED;
    if (ArgCount == 4) {
      if (strcmp(Arg[3], "HALF") == 0)
        mode = MAX2870_REF_HALF;
      else if (strcmp(Arg[3], "DOUBLE") == 0)
        mode = MAX2870_REF_DOUBLE;
      else if (strcmp(Arg[3], "NONE") != 0 && strcmp(Arg[3], "UNDIVIDED") != 0)
        return ERR_BAD_ARGUMENTS;
    }
    int err = vfo.setrf(atol(Arg[1]), atoi(Arg[2]), mode);
    if (err == MAX2870_ERROR_NONE && RequestedFreq != 0)
      err = SetFrequency(RequestedFreq, vfo.ReadPowerLevel()); // write new settings to the chip
    PrintStatus();
    return err;
  }

  if (strcmp(cmd, "CP") == 0) {
    if (ArgCount != 2)
      return ERR_BAD_ARGUMENTS;
    vfo.setCPcurrent(atof(Arg[1]));
    PrintStatus();
    return MAX2870_ERROR_NONE;
  }

  if (strcmp(cmd, "PD") == 0) {
    if (ArgCount != 2)
      return ERR_BAD_ARGUMENTS;
    int err;
    if (strcmp(Arg[1], "INV") == 0)
      err = vfo.setPDpolarity(MAX2870_LOOP_TYPE_INVERTING);
    else if (strcmp(Arg[1], "NONINV") == 0)
      err = vfo.setPDpolarity(MAX2870_LOOP_TYPE_NONINVERTING);
    else
      return ERR_BAD_ARGUMENTS;
    PrintStatus();
    return err;
  }

  if (strcmp(cmd, "CE") == 0) {
    if (ArgCount != 2)
      return ERR_BAD_ARGUMENTS;
    if (strcmp(Arg[1], "ON") == 0 || strcmp(Arg[1], "1") == 0)
      SetCE(true);
    else if (strcmp(Arg[1], "OFF") == 0 || strcmp(Arg[1], "0") == 0)
      SetCE(false);
    else
      return ERR_BAD_ARGUMENTS;
    PrintStatus();
    return MAX2870_ERROR_NONE;
  }

  if (strcmp(cmd, "VAS") == 0) {
    if (ArgCount != 2)
      return ERR_BAD_ARGUMENTS;
    if (strcmp(Arg[1], "ON") == 0 || strcmp(Arg[1], "1") == 0) {
      vfo.useVCOTable(false);
      vfo.setVAS(true);
    }
    else if (strcmp(Arg[1], "OFF") == 0 || strcmp(Arg[1], "0") == 0) {
      // keep the current VCO: read back, or the one from the table, else the most centered one in the table
      int vco = CurrentVCO();
      if (vco < 0)
        vco = vfo.selectVCO((uint16_t)((RequestedFreq * vfo.ReadOutDivider() + 500000LL) / 1000000LL), -1);
      if (vco < 0)
        return ERR_NO_VCO_TABLE; // the autoselected VCO cannot be read back
      vfo.useVCOTable(false);
      vfo.setVAS(false, vco);
    }
    else
      return ERR_BAD_ARGUMENTS;
    PrintStatus();
    return MAX2870_ERROR_NONE;
  }

  if (strcmp(cmd, "VCO") == 0) {
    if (ArgCount != 2 || !isdigit(Arg[1][0]))
      return ERR_BAD_ARGUMENTS;
    int vco = atoi(Arg[1]);
    if (vco > 63)
      return ERR_OUT_OF_RANGE;
    vfo.useVCOTable(false);
    vfo.setVAS(false, vco);
    vfo.writeRegister(0); // apply (no autoselect with VAS off)
    delay(5);
    PrintStatus();
    return MAX2870_ERROR_NONE;
  }

  if (strcmp(cmd, "FRAC") == 0) {
    if (ArgCount != 2)
      return ERR_BAD_ARGUMENTS;
    if (strcmp(Arg[1], "ON") == 0 || strcmp(Arg[1], "1") == 0)
      ForceFrac = true;
    else if (strcmp(Arg[1], "OFF") == 0 || strcmp(Arg[1], "0") == 0)
      ForceFrac = false;
    else
      return ERR_BAD_ARGUMENTS;
    vfo.setForceFrac(ForceFrac);
    int err = MAX2870_ERROR_NONE;
    if (RequestedFreq != 0)
      err = SetFrequency(RequestedFreq, vfo.ReadPowerLevel()); // apply the new mode
    delay(5);
    PrintStatus();
    return err;
  }

  if (strcmp(cmd, "FT") == 0) { // FT freq_Hz [watch_ms]: set frequency and measure the lock time
    if (ArgCount < 2 || ArgCount > 3)
      return ERR_BAD_ARGUMENTS;
    long watch = (ArgCount == 3) ? atol(Arg[2]) : 100;
    int err = SetFrequency(atoll(Arg[1]), vfo.ReadPowerLevel());
    if (err != MAX2870_ERROR_NONE)
      return err;
    LockStats st;
    MonitorLock(watch*1000UL, &st);
    Serial.print(F("# FT n=")); Serial.print(st.pulses);
    Serial.print(F(" low=")); Serial.print(st.lowTime);
    Serial.print(F(" lock=")); Serial.print(st.lockTime);
    Serial.println(st.locked ? F("") : F(" UNLOCKED"));
    PrintStatus();
    return MAX2870_ERROR_NONE;
  }

  if (strcmp(cmd, "CAL") == 0) {
    // CAL [step_MHz] [AUTO|LOCK|TUNE]
    if (ArgCount > 3)
      return ERR_BAD_ARGUMENTS;
    uint8_t method = MAX2870_CAL_AUTO;
    int step = 10;
    for (int i = 1; i < ArgCount; i++) {
      if (strcmp(Arg[i], "AUTO") == 0)
        method = MAX2870_CAL_AUTO;
      else if (strcmp(Arg[i], "LOCK") == 0)
        method = MAX2870_CAL_LOCK;
      else if (strcmp(Arg[i], "TUNE") == 0)
        method = MAX2870_CAL_TUNE;
      else if (isdigit(Arg[i][0]))
        step = atoi(Arg[i]);
      else
        return ERR_BAD_ARGUMENTS;
    }
    if (step < 1 || step > 100)
      return ERR_OUT_OF_RANGE;
    Serial.println(F("# VCO calibration, RF outputs muted..."));
    unsigned long t0 = millis();
    int found = vfo.calibrateVCO(step, 1000, 2000, method);
    if (found < 0)
      return method == MAX2870_CAL_TUNE ? ERR_NO_READBACK : ERR_NO_LOCK_PIN;
    Serial.print(F("# ")); Serial.print(TableKindName(vfo.ReadVCOTableKind()));
    Serial.print(F(" table, ")); Serial.print(found); Serial.print(F(" VCOs found in "));
    Serial.print(millis() - t0); Serial.print(F(" ms, "));
    Serial.print(vfo.recoveries); Serial.print(F(" PLL recoveries, "));
    Serial.print(vfo.rechecks); Serial.print(F(" VCOs measured again, "));
    Serial.print(VCOTableGaps()); Serial.println(F(" MHz of 3000-6000 not covered"));
    PrintVCOTable();
    return found > 0 ? MAX2870_ERROR_NONE : ERR_NO_VCO_TABLE;
  }

  if (strcmp(cmd, "VTAB") == 0) {
    if (ArgCount == 4 || ArgCount == 5) { // VTAB vco min max [LOCK|TUNE]
      int v = atoi(Arg[1]);
      long lo = atol(Arg[2]), hi = atol(Arg[3]);
      if (v < 0 || v > 63 || lo < 0 || hi < lo || hi > 65535)
        return ERR_OUT_OF_RANGE;
      if (ArgCount == 5) {
        if (strcmp(Arg[4], "TUNE") == 0)
          vfo.setVCOTableKind(MAX2870_CAL_TUNE);
        else if (strcmp(Arg[4], "LOCK") == 0)
          vfo.setVCOTableKind(MAX2870_CAL_LOCK);
        else
          return ERR_BAD_ARGUMENTS;
      }
      vfo.setVCOTable(v, lo, hi);
      PrintVCOTableEntry(v, lo, hi);
      return MAX2870_ERROR_NONE;
    }
    if (ArgCount != 1)
      return ERR_BAD_ARGUMENTS;
    PrintVCOTable();
    return MAX2870_ERROR_NONE;
  }

  if (strcmp(cmd, "VMODE") == 0) {
    if (ArgCount < 2 || ArgCount > 3)
      return ERR_BAD_ARGUMENTS;
    int margin = (ArgCount == 3) ? atoi(Arg[2]) : -1; // -1: default for the table kind
    if (margin < -1 || margin > 255)
      return ERR_OUT_OF_RANGE;
    if (strcmp(Arg[1], "TABLE") == 0) {
      if (!vfo.VCOTableValid())
        return ERR_NO_VCO_TABLE;
      vfo.useVCOTable(true, margin);
    }
    else if (strcmp(Arg[1], "AUTO") == 0) {
      vfo.useVCOTable(false);
      vfo.setVAS(true);
    }
    else
      return ERR_BAD_ARGUMENTS;
    int err = MAX2870_ERROR_NONE;
    if (RequestedFreq != 0)
      err = SetFrequency(RequestedFreq, vfo.ReadPowerLevel()); // apply the new mode
    delay(5);
    PrintStatus();
    return err;
  }

  if (strcmp(cmd, "VSCAN") == 0) { // VSCAN freq_Hz [settle_ms [watch_ms]]: lock test of every manual VCO
    if (ArgCount < 2 || ArgCount > 4)
      return ERR_BAD_ARGUMENTS;
    long long freq = atoll(Arg[1]);
    int settle = (ArgCount >= 3) ? atoi(Arg[2]) : 2;
    int watch = (ArgCount >= 4) ? atoi(Arg[3]) : 5;
    bool tableUse = vfo.ReadUseVCOTable();
    vfo.useVCOTable(false);
    vfo.setVAS(true);
    int err = SetFrequency(freq, vfo.ReadPowerLevel());
    if (err != MAX2870_ERROR_NONE)
      return err;
    for (int v = 0; v < 64 && Serial.available() == 0; v++) {
      vfo.setVAS(false, v);
      vfo.writeRegister(0);
      delay(settle);
      LockStats st;
      MonitorLock(watch*1000UL, &st);
      Serial.print(F("V f=")); printLL(freq);
      Serial.print(F(" vco=")); Serial.print(v);
      Serial.print(F(" n=")); Serial.print(st.pulses);
      Serial.print(F(" low=")); Serial.print(st.lowTime);
      Serial.println(st.locked && st.pulses == 0 ? F(" LOCK") : F(""));
    }
    vfo.setVAS(true);
    vfo.useVCOTable(tableUse);
    SetFrequency(freq, vfo.ReadPowerLevel());
    FlushSerialInput();
    return MAX2870_ERROR_NONE;
  }

  if (strcmp(cmd, "REG") == 0) {
    if (ArgCount == 2 && strcmp(Arg[1], "W") == 0) {
      vfo.WriteRegs();
    }
    else if (ArgCount == 3) {
      if (!isdigit(Arg[1][0]))
        return ERR_BAD_ARGUMENTS;
      int n = atoi(Arg[1]);
      if (n > 5)
        return ERR_OUT_OF_RANGE;
      char *end;
      uint32_t value = strtoul(Arg[2], &end, 16);
      if (*end != 0)
        return ERR_BAD_ARGUMENTS;
      vfo.MAX2870_R[n] = (value & ~7UL) | n; // address bits forced
      vfo.writeRegister(n);
    }
    else if (ArgCount != 1) {
      return ERR_BAD_ARGUMENTS;
    }
    delay(5);
    PrintRegisters();
    return MAX2870_ERROR_NONE;
  }

  if (strcmp(cmd, "SWEEP") == 0)
    return CmdSweep();

  if (strcmp(cmd, "BURST") == 0)
    return CmdBurst();

  if (strcmp(cmd, "FREQ_DIRECT") == 0) {
    if (ArgCount != 7)
      return ERR_BAD_ARGUMENTS;
    bool fracMode = (strcmp(Arg[6], "1") == 0 || strcmp(Arg[6], "TRUE") == 0);
    vfo.setfDirect(atoi(Arg[1]), atol(Arg[2]), atoi(Arg[3]), atoi(Arg[4]), atoi(Arg[5]), fracMode);
    PrintStatus();
    return MAX2870_ERROR_NONE;
  }

  return ERR_UNKNOWN_COMMAND;
}

// split Command in upper case fields
void Tokenize() {
  for (char *p = Command; *p; p++)
    *p = toupper(*p);
  ArgCount = 0;
  char *tok = strtok(Command, " \t");
  while (tok != NULL && ArgCount < MaxArgs) {
    Arg[ArgCount++] = tok;
    tok = strtok(NULL, " \t");
  }
}

void setup() {
  Serial.begin(SerialPortRate);
  vfo.init(SSpin, LockPin, true, CEpin, true);
  SetCE(true); // enable the MAX2870

  // initial setting: 100 MHz reference, fPFD = 10 MHz, 3 GHz
  delay(100);
  vfo.setrf(100000000UL, 5, MAX2870_REF_HALF);
  vfo.setCPcurrent(5.12);
  ReadbackOK = vfo.checkReadback(); // MUX_OUT connected to MISO: register 6 readback
  Serial.println(ReadbackOK ? F("# register 6 readback OK") : F("# register 6 readback not available"));
  SetFrequency(3000000000LL, 2); // -1 dBm: at -4 dBm some VCOs were noisy or off frequency on the test board
  delay(5);

  Serial.println(F("# MAX2870 ready, HELP for commands"));
  PrintStatus();
}

void loop() {
  static int length = 0;
  static bool overflow = false;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      Command[length] = 0;
      if (overflow) {
        Reply(ERR_LINE_TOO_LONG);
      }
      else {
        Tokenize();
        if (ArgCount > 0)  // empty lines are ignored
          Reply(Execute());
      }
      length = 0;
      overflow = false;
    }
    else if (length < CommandSize-1) {
      Command[length++] = c;
    }
    else {
      overflow = true;
    }
  }
}
