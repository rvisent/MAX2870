/*

  MAX2870 demo by Bryce Cherry

  Commands:
  REF reference_frequency_in_Hz reference_divider reference_multiplier(UNDIVIDED/DOUBLE/HALF) - Set reference frequency, reference divider and reference doubler/divide by 2
  F frequency_in_Hz power_level(0-4) attenuation (0-15)
  FD frequency_delta_Hz [power_level(0-4) attenuation (0-15)]
  FREQ_DIRECT R_divider INT_value MOD_value FRAC_value RF_DIVIDER_value PRESCALER_value FRACTIONAL_MODE(true/false) - sets RF parameters directly
  (BURST/BURST_CONT/BURST_SINGLE) on_time_in_uS off time_in_uS count (AUX) - perform a on/off burst on frequency and power level set with FREQ/FREQ_P - count is only used with BURST_CONT - if AUX is used, will burst on the auxiliary output; otherwise, it will burst on the primary output
  SWEEP start_frequency stop_frequency step_in_mS(1-32767) [power_level(1-4) [aux_power_level(0-4) aux_frequency_output(DIVIDED/FUNDAMENTAL)]] - sweep RF frequency
  STEP frequency_in_Hz [step_in_mS(1-32767)] - set sweep step and time per sample
  STATUS - view status of VFO
  CE (ON/OFF) - enable/disable MAX2870
  CP_CURRENT current_in_mA_floating - adjust charge pump current to suit your loop filter (default library value is 2.56 mA)
  PD_POLARITY (INVERTING/NONINVERTING) - change phase detector polarity (default library is noninverting for passive/noninverting loop filters)

*/

#include "MAX2870.h"

//#define EXTERNAL_ATTENUATOR // define only if HMC540 0-15 dB attenuator is fitted

MAX2870 vfo;

// use hardware SPI pins for Data and Clock (+MUXOUT?)
const byte SSpin = 48; // LE
const byte LockPin = 50; // MISO no
const byte CEpin = 52;
#ifdef EXTERNAL_ATTENUATOR
const byte Att0 = 40; // ext att HMC540 DO 40-43 0000 max attenuation 15 dB
#endif

const int CommandSize = 50;
char Command[CommandSize];
#ifdef EXTERNAL_ATTENUATOR
int8_t ExtAtt;
#endif
uint32_t sweep_fStep = 1000000L;  // default 1 MHz
uint16_t sweep_tStep = 100;       // default 100 ms

// ensures that the serial port is flushed fully on request
const unsigned long SerialPortRate = 115200;
const byte SerialPortRateTolerance = 5; // percent - increase to 50 for rates above 115200 up to 4000000
const byte SerialPortBits = 10; // start (1), data (8), stop (1)
const unsigned long TimePerByte = ((((1000000ULL * SerialPortBits) / SerialPortRate) * (100 + SerialPortRateTolerance)) / 100); // calculated on serial port rate + tolerance and rounded down to the nearest uS, long caters for even the slowest serial port of 75 bps

void FlushSerialBuffer() {
  while (true) {
    if (Serial.available() > 0) {
      byte dummy = Serial.read();
      while (Serial.available() > 0) { // flush additional bytes from serial buffer if present
        dummy = Serial.read();
      }
      if (TimePerByte <= 16383) {
        delayMicroseconds(TimePerByte); // delay in case another byte may be received via the serial port
      }
      else { // deal with delayMicroseconds limitation
        unsigned long DelayTime = TimePerByte;
        DelayTime /= 1000;
        if (DelayTime > 0) {
          delay(DelayTime);
        }
        DelayTime = TimePerByte;
        DelayTime %= 1000;
        if (DelayTime > 0) {
          delayMicroseconds(DelayTime);
        }
      }
    }
    else {
      break;
    }
  }
}

bool getField (char* buffer, int index) {
  int CommandPos = 0;
  int FieldPos = 0;
  int SpaceCount = 0;
  bool found = false;
  while (CommandPos < CommandSize) {
    if (Command[CommandPos] == ' ') {
      SpaceCount++;
      CommandPos++;
    }
    char c = Command[CommandPos];
    if (!c || c == 0x0D || c == 0x0A) {
      break;
    }
    if (SpaceCount == index) {
      buffer[FieldPos] = c;
      FieldPos++;
      found = true;
    }
    CommandPos++;
  }
  for (int ch = 0; ch < strlen(buffer); ch++) { // correct case of command
    buffer[ch] = toupper(buffer[ch]);
  }
  buffer[FieldPos] = '\0';
  return found;
}

char* lltoa( long long value, char *string, int radix)
{
  char tmp[33];
  char *tp = tmp;
  long i;
  int sign;
  char *sp;

  if ( string == NULL )
  {
    return 0 ;
  }

  if (radix > 36 || radix <= 1)
  {
    return 0 ;
  }

  sign = (radix == 10 && value < 0);
  if (sign)
  {
    value = -value;
  }

  while (value || tp == tmp)
  {
    i = value % radix;
    value /= radix;
    if (i < 10)
      *tp++ = i+'0';
    else
      *tp++ = i + 'a' - 10;
  }

  sp = string;

  if (sign)
    *sp++ = '-';
  while (tp > tmp)
    *sp++ = *--tp;
  *sp = 0;

  return string;
}

void PrintVFOstatus() {
  if (vfo.ReadRDIV2() != 0 && vfo.ReadRefDoubler() != 0) {
    Serial.println(F("Reference doubler and reference divide by 2 enabled - invalid state"));
  }
  else if (vfo.ReadRDIV2() != 0) {
    Serial.print(F("REF/2"));
  }
  else if (vfo.ReadRefDoubler() != 0) {
    Serial.print(F("REF*2"));
  }
  else {
    Serial.print(F("REF_NO*/"));
  }
  Serial.print(F("  R:")); Serial.print(vfo.ReadR());
  Serial.print(F("  INT:")); Serial.print(vfo.ReadInt());
  Serial.print(F("  FRAC/MOD:")); Serial.print(vfo.ReadFraction());
  Serial.print(F("/")); Serial.print(vfo.ReadMod());
  Serial.print(F("  OutDIV:")); Serial.print(vfo.ReadOutDivider());
  Serial.print(F(" (2^")); Serial.print(vfo.ReadOutDivider_PowerOf2()); Serial.println(F(")"));
  Serial.print(F("fPFD_Hz:")); Serial.print(vfo.ReadPFDfreq());
  Serial.print(F("  PWR A:")); Serial.print(vfo.ReadPowerLevel());
  Serial.print(F(" B:")); Serial.print(vfo.ReadAuxPowerLevel());
#ifdef EXTERNAL_ATTENUATOR
  Serial.print(F(" ExtAtt:")); Serial.println(ExtAtt);
#endif
  char CurrentFreq[MAX2870_ReadCurrentFrequency_ArraySize];
  Serial.print(F("fOUT_Hz:")); Serial.print(lltoa(vfo.ReadCurrentFrequency(),CurrentFreq,10));
  Serial.print(F("  fERR_Hz:")); Serial.print(vfo.MAX2870_FrequencyError);
  Serial.print(F("  fStep_Hz:")); Serial.println(sweep_fStep);
  Serial.print(F("  tStep_ms:")); Serial.println(sweep_tStep);
}

void PrintErrorCode(byte value) {
  switch (value) {
    case MAX2870_ERROR_NONE:
      break;
    case MAX2870_ERROR_STEP_FREQUENCY_EXCEEDS_PFD:
      Serial.println(F("Step frequency exceeds PFD frequency"));
      break;
    case MAX2870_ERROR_RF_FREQUENCY:
      Serial.println(F("RF frequency out of range"));
      break;
    case MAX2870_ERROR_POWER_LEVEL:
      Serial.println(F("Power level incorrect"));
      break;
    case MAX2870_ERROR_AUX_POWER_LEVEL:
      Serial.println(F("Auxiliary power level incorrect"));
      break;
    case MAX2870_ERROR_AUX_FREQ_DIVIDER:
      Serial.println(F("Auxiliary frequency divider incorrect"));
      break;
    case MAX2870_ERROR_ZERO_PFD_FREQUENCY:
      Serial.println(F("PFD frequency is zero"));
      break;
    case MAX2870_ERROR_MOD_RANGE:
      Serial.println(F("Mod is out of range"));
      break;
    case MAX2870_ERROR_FRAC_RANGE:
      Serial.println(F("Fraction is out of range"));
      break;
    case MAX2870_ERROR_N_RANGE:
      Serial.println(F("N is of range"));
      break;
    case MAX2870_ERROR_N_RANGE_FRAC:
      Serial.println(F("N is out of range under fractional mode"));
      break;
    case MAX2870_ERROR_RF_FREQUENCY_AND_STEP_FREQUENCY_HAS_REMAINDER:
      Serial.println(F("RF frequency and step frequency division has remainder"));
      break;
    case MAX2870_ERROR_PFD_EXCEEDED_WITH_FRACTIONAL_MODE:
      Serial.println(F("PFD exceeds 50 MHz under fractional mode"));
      break;
    case MAX2870_ERROR_PRECISION_FREQUENCY_CALCULATION_TIMEOUT:
      Serial.println(F("Precision frequency calculation timeout"));
      break;
//    case MAX2870_WARNING_FREQUENCY_ERROR:
//      Serial.println(F("Actual frequency is different than desired"));
//      break;
    case MAX2870_ERROR_DOUBLER_EXCEEDED:
      Serial.println(F("Reference frequency with doubler exceeded"));
      break;
    case MAX2870_ERROR_R_RANGE:
      Serial.println(F("R divider is out of range"));
      break;
    case MAX2870_ERROR_REF_FREQUENCY:
      Serial.println(F("Reference frequency is out of range"));
      break;
    case MAX2870_ERROR_REF_MULTIPLIER_TYPE:
      Serial.println(F("Reference multiplier type is incorrect"));
      break;
    case MAX2870_ERROR_PFD_AND_STEP_FREQUENCY_HAS_REMAINDER:
      Serial.println(F("PFD and step frequency division has remainder"));
      break;
    case MAX2870_ERROR_PFD_LIMITS:
      Serial.println(F("PFD frequency is out of range"));
      break;
  }
}

void setup() {
  Serial.begin(SerialPortRate);
  vfo.init(SSpin, LockPin, true, CEpin, true);
  digitalWrite(CEpin, HIGH); // enable the MAX2870

  // initial setting @@@@@@ TEST RV
  delay(100);
  vfo.setrf(100000000UL, 5, MAX2870_REF_HALF);
  vfo.setCPcurrent(5.12);
  vfo.setf(3000000000LL, 1); // 3 GHz

#ifdef EXTERNAL_ATTENUATOR
  // external attenuator
  for (int i=0; i<4; i++)
  {
    pinMode(Att0+i, OUTPUT);
    // set max att
    digitalWrite(Att0+i, LOW);
  }
  ExtAtt = 15;
#endif
}

void loop() {
  static int ByteCount = 0;
  if (Serial.available() > 0) {
    char value = Serial.read();
    if (value != '\n' && ByteCount < CommandSize) {
      Command[ByteCount] = value;
      ByteCount++;
    }
    else {
      if (ByteCount < CommandSize)
        Command[ByteCount] = 0;
      else
        Command[CommandSize-1] = 0;
      Serial.println();
      Serial.print("RECEIVED: '");
      Serial.print(Command);
      Serial.println("'");
      ByteCount = 0;
      bool ValidField = true;
      char field[20];
      getField(field, 0);
      if (strcmp(field, "REF") == 0) {
        getField(field, 1);
        unsigned long ReferenceFreq = atol(field);
        getField(field, 2);
        word ReferenceDivider = atoi(field);
        getField(field, 3);
        byte ReferenceHalfDouble = MAX2870_REF_UNDIVIDED;
        if (strcmp(field, "DOUBLE") == 0) {
          ReferenceHalfDouble = MAX2870_REF_DOUBLE;
        }
        else if (strcmp(field, "HALF") == 0) {
          ReferenceHalfDouble = MAX2870_REF_HALF;
        }
        byte ErrorCode = vfo.setrf(ReferenceFreq, ReferenceDivider, ReferenceHalfDouble);
        if (ErrorCode != MAX2870_ERROR_NONE) {
          ValidField = false;
          PrintErrorCode(ErrorCode);
        }
      }
      else if (strcmp(field, "F") == 0 || strcmp(field, "FD") == 0) {
        bool FreqDeltaRequired = false;
        if (strcmp(field, "FD") == 0) {
          FreqDeltaRequired = true;
        }
        
        byte PowerLevel;
        if (getField(field, 2))
          PowerLevel = atoi(field);
        else
          PowerLevel = vfo.ReadPowerLevel();
      
        /*
        getField(field, 3);
        byte AuxPowerLevel = atoi(field);
        getField(field, 4);
        byte AuxFrequencyDivider;
        if (strcmp(field, "DIVIDED") == 0) {
          AuxFrequencyDivider = MAX2870_AUX_DIVIDED;
        }
        else if (strcmp(field, "FUNDAMENTAL") == 0) {
          AuxFrequencyDivider = MAX2870_AUX_FUNDAMENTAL;
        }
        else {
          ValidField = false;
        }
        unsigned long FrequencyTolerance = 0;
        if (PrecisionRequired == true) {
          getField(field, 5);
          FrequencyTolerance = atol(field);
        }
        getField(field, 6);
        unsigned long CalculationTimeout = atol(field);
        unsigned long FrequencyWriteTimeStart = millis();
        */
        if (ValidField == true) {
          getField(field, 1);
          long long freq = atoll(field);
          if (FreqDeltaRequired)
            freq += vfo.ReadCurrentFrequency();

          byte ErrorCode = vfo.setf(freq, PowerLevel); //, AuxPowerLevel, AuxFrequencyDivider, PrecisionRequired, FrequencyTolerance, CalculationTimeout);
          if (ErrorCode != MAX2870_ERROR_NONE)
            ValidField = false;
#ifdef EXTERNAL_ATTENUATOR
          else
          {
            // set external attenuator
            if (getField(field, 3))
            {
              int8_t tempExtAtt = atoi(field);
              if (tempExtAtt >= 0 && tempExtAtt < 16)
              {
                ExtAtt = tempExtAtt;
                for (int i=0; i<4; i++)
                {
                  digitalWrite(Att0+i, !(tempExtAtt & 1));
                  tempExtAtt >>= 1;
                }
              }
              else
                Serial.println("BAD EXT ATT");
            }
          }
#endif
          PrintErrorCode(ErrorCode);
          if (ValidField == true) {
            /*
            unsigned long FrequencyWriteTime = millis();
            FrequencyWriteTime -= FrequencyWriteTimeStart;
            Serial.print(F("Time measured during setf() with CPU speed of "));
            Serial.print((F_CPU / 1000000UL));
            Serial.print(F("."));
            Serial.print((F_CPU % 1000000UL));
            Serial.print(F(" MHz: "));
            Serial.print((FrequencyWriteTime / 1000));
            Serial.print(F("."));
            Serial.print((FrequencyWriteTime % 1000));
            Serial.println(F(" seconds"));
            */
            PrintVFOstatus();
          }
        }
      }
      else if (strcmp(field, "FREQ_DIRECT") == 0) {
        getField(field, 1);
        word R_divider = atoi(field);
        getField(field, 2);
        word INT_value = atol(field);
        getField(field, 3);
        word MOD_value = atoi(field);
        getField(field, 4);
        word FRAC_value = atoi(field);
        getField(field, 5);
        word RF_DIVIDER_value = atoi(field);
        getField(field, 6);
        if (strcmp(field, "TRUE") == 0 || strcmp(field, "FALSE") == 0) {
          bool FRACTIONAL_MODE = false;
          if (strcmp(field, "TRUE") == 0) {
            FRACTIONAL_MODE = true;
          }
          vfo.setfDirect(R_divider, INT_value, MOD_value, FRAC_value, RF_DIVIDER_value, FRACTIONAL_MODE);
        }
        else {
          ValidField = false;
        }
      }
      else if (strcmp(field, "BURST") == 0 || strcmp(field, "BURST_CONT") == 0 || strcmp(field, "BURST_SINGLE") == 0) {
        bool ContinuousBurst = false;
        bool SingleBurst = false;
        unsigned long BurstCount;
        if (strcmp(field, "BURST_CONT") == 0) {
          ContinuousBurst = true;
        }
        else if (strcmp(field, "BURST_SINGLE") == 0) {
          SingleBurst = true;
        }
        bool AuxOutput = false;
        getField(field, 1);
        unsigned long BurstOnTime = atol(field);
        getField(field, 2);
        unsigned long BurstOffTime = atol(field);
        getField(field, 3);
        if (strcmp(field, "AUX") == 0) {
          AuxOutput = true;
        }
        else if (ContinuousBurst == false && SingleBurst == false) {
          BurstCount = atol(field);
          getField(field, 4);
          if (strcmp(field, "AUX") == 0) {
            AuxOutput = true;
          }
        }
        unsigned long OnBurstData[MAX2870_RegsToWrite];
        vfo.ReadSweepValues(OnBurstData);
        if (AuxOutput == false) {
          vfo.setPowerLevel(0);
        }
        else {
          vfo.setAuxPowerLevel(0);
        }
        unsigned long OffBurstData[MAX2870_RegsToWrite];
        vfo.ReadSweepValues(OffBurstData);
        Serial.print(F("Burst "));
        Serial.print((BurstOnTime / 1000));
        Serial.print(F("."));
        Serial.print((BurstOnTime % 1000));
        Serial.print(F(" mS on, "));
        Serial.print((BurstOffTime / 1000));
        Serial.print(F("."));
        Serial.print((BurstOffTime % 1000));
        Serial.println(F(" mS off"));
        if (SingleBurst == true) {
          vfo.WriteSweepValues(OffBurstData);
          if (BurstOffTime <= 16383) {
            delayMicroseconds(BurstOffTime);
          }
          else {
            delay((BurstOffTime / 1000));
            delayMicroseconds((BurstOffTime % 1000));
          }
        }
        if (ContinuousBurst == false && SingleBurst == false && BurstCount == 0) {
          ValidField = false;
        }
        if (ValidField == true) {
          FlushSerialBuffer();
          while (true) {
            vfo.WriteSweepValues(OnBurstData);
            if (BurstOnTime <= 16383) {
              delayMicroseconds(BurstOnTime);
            }
            else {
              delay((BurstOnTime / 1000));
              delayMicroseconds((BurstOnTime % 1000));
            }
            vfo.WriteSweepValues(OffBurstData);
            if (ContinuousBurst == false && SingleBurst == false) {
              BurstCount--;
            }
            if ((ContinuousBurst == false && BurstCount == 0) || SingleBurst == true || Serial.available() > 0) {
              for (int i = 0; i < MAX2870_RegsToWrite; i++) {
                vfo.MAX2870_R[i] = OnBurstData[i];
              }
              Serial.println(F("End of burst"));
              break;
            }
            if (BurstOffTime <= 16383) {
              delayMicroseconds(BurstOffTime);
            }
            else {
              delay((BurstOffTime / 1000));
              delayMicroseconds((BurstOffTime % 1000));
            }
          }
        }
      }
      
      else if (strcmp(field, "SWEEP") == 0) {
        getField(field, 1);
        long long StartFrequency = atoll(field);
        getField(field, 2);
        long long StopFrequency = atoll(field);
        byte PowerLevel, AuxPowerLevel = 0, AuxFrequencyDivider = MAX2870_AUX_DIVIDED;
        if (getField(field, 3)) {
          PowerLevel = atoi(field);

          if (getField(field, 4))
            AuxPowerLevel = atoi(field);
          if (getField(field, 5) && !strcmp(field, "FUNDAMENTAL"))
            AuxFrequencyDivider = MAX2870_AUX_FUNDAMENTAL;
        }
        else
          PowerLevel = vfo.ReadPowerLevel(); // default: no change current value

        if (StartFrequency >= StopFrequency || sweep_fStep == 0 || sweep_tStep == 0) {
          if (sweep_fStep == 0 || sweep_tStep == 0)
            Serial.println(F("Please use command STEP before SWEEP"));
          else
            Serial.println(F("Stop frequency must be greater than start frequency"));
        }
        else {
          Serial.println(F("Now sweeping"));
          FlushSerialBuffer();
          while (true) {
            if (Serial.available() > 0) {
              break;
            }
            long long f = StartFrequency;
            if (vfo.setf(f, PowerLevel, AuxPowerLevel, AuxFrequencyDivider) != MAX2870_ERROR_NONE) {
              Serial.println(F("Bad initial frequency"));
              break;
            }
            for (; f <= StopFrequency; f += sweep_fStep) {
              if (Serial.available() > 0) {
                break;
              }
              delay(sweep_tStep);
              vfo.sweepStep(f);
            }
            Serial.print(F("*"));
          }
          Serial.println();
          Serial.println(F("End of sweep"));
        }
      }
      
      else if (strcmp(field, "STEP") == 0) {
        getField(field, 1);
        sweep_fStep = atol(field);
        if (getField(field, 2))
          sweep_tStep = atol(field);
      }
      else if (strcmp(field, "STATUS") == 0) {
        PrintVFOstatus();
        SPI.end();
        if (digitalRead(LockPin) == LOW) {
          Serial.println(F("Lock pin LOW"));
        }
        else {
          Serial.println(F("Lock pin HIGH"));
        }
        SPI.begin();
      }
      else if (strcmp(field, "CE") == 0) {
        getField(field, 1);
        if (strcmp(field, "ON") == 0) {
          digitalWrite(CEpin, HIGH);
        }
        else if (strcmp(field, "OFF") == 0) {
          digitalWrite(CEpin, LOW);
        }
        else {
          ValidField = false;
        }
      }
      else if (strcmp(field, "CP_CURRENT") == 0) {
        getField(field, 1);
        float ChargePumpCurrent = atof(field);
        vfo.setCPcurrent(ChargePumpCurrent);
      }
      else if (strcmp(field, "PD_POLARITY") == 0) {
        getField(field, 1);
        if (strcmp(field, "INVERTING") == 0) {
          vfo.setPDpolarity(MAX2870_LOOP_TYPE_INVERTING);
        }
        else if (strcmp(field, "NONINVERTING") == 0) {
          vfo.setPDpolarity(MAX2870_LOOP_TYPE_NONINVERTING);
        }
        else {
          ValidField = false;
        }
      }
      else {
        ValidField = false;
      }
      FlushSerialBuffer();
      if (ValidField == true) {
        Serial.println(F("OK"));
      }
      else {
        Serial.println(F("ERROR"));
      }
    }
  }
}
