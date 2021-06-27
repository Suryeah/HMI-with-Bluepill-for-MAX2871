#include <SPI.h>
#include <Wire.h>

// PINS MAX2871
#define CLOCKPIN PA1
#define DATAPIN PA2
#define LE PA3
//#define CE PA4                //Directely Shorted on Board

#define SUPPLY 7  //Oscillator ON 
#define LED PC13

//composition of MAX2871 Registers
//Register 0
unsigned long INT = 0x0;      //Enables fractional-N mode
unsigned long NDIV = 0x0;     //Integer part from N-Divider
unsigned long FRAC = 0x0;
unsigned long ADDR0 = 0x0;

//Register 1
unsigned long FUO = 0x1;
unsigned long CPL = 0x0;      //Charge pump liniarity 30%
unsigned long CPT = 0x00;     //Charge pump test mode = normal mode
unsigned long PHASE = 0x1;    //Phase Value (recomened)
unsigned long MODULUS = 0x7D; //4000 for max resolution
unsigned long ADDR1 = 0x1;

//Register 2
unsigned long LDS = 0x0;      //1 if fPFD > 32 MHz
unsigned long SDN = 0x0;      //noise mode = Low-noise mode
unsigned long MUX = 0x3;      //MUX pin configuration = Digital lock detect
unsigned long DBR = 0x0;      //reference doubler is disabled
unsigned long RDIV2 = 0x0;    //reference divide-by-2 is disabled
unsigned long RCNT = 0x02;    //reference divide Value is unused
unsigned long REG4DB = 0x0;   //double buffer mode disabled
unsigned long CP = 0xF;       //charge pump current = 0.32 mA (1.36/RSET * (1 + CP[3:0]) RSET = 5k1)
unsigned long LDF = 0x1;      //lock dtect function = Frac-N lock detect
unsigned long LDP = 0x0;      //lock detect precision = 10ns
unsigned long PDP = 0x1;      //phase detector polarity set poitive
unsigned long SHDN = 0x0;     //power down mode = normal mode
unsigned long TRI = 0x0;      //charge pump output high-impedance mode disabled
unsigned long RST = 0x0;      // counter reset mode = normal operation
unsigned long ADDR2 = 0x2;

//Register 3
unsigned long VCO_MS = 0x0;   // VCO maual selction: unused
unsigned long VAS_SHDN = 0x0; //VAS enabled
unsigned long VAS_TEMP = 0x0; //VAS temperature compensation enabled
unsigned long CSM = 0x0;      //Cycle slip mode disabled
unsigned long MUTEDEL = 0x0;  //mute delay mode disabled
unsigned long CDM = 0x1;      // Fast-lock mode enabled
unsigned long CDIV = 0xE46;   // clock divider value unused
unsigned long ADDR3 = 0x3;

//Register 4
unsigned long RES = 0x3;      //Reserved
unsigned long SDLDO = 0x0;    //LDO endabled
unsigned long SDDIV = 0x0;    //VCO Divider enabled
unsigned long SDREF = 0x0;    //Reference input enabled
unsigned long FB = 0x0;       //VCO to N counter mode is NOT divided
unsigned long DIVA = 0x01;    //Division Factor for frequencies < 3GHz
unsigned long BS = 0x64;      //shoud be choosen so that fPFD/BS = 50kH or less
unsigned long SDVCO = 0x0;    //VCO enabled
unsigned long MTLD = 0x0;     //RFOUT Mute until Lock detet mode disabled
unsigned long BDIV = 0x0;     //RFOUTB is divided (so it's the same as RFOUTA)
unsigned long RFB_EN = 0x0;   //RFOUTB enabled
unsigned long BPWR = 0x0;     //RFOUTB = 5 dBm
unsigned long RFA_EN = 0x0;   //RFOUTA enabled
unsigned long APWR = 0x2;     //RFOUTA = 5dBm
unsigned long ADDR4 = 0x4;

//Register 5
unsigned long VAS_DLY = 0x0;    //0x0 if VAS_TEMP = 0, 0x3 if VAS_TEMP = 1
unsigned long SDPLL = 0x0;      //PLL enabled
unsigned long F01 = 0x0;        //if F = 0 then int
unsigned long LD = 0x1;         //Lock-Detect pin function = HIGH
unsigned long MUX_MSB = 0x0;    //MSB of MUX
unsigned long ADCS = 0x0;       //ADC normal operation (ADC isn't used)
unsigned long ADCM = 0x0;       //ADC disabled
unsigned long ADDR5 = 0x5;

unsigned long long FreqOUT = 2000000000;
unsigned long long FMIN = 100000;
unsigned long long FMAX = 6000000001;
unsigned long long composedRegisterValue;

String txtMsg = "";
int ASF;

void CalculateRegisterValues(void);
void WriteMAX2871( unsigned long);
void blinkL(void);
void Stop(void);
void mono_Tone(void);
void ProgramMAX2871(void);


void setup()
{
  Serial3.begin(115200);
  Serial.begin(115200);           // Initializing Serail communication.

  pinMode(CLOCKPIN, OUTPUT);
  pinMode(DATAPIN, OUTPUT);
  pinMode(LE, OUTPUT);
  // pinMode(CE, OUTPUT);         //Directely Shorted on Board
  pinMode(SUPPLY, OUTPUT);

  digitalWrite(CLOCKPIN, LOW);
  digitalWrite(DATAPIN, LOW);
  digitalWrite(LE, HIGH);
  // digitalWrite(CE, HIGH);      //Directely Shorted on Board
  digitalWrite(SUPPLY, HIGH);

  CalculateRegisterValues();

  //init MAX2871 2* (see datasheet)
  for (int i = 0; i < 2; i++)
  {
    composedRegisterValue = VAS_DLY << 29 | SDPLL << 25
                            | F01 << 24 | LD << 22
                            | MUX_MSB << 18 | ADCS << 6
                            | ADCM << 3 | ADDR5;

    Serial.print("REG 5 :");
    WriteMAX2871(composedRegisterValue);
    delay(20);

    composedRegisterValue = RES << 29 | SDLDO << 28
                            | SDDIV << 27 | SDREF << 26 | FB << 23
                            | DIVA << 20 | BS << 12 | SDVCO << 11 | MTLD << 10
                            | BDIV << 9 | RFB_EN << 8 | BPWR << 6 | RFA_EN << 5
                            | APWR << 3 | ADDR4;

    Serial.print("REG 4 :");
    WriteMAX2871(composedRegisterValue);

    composedRegisterValue = VCO_MS << 26 | VAS_SHDN << 25
                            | VAS_TEMP << 24 | CSM << 18
                            | MUTEDEL << 17 | CDM << 15 | CDIV << 3 | ADDR3;

    Serial.print("REG 3 :");
    WriteMAX2871(composedRegisterValue);

    composedRegisterValue = LDS << 31 | SDN << 29 | MUX << 26
                            | DBR << 25 | RDIV2 << 24 | RCNT << 14
                            | REG4DB << 13 | CP << 9 | LDF << 8 | LDP << 7
                            | PDP << 6 | SHDN << 5 | TRI << 4 | RST << 3 | ADDR2;

    Serial.print("REG 2 :");
    WriteMAX2871(composedRegisterValue);

    composedRegisterValue = FUO << 30 | CPL << 29 | CPT << 27
                            | PHASE << 15 | MODULUS << 3 | ADDR1;

    Serial.print("REG 1 :");
    WriteMAX2871(composedRegisterValue);

    composedRegisterValue = INT << 31 | NDIV << 15
                            | FRAC << 3 | ADDR0;

    Serial.print("REG 0 :");
    WriteMAX2871(composedRegisterValue);
  }
  RFA_EN = 0x01;
  ProgramMAX2871();
}

void loop()
{
  while (Serial3.available() > 0)
  {
    txtMsg = Serial3.readStringUntil('\n');
    Serial.println(txtMsg);

    switch (txtMsg[0])
    {
      case 'a':               // MONO Tone
        {
          FreqOUT = 0;
          FreqOUT = txtMsg.substring(2, 11).toInt();
          ASF = txtMsg.substring(12, 16).toInt();
          txtMsg = "";
          mono_Tone();
        } break;

      case 's':
        {
          Stop();
        } break;

      default:
        txtMsg = "";
    }
  }
}

void mono_Tone(void)
{
  Serial.println(" mono_Tone ");
  FreqOUT = FreqOUT *10;
  //Serial.println("Freq: " + String(FreqOUT));
  //Serial.println("ASF:  " + String(ASF));
// if ((FreqOUT >= FMIN) && (FreqOUT <= FMAX))
  Serial3.println("yes");
  /*
  else
    Serial3.println("No");

    switch (ASF)
    {
      case 4:
        APWR = 0x0;
        break;
      case 1:
        APWR = 0x1;
        break;
      case 2:
        APWR = 0x2;
        break;
      case 5:
        APWR = 0x3;
        break;
    }
  */
  RFA_EN = 0x01;

  ProgramMAX2871();
}

void Stop(void)
{
  RFA_EN = 0x00;
  ProgramMAX2871();
  Serial.println("STOPPED");
  Serial3.print("off");
}


void blinkL(void)
{
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED , HIGH);
    delay(500);
    digitalWrite(LED , LOW);
    delay(500);
  }
}

void WriteMAX2871( unsigned long data )         //Writes 32 Bit value to register of MAX2871
{
  Serial.println(data , HEX);
  digitalWrite(LE, LOW);

  shiftOut(DATAPIN, CLOCKPIN, MSBFIRST, ((data & 0xFF000000) >> 24));
  shiftOut(DATAPIN, CLOCKPIN, MSBFIRST, ((data & 0x00FF0000) >> 16));
  shiftOut(DATAPIN, CLOCKPIN, MSBFIRST, ((data & 0x0000FF00) >> 8));
  shiftOut(DATAPIN, CLOCKPIN, MSBFIRST, (data & 0x000000FF));

  digitalWrite(LE, HIGH);
}

void ProgramMAX2871(void)                          // compose register value of register 0 and 4
{
  CalculateRegisterValues();

  composedRegisterValue = INT << 31 | NDIV << 15 | FRAC << 3 | ADDR0;           /* Addr 0 */
  WriteMAX2871(composedRegisterValue);

  composedRegisterValue = RES << 29 | SDLDO << 28 | SDDIV << 27                 /* Addr 4 */
                          | SDREF << 26 | FB << 23 | DIVA << 20 | BS << 12
                          | SDVCO << 11 | MTLD << 10 | BDIV << 9 | RFB_EN << 8
                          | BPWR << 6 | RFA_EN << 5 | APWR << 3 | ADDR4;

  WriteMAX2871(composedRegisterValue);
}


void CalculateRegisterValues()                 /* calculates values of NDIV, FRAC & DIVA  */
{
  double rest;

  if (FreqOUT >= 3000000000)
  {
    DIVA = 0;
    NDIV = ((FreqOUT / 50000000) * 10.0);
    rest = FreqOUT % 50000000;
    FRAC = rest / 50000000.0 * 4000.0;
    Serial.println("0");
  }
  else if ((FreqOUT < 3000000000) && (FreqOUT >= 1500000000))
  {
    DIVA = 1;
    NDIV = ((FreqOUT * 2 / 50000000) * 5.0);
    rest = FreqOUT * 2 % 50000000;
    FRAC = rest / 50000000.0 * 4000.0;
    Serial.println("1");
  }
  else if ((FreqOUT < 1500000000) && (FreqOUT >= 750000000))
  {
    DIVA = 2;
    NDIV = ((FreqOUT * 4 / 50000000) * 2.5);
    rest = FreqOUT * 4 % 50000000;
    FRAC = rest / 50000000.0 * 4000.0;
    Serial.println("2");
  }
  else if ((FreqOUT < 750000000) && (FreqOUT >= 375000000))
  {
    DIVA = 3;
    NDIV = ((FreqOUT * 8 / 50000000) * 1.265);
    rest = FreqOUT * 8 % 50000000;
    FRAC = (rest / 50000000.0 * 4000.0);
    Serial.println("3");
  }
  else if ((FreqOUT < 375000000) && (FreqOUT >= 187500000))
  {
    DIVA = 4;
    NDIV = ((FreqOUT * 16 / 50000000) * 0.625);
    rest = FreqOUT * 16 % 50000000;
    FRAC = ((rest / 50000000.0 * 4000.0) / 50);
    Serial.println("4");
  }
  else if ((FreqOUT < 187500000) && (FreqOUT >= 93750000))
  {
    DIVA = 5;
    NDIV = ((FreqOUT * 32 / 50000000) * 0.3125);
    rest = FreqOUT * 32 % 50000000;
    FRAC = rest / 50000000.0 * 4000.0;
    Serial.println("5");
  }
  else if ((FreqOUT < 93750000) && (FreqOUT >= 46875000))
  {
    DIVA = 6;
    NDIV = ((FreqOUT * 64 / 50000000) * 0.15625);
    rest = FreqOUT * 64 % 50000000;
    FRAC = rest / 50000000.0 * 4000.0;
    Serial.println("6");
  }
  else
  {
    DIVA = 7;
    NDIV = ((FreqOUT * 128 / 50000000) * 0.078128125);
    rest = FreqOUT * 128 % 50000000;
    FRAC = rest / 50000000.0 * 4000.0;
    Serial.println("7");
  }

  Serial.print("NDIV: ");
  Serial.println(NDIV, DEC);
  Serial.print("rest: ");
  Serial.println(rest, DEC);
  Serial.print("FRAC: ");
  Serial.println(FRAC, DEC);
}
