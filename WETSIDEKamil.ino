/*
  MENCO MEEN ROV V5_0
  MOD ON 13-Mar-2024(NOT TESTED)
  MCU:M2560
  6 X ESC_BLDC
  1 X CAM_SERVO
  1 X COMP-HMC5883L
  1 X ON_TP-BMP280
  1 X EX_TPD-MS5540C
  COMM:RS422M490
            6-T
  ===================
             F
  FHL(CCW)--------FHR(CCW)
             |
  L          |            R
             |
  AVL(CCW)---|---AVR(CW)
             |
             |
             |
  AHL(CW)---------AHR(CW)
             R

**************PIN_CONX**************************
  FUNC  COMPO    D_PIN   A_PIN 6T_CFG
  ESCs
      FHL       D9~            6T(FL)
      FHR       D7~            6T(FR)
      AHL       D5~            6T(RL)
      AHR       D3~            6T(RR)
      AVL       D6~            6T(VL)
      AVR       D2~            6T(VR)
  CAM
      RLY       D30
      SVO       D46~
  LED
      RLY       D31
      TIP       D4~
  VOLT                  A6
  ACS712                A2
  I2C
      SCL       D21
      SDA       D20
      vcc       3v3             DEPTH/BMP280
      vcc       5v              COMPASS
  SPI
      MISO      D52
      MOSI      D50
      SCLK      D51
      MCLK      D12
      SS        D53            UNUSED
      vcc       3v3
  UART
      TX0       D1
      RX0       D0
      TX1       D18
      RX1       D19
      TX2       D16
      RX2       D17
      TX3       D14
      RX3       D15
*********************************************
  CREATED BY MENCO
  http://menco.pline.co.in
  email: menco.mec@gmail.com
  Copyright (c) 2024 MENCO
  ==============================================================================
  Permission is hereby NOT granted, to any person(s) obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ============================================================================
*/
#include <EasyTransfer.h>//ET
#include <Servo.h>//ESC_SVO
#include <Wire.h>//I2C
#include <HMC5883L.h>//COMPASS
#include <SPI.h>//DEPTH
#include <Adafruit_BMP280.h>//BMP280

#define minpulse 1000
#define maxpulse 2000
//CAM
#define Tilt_N 90
//BMP280
#define BMP280_ADDRESS 0x76
//ESC_NEUT specific
const int ESC_FHL_N = 90;
const int ESC_FHR_N = 90;
const int ESC_AHL_N = 90;
const int ESC_AHR_N = 90;
const int ESC_AVL_N = 90;
const int ESC_AVR_N = 90;

const int FHL_pin = 9;
const int FHR_pin = 7;
const int AHL_pin = 5;
const int AHR_pin = 3;
const int AVL_pin = 6;
const int AVR_pin = 2;

//V_I

//ESC Objects
Servo ESC_FHL;
Servo ESC_FHR;
Servo ESC_AHL;
Servo ESC_AHR;
Servo ESC_AVL;
Servo ESC_AVR;


//CAM Init
int Cam_relay_state = HIGH;
Servo CamSvo_tilt;


//LIGHTS vari
int Led_relay_state = HIGH;
const int TIP = 4;


//COMPASS vari
const int hmc5883Address = 0x1E; //0011110b, I2C 7bit address
const byte hmc5883ModeRegister = 0x02;
const byte hmcContinuousMode = 0x00;
const byte hmcDataOutputXMSBAddress = 0x03;
int x, y, z; //triple axis data
int ROVangle; //horizontal heading angle.

//VOLT vari
const int Voltpin = A6;
int volts;
const float RefVolts = 5.0;
const float ResistFactor = 204.93;

//DEPTH vari
const int clock = 12;//Mega MCLK
float ROV_SeaDepth = 0; //SEA WATER in Ft
float ROV_FreDepth = 0; //FRESH WATER in Feet
void resetsensor()
{
  SPI.setDataMode(SPI_MODE0);
  SPI.transfer(0x15);
  SPI.transfer(0x55);
  SPI.transfer(0x40);
}

//BMP280
Adafruit_BMP280 bmp; // I2C


EasyTransfer ETin, ETout;

//RECEIVE_SEND DATA STRUCT
struct SEND_DATA_STRUCTURE {
  int ROVBattVolt;//V
  float ROVAmp; //A
  double ROV_ON_Temp; //C
  double ROV_ON_Press; //psi
  float ROV_EX_Temp; //C
  double ROV_EX_Press;//psi
  double ROVDepth; //Ft
  int ROVDir; //deg
  volatile boolean ROVPing;
};

struct RECEIVE_DATA_STRUCTURE {
  int FHL_pwm;
  int FHR_pwm;
  int AHL_pwm;
  int AHR_pwm;
  int AVL_pwm;
  int AVR_pwm;
  int Cam_tilt;
  volatile boolean LED_OnOff;
  volatile boolean CAM_OnOff;
  int LED_bright;
};

RECEIVE_DATA_STRUCTURE Rx_data;
SEND_DATA_STRUCTURE Tx_data;

void setup() {

  ESC_FHL.attach(FHL_pin, minpulse, maxpulse);
  ESC_FHR.attach(FHR_pin, minpulse, maxpulse);
  ESC_AHL.attach(AHL_pin, minpulse, maxpulse);
  ESC_AHR.attach(AHR_pin, minpulse, maxpulse);
  ESC_AVL.attach(AVL_pin, minpulse, maxpulse);
  ESC_AVR.attach(AVR_pin, minpulse, maxpulse);


  ESC_FHL.write(ESC_FHL_N);
  ESC_FHR.write(ESC_FHR_N);
  ESC_AHL.write(ESC_AHL_N);
  ESC_AHR.write(ESC_AHR_N);
  ESC_AVL.write(ESC_AVL_N);
  ESC_AVR.write(ESC_AVR_N);

  //Start I2C
  Wire.begin();

  //COMPASS Initialise
  Wire.beginTransmission(hmc5883Address);
  Wire.write(hmc5883ModeRegister);
  Wire.write(hmcContinuousMode);
  Wire.endTransmission();

  //CAMERA Initialise
  pinMode(30, OUTPUT);
  digitalWrite(30, Cam_relay_state);
  CamSvo_tilt.attach(46);
  CamSvo_tilt.write(Tilt_N);

  //LIGHTS
  pinMode(31, OUTPUT);
  digitalWrite(31, Led_relay_state);
  pinMode(TIP, OUTPUT);

  //DEPTH
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  pinMode(clock, OUTPUT);
  delay(75);

  Serial.begin(115200);
  Serial1.begin(115200);


  ETout.begin(details(Tx_data), &Serial1);
  ETin.begin(details(Rx_data), &Serial1);


  //BMP280 Initialise_variables
  while ( !Serial ) delay(20);
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS);
  if (!status) {
    while (1) delay(5);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  Tx_data.ROVPing = false;
}//setup end

void loop() {

  //ET IN_OUT

  for (uint8_t i = 0; i < 5; i++) {
    ETin.receiveData();
    delay(10); //DO NOT REMOVE
  }
  //ET END

  ESC_FHL.write(Rx_data.FHL_pwm);
  ESC_FHR.write(Rx_data.FHR_pwm);
  ESC_AHL.write(Rx_data.AHL_pwm);
  ESC_AHR.write(Rx_data.AHR_pwm);
  ESC_AVL.write(Rx_data.AVL_pwm);
  ESC_AVR.write(Rx_data.AVR_pwm);

  //CAM
  if (Rx_data.CAM_OnOff == 1) {
    digitalWrite (30, LOW); //RLY-ACTIVE
  }
  if (Rx_data.CAM_OnOff == 0) {
    digitalWrite (30, HIGH); //RLY-DEACTIVE
  }
  //CAMTILT
  CamSvo_tilt.write(Rx_data.Cam_tilt);
  //CAM END//

  //LIGHT
  if (Rx_data.LED_OnOff == 1) {
    digitalWrite (31, LOW);//RLY-ACTIVE
  }
  if (Rx_data.LED_OnOff == 0) {
    digitalWrite (31, HIGH);//RLY-DEACTIVE
  }
  analogWrite(TIP, Rx_data.LED_bright);
  //LIGHTS END


  //VOLT
  volts = analogRead(Voltpin) / ResistFactor * RefVolts; //Read V
  Tx_data.ROVBattVolt = volts;


  //CURRENT
  unsigned int x = 0;
  float AcsValue = 0.0, Samples = 0.0, AvgAcs = 0.0, AcsValueF = 0.0;
  for (int x = 0; x < 50; x++) {
    AcsValue = analogRead(A2);
    Samples = Samples + AcsValue;
    delay (3); // let ADC settle before next sample 3ms
  }
  AvgAcs = Samples / 50.0; //AVG
  AcsValueF = (2.5 - (AvgAcs * (5.0 / 1024.0)) ) / 0.100;
  delay(8);
  Tx_data.ROVAmp = (AcsValueF) / 10;
  //CURRENT END

  //COMPASS
  Wire.beginTransmission(hmc5883Address);
  Wire.write(hmcDataOutputXMSBAddress);
  Wire.endTransmission();
  //Read data from each axis
  Wire.requestFrom(hmc5883Address, 6);
  if (6 <= Wire.available())
  {
    x = Wire.read() << 8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read() << 8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read() << 8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  ROVangle = atan2(y, x) / M_PI * 180;
  if (ROVangle < 0)
  {
    ROVangle = ROVangle + 360;
  }

  Tx_data.ROVDir = ROVangle; //Send to TOP
  //COMPASS END

  //EX_T_P_DEPTH

  TCCR1B = (TCCR1B & 0xF8) | 1 ; //generates the MCKL signal
  analogWrite (clock, 128) ;
  resetsensor(); //resets the sensor - caution: afterwards mode = SPI_MODE0!
  //Calibration word 1
  unsigned int result1 = 0;
  unsigned int inbyte1 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0x50);
  SPI.setDataMode(SPI_MODE1);
  result1 = SPI.transfer(0x00);
  result1 = result1 << 8; //shift returned byte
  inbyte1 = SPI.transfer(0x00); //send dummy byte to read second byte of word
  result1 = result1 | inbyte1; //combine first and second byte of word
  resetsensor(); //resets the sensor

  unsigned int result2 = 0;
  byte inbyte2 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0x60);
  SPI.setDataMode(SPI_MODE1);
  result2 = SPI.transfer(0x00);
  result2 = result2 << 8;
  inbyte2 = SPI.transfer(0x00);
  result2 = result2 | inbyte2;
  resetsensor(); //resets the sensor

  unsigned int result3 = 0;
  byte inbyte3 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0x90);
  SPI.setDataMode(SPI_MODE1);
  result3 = SPI.transfer(0x00);
  result3 = result3 << 8;
  inbyte3 = SPI.transfer(0x00);
  result3 = result3 | inbyte3;
  resetsensor(); //resets the sensor

  unsigned int result4 = 0;
  byte inbyte4 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0xA0);
  SPI.setDataMode(SPI_MODE1);
  result4 = SPI.transfer(0x00);
  result4 = result4 << 8;
  inbyte4 = SPI.transfer(0x00);
  result4 = result4 | inbyte4;

  long c1 = (result1 >> 1) & 0x7FFF;
  long c2 = ((result3 & 0x003F) << 6) | (result4 & 0x003F);
  long c3 = (result4 >> 6) & 0x03FF;
  long c4 = (result3 >> 6) & 0x03FF;
  long c5 = ((result1 & 0x0001) << 10) | ((result2 >> 6) & 0x03FF);
  long c6 = result2 & 0x003F;
  resetsensor(); //resets the sensor

  //Ext_Pressure:
  unsigned int presMSB = 0; //first byte of value
  unsigned int presLSB = 0; //last byte of value
  unsigned int D1 = 0;
  SPI.transfer(0x0F); //send first byte of command to get pressure value
  SPI.transfer(0x40); //send second byte of command to get pressure value
  delay(35); //wait for conversion end
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  presMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
  presMSB = presMSB << 8; //shift first byte
  presLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
  D1 = presMSB | presLSB; //combine first and second byte of value
  resetsensor(); //resets the sensor

  //Ext_Temperature:
  unsigned int tempMSB = 0; //first byte of value
  unsigned int tempLSB = 0; //last byte of value
  unsigned int D2 = 0;
  SPI.transfer(0x0F); //send first byte of command to get temperature value
  SPI.transfer(0x20); //send second byte of command to get temperature value
  delay(35); //wait for conversion end
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  tempMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
  tempMSB = tempMSB << 8; //shift first byte
  tempLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
  D2 = tempMSB | tempLSB; //combine first and second byte of value

  const long UT1 = (c5 << 3) + 20224;
  const long dT = D2 - UT1;
  const long TEMP = 200 + ((dT * (c6 + 50)) >> 10);
  const long OFF  = (c2 * 4) + (((c4 - 512) * dT) >> 12);
  const long SENS = c1 + ((c3 * dT) >> 10) + 24576;
  const long X = (SENS * (D1 - 7168) >> 14) - OFF;
  long PCOMP = ((X * 10) >> 5) + 2500;
  float TEMPREAL = TEMP / 10;
  double PCOMPHG = PCOMP * 750.06 / 10000; // mbar*10 -> mmHg === ((mbar/10)/1000)*750/06
  double PCOMPS1 = PCOMPHG * 0.019334;
  double ROVExTemp = TEMPREAL;
  long T2 = 0;
  float P2 = 0;
  if (TEMP < 200)
  {
    T2 = (11 * (c6 + 24) * (200 - TEMP) * (200 - TEMP) ) >> 20;
    P2 = (3 * T2 * (PCOMP - 3500) ) >> 14;
  }
  else if (TEMP > 450)
  {
    T2 = (3 * (c6 + 24) * (450 - TEMP) * (450 - TEMP) ) >> 20;
    P2 = (T2 * (PCOMP - 10000) ) >> 13;
  }

  if ((TEMP < 200) || (TEMP > 450))
  {
    const float TEMP2 = TEMP - T2;
    const float PCOMP2 = PCOMP - P2;
    float TEMPREAL2 = TEMP2 / 10;
    double PCOMPHG2 = PCOMP2 * 750.06 / 10000; // mbar*10 -> mmHg === ((mbar/10)/1000)*750/06
    double PCOMPS1 = PCOMPHG2 * 0.019334;
    double ROVExTemp = TEMPREAL2;//EX T in C
  }
  Tx_data.ROV_EX_Temp = ROVExTemp;//EX T in C
  Tx_data.ROV_EX_Press = PCOMPS1; //EX P in psi
  delay(85);

  //BMP280
  double ROV_ON_Pres_pa = (bmp.readPressure());//ON_P Pa
  Tx_data.ROV_ON_Press = (ROV_ON_Pres_pa / 6894.7448254940);//Pa to Psi
  Tx_data.ROV_ON_Temp = (bmp.readTemperature());//ON_T- C
  double ROV_Alt = ((bmp.readAltitude(1016.78)) * 3.28084);//Alt in ft//Adjusted HWH
  delay(85);

  //ROV FreshWater Depth
  Tx_data.ROVDepth = ((Tx_data.ROV_EX_Press - Tx_data.ROV_ON_Press) * 0.33);//fresh water depth in ft

  //ROV LINK
  Tx_data.ROVPing = true;//ROV on

  //ET send
  ETout.sendData();

  delay(15);
}
