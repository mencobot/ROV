/*
  MENCO MEEN TOP V5_1
  MOD ON 13-mar-2024(TESTED-PART)
  MCU:MEGA2560_STD
  8 X THRUSTERS
  1 X PS2 SHLD
  1 X I2C_128x32_OLED_DISP
  UART RS422_MAX490
  =================PS2 GAME CTRL CONTROLS=======================================================
     CAM_TILT         L_XY             R_XY     SQR_P        TRI_P        L1_P      R1_P
     UP_PRESS         H_FF             SURF      LED         DISPLAY     PITCH_DN   AUX1
       A                A                A       ON_OFF      ON_OFF                 ON_OFF
       |                |                |
       G        YAW L<--L-->R   ROLL L<--R-->R  CIR_P        CRO_P        L2_P       R2_P
       |                |                |        CAM        HOLD        PITCH_UP    AUX2
       V                V                V       ON_OFF      ON_OFF      ON_OFF      ON_OFF
     DN_PRESS          H_RR             DIVE
  ==============================================================================================
  PIN_CONNX
                D   A
  PS2_SHLD
          RX   D9
          TX   D8
  SL0 SERIAL MONITOR FOR DEBUG
          RX  D0
          TX  D1
   SL1 RS422 COMM WITH ROV
          RX  D19
          TX  D18
  LED_POT
        CNTR      A3
  I2C
        SCL  D20
        SDA  D21
  VCC                 5V
  GND                 GND
  ==============================================================
  CREATED BY MENCO FOR USE WITH MEEN2 ROV HARDWARE
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
#include <SmartElexPs2Shield.h>
#include <EasyTransfer.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>

//DELAYS
#define Disp_wait 300

EasyTransfer ETin, ETout;
//SmartElexPs2Shield ps(9, 8);//UNO R3
SmartElexPs2Shield ps(11, 10);  //Mega2560

//OLED
#define SCREEN_WIDTH 128  // OLEDwidth,
#define SCREEN_HEIGHT 32  // OLEDheight
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//LCD I2C address
//Hold
volatile boolean HOLD_OnOff;

//ps2_N
const int LX_N = 123;
const int LY_N = 123;
const int RX_N = 133;
const int RY_N = 122;
const int T = 3;

const int CW_max2 = 180;
const int CW_max1 = 165;
const int CW_mid2 = 150;
const int CW_mid1 = 135;
const int CW_min2 = 120;
const int CW_min1 = 105;

const int CCW_max2 = 0;
const int CCW_max1 = 15;
const int CCW_mid2 = 30;
const int CCW_mid1 = 45;
const int CCW_min2 = 60;
const int CCW_min1 = 75;

const int p_min = 3;
const int p_max = 5;
const int T_step = 2;
const int P_step = 2;
const int Tilt_max = 114;
const int Tilt_min = 64;
const int Pan_max = 114;
const int Pan_min = 64;

int FHL_pwm;
int FHR_pwm;
int AHL_pwm;
int AHR_pwm;
int FVL_pwm;
int FVR_pwm;
int AVL_pwm;
int AVR_pwm;

const int FHL_N = 90;
const int FHR_N = 90;
const int AHL_N = 90;
const int AHR_N = 90;
const int FVL_N = 90;
const int FVR_N = 90;
const int AVL_N = 90;
const int AVR_N = 90;
const int ESC_trim = 3;

const int ml = 255;
const int mr = 255;

int requiredDir = 0;
long requiredDepth = 0;

int LX;
int LY;
int RX;
int RY;
int PF;
int PB;
int Disp;

struct RECEIVE_DATA_STRUCTURE {  //from ROV
  int ROVBattVolt;               //V
  float ROVAmp;                  //A
  double ROV_ON_Temp;            //C
  double ROV_ON_Press;           //psi
  float ROV_EX_Temp;             //C
  double ROV_EX_Press;           //psi
  double ROVDepth;               //Ft
  int ROVDir;                    //deg
  float ROVyaw;                  //deg
  float ROVpitch;                //deg
  float ROVroll;                 //deg
  volatile boolean ROVPing;
};

struct SEND_DATA_STRUCTURE {  //to ROV
  int FHL_pwm;
  int FHR_pwm;
  int AHL_pwm;
  int AHR_pwm;
  int FVL_pwm;
  int FVR_pwm;
  int AVL_pwm;
  int AVR_pwm;
  int Cam_tilt;
  int Cam_pan;
  volatile boolean LED_OnOff;
  volatile boolean CAM_OnOff;
  volatile boolean AUX1_OnOff;
  volatile boolean AUX2_OnOff;
  int LED_bright;
};

RECEIVE_DATA_STRUCTURE Rx_data;
SEND_DATA_STRUCTURE Tx_data;

void setup() {
  //CAM
  Tx_data.Cam_tilt = 90;
  Tx_data.Cam_pan = 90;
  Tx_data.CAM_OnOff = true;

  //LIGHT
  Tx_data.LED_OnOff = true;

  //HOLD
  HOLD_OnOff = false;

  //OLED
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true)
      ;
  }
  Serial.begin(9600);
  Serial1.begin(115200);
  ETin.begin(details(Rx_data), &Serial1);
  ETout.begin(details(Tx_data), &Serial1);

  //PS2
  ps.begin(115200);
  ps.SetController(ps.AnalogMode);

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 5);
  oled.println(" TOP Ready ");
  oled.setCursor(0, 20);
  oled.println("...Arming ROV");
  oled.display();
  delay(Disp_wait);
}

void loop() {
  //PS2
  ps.ReadControllerButtons();
  //Read L_R X_Y JS Val
  LX = ps.LEFT_X_AXIS;
  LY = ps.LEFT_Y_AXIS;
  RX = ps.RIGHT_X_AXIS;
  RY = ps.RIGHT_Y_AXIS;
  PF = ps.L1_Pressure;          //Pitch_dn
  PB = ps.R1_Pressure;          //Pitch_up
  Disp = ps.TRIANGLE_Pressure;  //Disp on_off

  if ((LX <= (LX_N + T)) && (LX >= (LX_N - T)) && (LY <= (LY_N + T)) && (LY >= (LY_N - T)))  //H_Stop
  {
    FHL_pwm = FHL_N;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = AHR_N;
  }
  if ((RX <= (RX_N + T)) && (RX >= (RX_N - T)) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //V_Stop
  {
    FVL_pwm = FVL_N;
    FVR_pwm = FVR_N;
    AVL_pwm = AVL_N;
    AVR_pwm = AVR_N;
  }
  //LEFT_JS
  if ((LX >= 115) && (LX <= (LX_N - T)))  //Y_Left_min1
  {
    FHL_pwm = CW_min1;
    FHR_pwm = CCW_min1;
    AHL_pwm = CCW_min1;
    AHR_pwm = CW_min1;
  }
  if ((LX >= 95) && (LX <= 114))  //Y_Left_min2
  {
    FHL_pwm = CW_min2;
    FHR_pwm = CCW_min2;
    AHL_pwm = CCW_min2;
    AHR_pwm = CW_min2;
  }
  if ((LX >= 75) && (LX <= 94))  //Y_Left_mid1
  {
    FHL_pwm = CW_mid1;
    FHR_pwm = CCW_mid1;
    AHL_pwm = CCW_mid1;
    AHR_pwm = CW_mid1;
  }
  if ((LX >= 55) && (LX <= 74))  //Y_Left_mid2
  {
    FHL_pwm = CW_mid2;
    FHR_pwm = CCW_mid2;
    AHL_pwm = CCW_mid2;
    AHR_pwm = CW_mid2;
  }
  if ((LX >= 25) && (LX <= 54))  //Y_Left_max1
  {
    FHL_pwm = CW_max1;
    FHR_pwm = CCW_max1;
    AHL_pwm = CCW_max1;
    AHR_pwm = CW_max1;
  }
  if ((LX >= 0) && (LX <= 24))  //Y_Left_max2
  {
    FHL_pwm = CW_max2;
    FHR_pwm = CCW_max2;
    AHL_pwm = CCW_max2;
    AHR_pwm = CW_max2;
  }
  if ((LX >= (LX_N + T)) && (LX <= 140))  //Y_Right_min1
  {
    FHL_pwm = CCW_min1;
    FHR_pwm = CW_min1;
    AHL_pwm = CW_min1;
    AHR_pwm = CCW_min1;
  }
  if ((LX >= 141) && (LX <= 160))  //Y_Right_min2
  {
    FHL_pwm = CCW_min2;
    FHR_pwm = CW_min2;
    AHL_pwm = CW_min2;
    AHR_pwm = CCW_min2;
  }
  if ((LX >= 161) && (LX <= 180))  //Y_Right_mid1
  {
    FHL_pwm = CCW_mid1;
    FHR_pwm = CW_mid1;
    AHL_pwm = CW_mid1;
    AHR_pwm = CCW_mid1;
  }
  if ((LX >= 181) && (LX <= 200))  //Y_Right_mid2
  {
    FHL_pwm = CCW_mid2;
    FHR_pwm = CW_mid2;
    AHL_pwm = CW_mid2;
    AHR_pwm = CCW_mid2;
  }
  if ((LX >= 201) && (LX <= 230))  //Y_Right_max1
  {
    FHL_pwm = CCW_max1;
    FHR_pwm = CW_max1;
    AHL_pwm = CW_max1;
    AHR_pwm = CCW_max1;
  }
  if ((LX >= 231) && (LX <= 255))  //Y_Right_max2
  {
    FHL_pwm = CCW_max2;
    FHR_pwm = CW_max2;
    AHL_pwm = CW_max2;
    AHR_pwm = CCW_max2;
  }
  if ((LY <= (LY_N - T)) && (LY >= 115))  //H_Fwd_min1
  {
    FHL_pwm = CW_min1;
    FHR_pwm = CW_min1;
    AHL_pwm = CW_min1;
    AHR_pwm = CW_min1;
  }
  if ((LY <= 114) && (LY >= 95))  //H_Fwd_min2
  {
    FHL_pwm = CW_min2;
    FHR_pwm = CW_min2;
    AHL_pwm = CW_min2;
    AHR_pwm = CW_min2;
  }
  if ((LY <= 94) && (LY >= 75))  //H_Fwd_mid1
  {
    FHL_pwm = CW_mid1;
    FHR_pwm = CW_mid1;
    AHL_pwm = CW_mid1;
    AHR_pwm = CW_mid1;
  }
  if ((LY <= 74) && (LY >= 55))  //H_Fwd_mid2
  {
    FHL_pwm = CW_mid2;
    FHR_pwm = CW_mid2;
    AHL_pwm = CW_mid2;
    AHR_pwm = CW_mid2;
  }
  if ((LY <= 54) && (LY >= 25))  //H_Fwd_max1
  {
    FHL_pwm = CW_max1;
    FHR_pwm = CW_max1;
    AHL_pwm = CW_max1;
    AHR_pwm = CW_max1;
  }
  if ((LY <= 24) && (LY >= 0))  //H_Fwd_max2
  {
    FHL_pwm = CW_max2;
    FHR_pwm = CW_max2;
    AHL_pwm = CW_max2;
    AHR_pwm = CW_max2;
  }
  if ((LY >= (LY_N + T)) && (LY <= 140))  //H_Rev_min1
  {
    FHL_pwm = CCW_min1;
    FHR_pwm = CCW_min1;
    AHL_pwm = CCW_min1;
    AHR_pwm = CCW_min1;
  }
  if ((LY >= 141) && (LY <= 160))  //H_Rev_min2
  {
    FHL_pwm = CCW_min2;
    FHR_pwm = CCW_min2;
    AHL_pwm = CCW_min2;
    AHR_pwm = CCW_min2;
  }
  if ((LY >= 161) && (LY <= 180))  //H_Rev_mid1
  {
    FHL_pwm = CCW_mid1;
    FHR_pwm = CCW_mid1;
    AHL_pwm = CCW_mid1;
    AHR_pwm = CCW_mid1;
  }
  if ((LY >= 181) && (LY <= 200))  //H_Rev_mid2
  {
    FHL_pwm = CCW_mid2;
    FHR_pwm = CCW_mid2;
    AHL_pwm = CCW_mid2;
    AHR_pwm = CCW_mid2;
  }
  if ((LY >= 201) && (LY <= 230))  //H_Rev_max1
  {
    FHL_pwm = CCW_max1;
    FHR_pwm = CCW_max1;
    AHL_pwm = CCW_max1;
    AHR_pwm = CCW_max1;
  }
  if ((LY >= 231) && (LY <= 255))  //H_Rev_max2
  {
    FHL_pwm = CCW_max2;
    FHR_pwm = CCW_max2;
    AHL_pwm = CCW_max2;
    AHR_pwm = CCW_max2;
  }
  ///LJS_diagonal
  if ((LX >= (LX_N + T)) && (LX <= 139) && (LY <= (LY_N - T)) && (LY >= 116))  //LQ1_min1
  {
    FHL_pwm = CW_min1;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_min1;
  }
  if ((LX >= 140) && (LX <= 159) && (LY <= 115) && (LY >= 96))  //LQ1_min2
  {
    FHL_pwm = CW_min2;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_min2;
  }
  if ((LX >= 160) && (LX <= 179) && (LY <= 95) && (LY >= 76))  //LQ1_med1
  {
    FHL_pwm = CW_mid1;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_mid1;
  }
  if ((LX >= 180) && (LX <= 199) && (LY <= 75) && (LY >= 56))  //LQ1_med2
  {
    FHL_pwm = CW_mid2;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_mid2;
  }
  if ((LX >= 200) && (LX <= 229) && (LY <= 55) && (LY >= 26))  //LQ1_max1
  {
    FHL_pwm = CW_max1;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_max1;
  }
  if ((LX >= 230) && (LX <= 255) && (LY <= 25) && (LY >= 0))  //LQ1_max2
  {
    FHL_pwm = CW_max2;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_max2;
  }
  /////////////////////////////////
  if ((LX >= (LX_N + T)) && (LX <= 140) && (LY >= (LY_N + T)) && (LY <= 140))  //LQ2_mi1
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_min1;
    AHL_pwm = CCW_min1;
    AHR_pwm = AHR_N;
  }
  if ((LX >= 141) && (LX <= 160) && (LY >= 141) && (LY <= 160))  //LQ2_min2
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_min2;
    AHL_pwm = CCW_min2;
    AHR_pwm = AHR_N;
  }
  if ((LX >= 161) && (LX <= 180) && (LY >= 161) && (LY <= 180))  //LQ2_med1
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_mid1;
    AHL_pwm = CCW_mid1;
    AHR_pwm = AHR_N;
  }
  if ((LX >= 181) && (LX <= 200) && (LY >= 181) && (LY <= 200))  //LQ2_med2
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_mid2;
    AHL_pwm = CCW_mid2;
    AHR_pwm = AHR_N;
  }
  if ((LX >= 201) && (LX <= 230) && (LY >= 201) && (LY <= 230))  //LQ2_max1
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_max1;
    AHL_pwm = CCW_max1;
    AHR_pwm = AHR_N;
  }
  if ((LX >= 231) && (LX <= 255) && (LY >= 231) && (LY <= 255))  //LQ2_max2
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_max2;
    AHL_pwm = CCW_max2;
    AHR_pwm = AHR_N;
  }
  ///////////////////
  if ((LX >= 115) && (LX <= (LX_N - T)) && (LY <= 140) && (LY >= (LY_N + T)))  //LQ3_min1
  {
    FHL_pwm = CCW_min1;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_min1;
  }
  if ((LX >= 95) && (LX <= 114) && (LY <= 160) && (LY >= 141))  //LQ3_min2
  {
    FHL_pwm = CCW_min2;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_min2;
  }
  if ((LX >= 75) && (LX <= 94) && (LY <= 180) && (LY >= 161))  //LQ3_med1
  {
    FHL_pwm = CCW_mid1;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_mid1;
  }
  if ((LX >= 55) && (LX <= 74) && (LY <= 200) && (LY >= 181))  //LQ3_med2
  {
    FHL_pwm = CCW_mid2;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_mid2;
  }
  if ((LX >= 25) && (LX <= 54) && (LY <= 230) && (LY >= 201))  //LQ3_max1
  {
    FHL_pwm = CCW_max1;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_max1;
  }
  if ((LX >= 0) && (LX <= 24) && (LY <= 255) && (LY >= 231))  //LQ3_max2
  {
    FHL_pwm = CCW_max2;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_max2;
  }
  //////////////////
  if ((LX <= (LX_N - T)) && (LX >= 115) && (LY <= (LY_N - T)) && (LY >= 115))  //LQ4_min1
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_min1;
    AHL_pwm = CW_min1;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 114) && (LX >= 95) && (LY <= 114) && (LY >= 95))  //LQ4_min2
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_min2;
    AHL_pwm = CW_min2;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 94) && (LX >= 75) && (LY <= 94) && (LY >= 75))  //LQ4_med1
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_mid1;
    AHL_pwm = CW_mid1;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 74) && (LX >= 55) && (LY <= 74) && (LY >= 55))  //LQ4_med2
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_mid2;
    AHL_pwm = CW_mid2;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 54) && (LX >= 25) && (LY <= 54) && (LY >= 25))  //LQ4_max1
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_max1;
    AHL_pwm = CW_max1;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 24) && (LX >= 0) && (LY <= 24) && (LY >= 0))  //LQ4_max2
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_max2;
    AHL_pwm = CW_max2;
    AHR_pwm = AHR_N;
  }
  /////////////////
  //RIGHT JS
  if ((RX >= 115) && (RX <= (RX_N - T)) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //R_Left_min1
  {
    FVL_pwm = CW_min1;
    FVR_pwm = CW_min1;
    AVL_pwm = CCW_min1;
    AVR_pwm = CCW_min1;
  }
  if ((RX >= 95) && (RX <= 114) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //R_Left_min2
  {
    FVL_pwm = CW_min2;
    FVR_pwm = CW_min2;
    AVL_pwm = CCW_min2;
    AVR_pwm = CCW_min2;
  }
  if ((RX >= 75) && (RX <= 94) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //R_Left_med1
  {
    FVL_pwm = CW_mid1;
    FVR_pwm = CW_mid1;
    AVL_pwm = CCW_mid1;
    AVR_pwm = CCW_mid1;
  }
  if ((RX >= 55) && (RX <= 74) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //R_Left_med2
  {
    FVL_pwm = CW_mid2;
    FVR_pwm = CW_mid2;
    AVL_pwm = CCW_mid2;
    AVR_pwm = CCW_mid2;
  }
  if ((RX >= 25) && (RX <= 54) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //R_Left_max1
  {
    FVL_pwm = CW_max1;
    FVR_pwm = CW_max1;
    AVL_pwm = CCW_max1;
    AVR_pwm = CCW_max1;
  }
  if ((RX >= 0) && (RX <= 24) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //R_Left_max2
  {
    FVL_pwm = CW_max2;
    FVR_pwm = CW_max2;
    AVL_pwm = CCW_max2;
    AVR_pwm = CCW_max2;
  }
  ///////////////
  if ((RX >= (RX_N + T)) && (RX <= 140) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //R_Right_min1
  {
    FVL_pwm = CCW_min1;
    FVR_pwm = CCW_min1;
    AVL_pwm = CW_min1;
    AVR_pwm = CW_min1;
  }
  if ((RX >= 141) && (RX <= 160) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //R_Right_min2
  {
    FVL_pwm = CCW_min2;
    FVR_pwm = CCW_min2;
    AVL_pwm = CW_min2;
    AVR_pwm = CW_min2;
  }
  if ((RX >= 161) && (RX <= 180) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //R_Right_med1
  {
    FVL_pwm = CCW_mid1;
    FVR_pwm = CCW_mid1;
    AVL_pwm = CW_mid1;
    AVR_pwm = CW_mid1;
  }
  if ((RX >= 181) && (RX <= 200) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //R_Right_med2
  {
    FVL_pwm = CCW_mid2;
    FVR_pwm = CCW_mid2;
    AVL_pwm = CW_mid2;
    AVR_pwm = CW_mid2;
  }
  if ((RX >= 201) && (RX <= 230) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //R_Right_max1
  {
    FVL_pwm = CCW_max1;
    FVR_pwm = CCW_max1;
    AVL_pwm = CW_max1;
    AVR_pwm = CW_max1;
  }
  if ((RX >= 231) && (RX <= 255) && (RY <= (RY_N + T)) && (RY >= (RY_N - T)))  //R_Right_max2
  {
    FVL_pwm = CCW_max2;
    FVR_pwm = CCW_max2;
    AVL_pwm = CW_max2;
    AVR_pwm = CW_max2;
  }
  ///////////////////////
  if ((RY <= (RY_N - T)) && (RY >= 115) && (RX >= (RX_N - T)) && (RX <= (RX_N + T)))  //Surf_min1
  {
    FVL_pwm = CCW_min1;
    FVR_pwm = CW_min1;
    AVL_pwm = CW_min1;
    AVR_pwm = CCW_min1;
  }
  if ((RY <= 114) && (RY >= 95) && (RX >= (RX_N - T)) && (RX <= (RX_N + T)))  //Surf_min2
  {
    FVL_pwm = CCW_min2;
    FVR_pwm = CW_min2;
    AVL_pwm = CW_min2;
    AVR_pwm = CCW_min2;
  }
  if ((RY <= 94) && (RY >= 75) && (RX >= (RX_N - T)) && (RX <= (RX_N + T)))  //Surf_mid1
  {
    FVL_pwm = CCW_mid1;
    FVR_pwm = CW_mid1;
    AVL_pwm = CW_mid1;
    AVR_pwm = CCW_mid1;
  }
  if ((RY <= 74) && (RY >= 55) && (RX >= (RX_N - T)) && (RX <= (RX_N + T)))  //Surf_mid2
  {
    FVL_pwm = CCW_mid2;
    FVR_pwm = CW_mid2;
    AVL_pwm = CW_mid2;
    AVR_pwm = CCW_mid2;
  }
  if ((RY <= 54) && (RY >= 25) && (RX >= (RX_N - T)) && (RX <= (RX_N + T)))  //Surf_max1
  {
    FVL_pwm = CCW_max1;
    FVR_pwm = CW_max1;
    AVL_pwm = CW_max1;
    AVR_pwm = CCW_max1;
  }
  if ((RY <= 24) && (RY >= 0) && (RX >= (RX_N - T)) && (RX <= (RX_N + T)))  //Surf_max2
  {
    FVL_pwm = CCW_max2;
    FVR_pwm = CW_max2;
    AVL_pwm = CW_max2;
    AVR_pwm = CCW_max2;
  }
  /////////////
  if ((RY >= (RY_N + T)) && (RY <= 140) && (RX >= (RX_N - T)) && (RX <= (RX_N + T)))  //Dive_min1
  {
    FVL_pwm = CW_min1;
    FVR_pwm = CCW_min1;
    AVL_pwm = CCW_min1;
    AVR_pwm = CW_min1;
  }
  if ((RY >= 141) && (RY <= 160) && (RX >= (RX_N - T)) && (RX <= (RX_N + T)))  //Dive_min2
  {
    FVL_pwm = CW_min2;
    FVR_pwm = CCW_min2;
    AVL_pwm = CCW_min2;
    AVR_pwm = CW_min2;
  }
  if ((RY >= 161) && (RY <= 180) && (RX >= (RX_N - T)) && (RX <= (RX_N + T)))  //Dive_mid1
  {
    FVL_pwm = CW_mid1;
    FVR_pwm = CCW_mid1;
    AVL_pwm = CCW_mid1;
    AVR_pwm = CW_mid1;
  }
  if ((RY >= 181) && (RY <= 200) && (RX >= (RX_N - T)) && (RX <= (RX_N + T)))  //Dive_mid2
  {
    FVL_pwm = CW_mid2;
    FVR_pwm = CCW_mid2;
    AVL_pwm = CCW_mid2;
    AVR_pwm = CW_mid2;
  }
  if ((RY >= 201) && (RY <= 230) && (RX >= (RX_N - T)) && (RX <= (RX_N + T)))  //Dive_max1
  {
    FVL_pwm = CW_max1;
    FVR_pwm = CCW_max1;
    AVL_pwm = CCW_max1;
    AVR_pwm = CW_max1;
  }
  if ((RY >= 231) && (RY <= 255) && (RX >= (RX_N - T)) && (RX <= (RX_N + T)))  //Dive_max2
  {
    FVL_pwm = CW_max2;
    FVR_pwm = CCW_max2;
    AVL_pwm = CCW_max2;
    AVR_pwm = CW_max2;
  }
  ////////////////////
  if ((PF >= 5) && (PB <= 3) && (RX <= (RX_N + T)) && (RX >= (RX_N - T)))  //P_fwd
  {
    FVL_pwm = CW_mid2;
    FVR_pwm = CCW_mid2;
    AVL_pwm = CW_max2;
    AVR_pwm = CCW_max2;
  }
  if ((PB >= 5) && (PF <= 3) && (RX <= (RX_N + T)) && (RX >= (RX_N - T)))  //P_rev
  {
    FVL_pwm = CCW_max2;
    FVR_pwm = CW_max2;
    AVL_pwm = CCW_mid2;
    AVR_pwm = CCW_mid2;
  }

  ///RJS_diagonal

  if ((RX >= (RX_N + T)) && (RX <= 140) && (RY <= (RY_N - T)) && (RY >= 115))  //pitch_fwdR_min1
  {
    FVL_pwm = CCW_min1;
    FVR_pwm = FVR_N;
    AVL_pwm = CW_min1;
    AVR_pwm = CCW_min1;
  }
  if ((RX >= 141) && (RX <= 160) && (RY <= 114) && (RY >= 95))  //pitch_fwdR_min2
  {
    FVL_pwm = CCW_min2;
    FVR_pwm = CCW_min1;
    AVL_pwm = CW_min2;
    AVR_pwm = CCW_min2;
  }
  if ((RX >= 161) && (RX <= 180) && (RY <= 94) && (RY >= 75))  //pitch_fwdR_med1
  {
    FVL_pwm = CCW_mid1;
    FVR_pwm = CCW_min2;
    AVL_pwm = CW_mid1;
    AVR_pwm = CCW_mid1;
  }
  if ((RX >= 181) && (RX <= 200) && (RY <= 74) && (RY >= 55))  //pitch_fwdR_med2
  {
    FVL_pwm = CCW_mid2;
    FVR_pwm = CCW_mid1;
    AVL_pwm = CW_mid2;
    AVR_pwm = CCW_mid2;
  }
  if ((RX >= 201) && (RY <= 230) && (RY <= 54) && (RY >= 25))  //pitch_fwdR_max1
  {
    FVL_pwm = CCW_max1;
    FVR_pwm = CCW_mid2;
    AVL_pwm = CW_max1;
    AVR_pwm = CCW_max1;
  }
  if ((RX >= 231) && (RY <= 255) && (RY <= 24) && (RY >= 0))  //pitch_fwdR_max2
  {
    FVL_pwm = CCW_max2;
    FVR_pwm = CCW_max1;
    AVL_pwm = CW_max2;
    AVR_pwm = CCW_max2;
  }
  /////////////////
  if ((RX >= (RX_N + T)) && (RX <= 140) && (RY >= (RY_N + T)) && (RY <= 140))  //pitch_revR_min1
  {
    FVL_pwm = CCW_min1;
    FVR_pwm = CW_min1;
    AVL_pwm = CW_min1;
    AVR_pwm = AVR_N;
    ;
  }
  if ((RX >= 141) && (RX <= 160) && (RY >= 141) && (RY <= 160))  //pitch_revR_min2
  {
    FVL_pwm = CCW_min2;
    FVR_pwm = CW_min2;
    AVL_pwm = CW_min2;
    AVR_pwm = CW_min1;
  }
  if ((RX >= 161) && (RX <= 180) && (RY >= 161) && (RY <= 180))  //pitch_revR_med1
  {
    FVL_pwm = CCW_mid1;
    FVR_pwm = CW_mid1;
    AVL_pwm = CW_mid1;
    AVR_pwm = CW_min2;
  }
  if ((RX >= 181) && (RX <= 200) && (RY >= 181) && (RY <= 200))  //pitch_revR_med2
  {
    FVL_pwm = CCW_mid2;
    FVR_pwm = CW_mid2;
    AVL_pwm = CW_mid2;
    AVR_pwm = CW_mid1;
  }
  if ((RX >= 201) && (RX <= 230) && (RY >= 201) && (RY <= 230))  //pitch_revR_max1
  {
    FVL_pwm = CCW_max1;
    FVR_pwm = CW_max1;
    AVL_pwm = CW_max1;
    AVR_pwm = CW_mid2;
  }
  if ((RX >= 231) && (RX <= 255) && (RY >= 231) && (RY <= 255))  //pitch_revR_max2
  {
    FVL_pwm = CCW_max2;
    FVR_pwm = CW_max2;
    AVL_pwm = CW_max2;
    AVR_pwm = CW_max1;
  }
  /////////////////////
  if ((RX >= 115) && (RX <= (RX_N - T)) && (RY <= 140) && (RY >= (RY_N + T)))  //pitch_revL_min1
  {
    FVL_pwm = CCW_min1;
    FVR_pwm = CW_min1;
    AVL_pwm = AVL_N;
    AVR_pwm = CCW_min1;
  }
  if ((RX >= 95) && (RX <= 114) && (RY <= 160) && (RY >= 141))  //pitch_revL_min2
  {
    FVL_pwm = CCW_min2;
    FVR_pwm = CW_min2;
    AVL_pwm = CW_min1;
    AVR_pwm = CCW_min2;
  }
  if ((RX >= 75) && (RX <= 94) && (RY <= 180) && (RY >= 161))  //pitch_revL_med2
  {
    FVL_pwm = CCW_mid1;
    FVR_pwm = CW_mid1;
    AVL_pwm = CW_min2;
    AVR_pwm = CCW_mid1;
  }
  if ((RX >= 55) && (RX <= 74) && (RY <= 200) && (RY >= 181))  //pitch_revL_med2
  {
    FVL_pwm = CCW_mid2;
    FVR_pwm = CW_mid2;
    AVL_pwm = CW_mid1;
    AVR_pwm = CCW_mid2;
  }
  if ((RX >= 25) && (RX <= 54) && (RY <= 230) && (RY >= 201))  //pitch_revL_max1
  {
    FVL_pwm = CCW_max1;
    FVR_pwm = CW_max1;
    AVL_pwm = CW_mid2;
    AVR_pwm = CCW_max1;
  }
  if ((RX >= 0) && (RX <= 24) && (RY <= 255) && (RY >= 231))  //pitch_revL_max2
  {
    FVL_pwm = CCW_max2;
    FVR_pwm = CW_max2;
    AVL_pwm = CW_max1;
    AVR_pwm = CCW_max2;
  }
  //////////////////
  if ((RX <= (RX_N - T)) && (RX >= 115) && (RY <= (RY_N - T)) && (RY >= 115))  //pitch_fwdL_min1
  {
    FVL_pwm = FVL_N;
    FVR_pwm = CW_min1;
    AVL_pwm = CW_min1;
    AVR_pwm = CCW_min1;
  }
  if ((RX <= 114) && (RX >= 95) && (RY <= 114) && (RY >= 95))  //pitch_fwdL_min2
  {
    FVL_pwm = CW_min1;
    FVR_pwm = CW_min2;
    AVL_pwm = CW_min2;
    AVR_pwm = CCW_min2;
  }
  if ((RX <= 94) && (RX >= 75) && (RY <= 94) && (RY >= 75))  //pitch_fwdL_med1
  {
    FVL_pwm = CW_min2;
    FVR_pwm = CW_mid1;
    AVL_pwm = CW_mid1;
    AVR_pwm = CCW_mid1;
  }
  if ((RX <= 74) && (RX >= 55) && (RY <= 74) && (RY >= 55))  //pitch_fwdL_med2
  {
    FVL_pwm = CW_mid1;
    FVR_pwm = CW_mid2;
    AVL_pwm = CW_mid2;
    AVR_pwm = CCW_mid2;
  }
  if ((RX <= 54) && (RY <= 25) && (RY <= 54) && (RY >= 25))  //pitch_fwdL_max1
  {
    FVL_pwm = CW_mid2;
    FVR_pwm = CW_max1;
    AVL_pwm = CW_max1;
    AVR_pwm = CCW_max1;
  }
  if ((RX <= 24) && (RX >= 0) && (RY <= 24) && (RY >= 0))  //pitch_fwdL_max2
  {
    FVL_pwm = CW_max1;
    FVR_pwm = CW_max2;
    AVL_pwm = CW_max2;
    AVR_pwm = CCW_max2;
  }
 
  Tx_data.FHL_pwm = FHL_pwm;
  Tx_data.FHR_pwm = FHR_pwm;
  Tx_data.AHL_pwm = AHL_pwm;
  Tx_data.AHR_pwm = AHR_pwm;
  Tx_data.FVL_pwm = FVL_pwm;
  Tx_data.FVR_pwm = FVR_pwm;
  Tx_data.AVL_pwm = AVL_pwm;
  Tx_data.AVR_pwm = AVR_pwm;

  //Toggle SQR_P
  if (ps.SQUARE_Pressure >= p_max) {
    Tx_data.LED_OnOff = !Tx_data.LED_OnOff;
  }
  //Toggle CAM CIR_P
  if (ps.CIRCLE_Pressure >= p_max) {
    Tx_data.CAM_OnOff = !Tx_data.CAM_OnOff;
  }
  //Hold_read
  if (ps.CROSS == 0) {
    requiredDir = Rx_data.ROVDir;
    requiredDepth = Rx_data.ROVDepth;
  }
  //Toggle HOLD CRO_P
  if (ps.CROSS_Pressure >= p_max) {
    HOLD_OnOff = !HOLD_OnOff;
  }
  //Toggle AUX1 R1_P
  if (ps.R1_Pressure >= p_max) {
    Tx_data.AUX1_OnOff = !Tx_data.AUX1_OnOff;
  }
  //Toggle AUX2 R2_P
  if (ps.R2_Pressure >= p_max) {
    Tx_data.AUX2_OnOff = !Tx_data.AUX2_OnOff;
  }
  //CAM_TILT
  if (ps.UP_Pressure >= p_max && ps.DOWN_Pressure <= p_min) {
    Tx_data.Cam_tilt = Tx_data.Cam_tilt + T_step;
  }
  if (ps.DOWN_Pressure >= p_max && ps.UP_Pressure <= p_min) {
    Tx_data.Cam_tilt = Tx_data.Cam_tilt - T_step;
  }
  Tx_data.Cam_tilt = constrain(Tx_data.Cam_tilt, Tilt_min, Tilt_max);

  //CAM_PAN
  if (ps.LEFT_Pressure >= p_max && ps.RIGHT_Pressure <= p_min) {
    Tx_data.Cam_pan = Tx_data.Cam_pan + P_step;
  }
  if (ps.RIGHT_Pressure >= p_max && ps.LEFT_Pressure <= p_min) {
    Tx_data.Cam_pan = Tx_data.Cam_pan - P_step;
  }
  Tx_data.Cam_pan = constrain(Tx_data.Cam_pan, Pan_min, Pan_max);

  // LIGHT
  int LED_Pot_Value = analogRead(A3);
  if (LED_Pot_Value >= 0 && LED_Pot_Value <= 106) {
    Tx_data.LED_bright = 30;
  }
  if (LED_Pot_Value >= 107 && LED_Pot_Value <= 208) {
    Tx_data.LED_bright = 55;
  }
  if (LED_Pot_Value >= 209 && LED_Pot_Value <= 310) {
    Tx_data.LED_bright = 80;
  }
  if (LED_Pot_Value >= 311 && LED_Pot_Value <= 412) {
    Tx_data.LED_bright = 105;
  }
  if (LED_Pot_Value >= 413 && LED_Pot_Value <= 514) {
    Tx_data.LED_bright = 130;
  }
  if (LED_Pot_Value >= 515 && LED_Pot_Value <= 616) {
    Tx_data.LED_bright = 155;
  }
  if (LED_Pot_Value >= 617 && LED_Pot_Value <= 718) {
    Tx_data.LED_bright = 180;
  }
  if (LED_Pot_Value >= 719 && LED_Pot_Value <= 820) {
    Tx_data.LED_bright = 205;
  }
  if (LED_Pot_Value >= 821 && LED_Pot_Value <= 922) {
    Tx_data.LED_bright = 230;
  }
  if (LED_Pot_Value >= 923 && LED_Pot_Value <= 1024) {
    Tx_data.LED_bright = 255;
  }

  //HOLD_Logic

  //ET IN_OUT
  ETout.sendData();
  //TOP<-ROV
  for (uint8_t i = 0; i < 5; i++) {
    ETin.receiveData();
    delay(10);
  }
  //DISP
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 5);
  oled.println("TO TOGGLE DISPLAY");
  oled.setCursor(0, 20);
  oled.println("PRESS PS2 TRIANGLE");
  oled.display();
  if (Disp >= p_max) {
    //ROV STATUS
    if (Rx_data.ROVPing == 1) {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 5);
      oled.println("ROV LINK OKAY");
      oled.setCursor(0, 20);
      oled.println("TOP>>>0<<<ROV");
      oled.display();
      delay(Disp_wait);
    } else {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 5);
      oled.println("ROV LINK LOST");
      oled.setCursor(0, 20);
      oled.println("TOP-|X|-ROV");
      oled.display();
      delay(Disp_wait);
    }
    //JS_POS
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("LX::LY::RX::RY");
    oled.setCursor(0, 20);
    oled.print(LX);
    oled.print("\t");
    oled.print(LY);
    oled.print("\t");
    oled.print(RX);
    oled.print("\t");
    oled.print(RY);
    oled.display();
    delay(Disp_wait);

    //CAM
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("CAM::TILT::PAN");
    oled.setCursor(0, 20);
    if (Tx_data.CAM_OnOff == 1) {
      oled.println(" ON ");
    } else {
      oled.println(" OFF ");
    }
    oled.setCursor(40, 20);
    oled.println(Tx_data.Cam_tilt);
    oled.setCursor(80, 20);
    oled.print(Tx_data.Cam_pan);
    oled.display();
    delay(Disp_wait);

    //LIGHTS
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("Light Level");
    oled.setCursor(0, 20);
    if (Tx_data.LED_OnOff == 1) {
      oled.println(" ON ");
    } else {
      oled.print(" OFF");
    }
    oled.setCursor(40, 20);
    oled.println(Tx_data.LED_bright, DEC);
    oled.display();
    delay(Disp_wait);

    //HOLD
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("HOLD_Head_Depth");
    oled.setCursor(0, 20);
    if (HOLD_OnOff == 1) {
      oled.println(" ON ");
    } else {
      oled.println(" OFF");
    }
    oled.display();
    delay(Disp_wait);

    //ROV BATT VOLT
    if (Rx_data.ROVBattVolt <= 9.8)  //9.6V 3s lipo low cut-off
    {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 5);
      oled.println("WARNING !!");
      oled.setCursor(0, 20);
      oled.println("BATT LOW VOLTS");
      oled.display();
      delay(Disp_wait);
    }
    if (Rx_data.ROVBattVolt >= 12.4)  //12.6V 3s lipo high cut-off
    {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 5);
      oled.println("WARNING !!");
      oled.setCursor(0, 20);
      oled.println("BATT HIGH VOLTS");
      oled.display();
      delay(Disp_wait);
    }
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("Batt Volts");
    oled.setCursor(0, 20);
    oled.println(Rx_data.ROVBattVolt);
    oled.display();
    delay(Disp_wait);

    //ROV BATT AMPS
    if (Rx_data.ROVAmp >= 10.5) {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 5);
      oled.println("WARNING !!");
      oled.setCursor(0, 20);
      oled.println("BATT HIGH AMPS");
      oled.display();
      delay(Disp_wait);
    }
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("Current Amps");
    oled.setCursor(0, 20);
    oled.println(Rx_data.ROVAmp);
    oled.display();
    delay(Disp_wait);

    //ROV ONBD TEMP
    if (Rx_data.ROV_ON_Temp >= 28.4)  //normal elecx operating Tmax=29.4C
    {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 5);
      oled.println("WARNING !!");
      oled.setCursor(0, 20);
      oled.println("HIGH INT TEMP");
      oled.display();
      delay(Disp_wait);
    }
    if (Rx_data.ROV_ON_Temp <= 11.0) {  //normal elecx operating Tmin=10C
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 5);
      oled.println("WARNING !!");
      oled.setCursor(0, 20);
      oled.println("LOW INT TEMP");
      oled.display();
      delay(Disp_wait);
    }
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("Int Temp *C");
    oled.setCursor(0, 20);
    oled.println(Rx_data.ROV_ON_Temp);
    oled.display();
    delay(Disp_wait);

    //ROV ONBD PRESS
    if (Rx_data.ROV_ON_Press >= 44.0)  //normal P at sea level 14.7psi + 26psi at 60ft depth
    {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 5);
      oled.println("WARNING !!");
      oled.setCursor(0, 20);
      oled.println("HIGH INT PRESS");
      oled.display();
      delay(Disp_wait);
    }
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("Int Press Psi");
    oled.setCursor(0, 20);
    oled.println(Rx_data.ROV_ON_Press);
    oled.display();
    delay(Disp_wait);

    //ROV EXT TEMP
    if (Rx_data.ROV_EX_Temp >= 35.0) {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 5);
      oled.println("WARNING !!");
      oled.setCursor(0, 20);
      oled.println("HIGH EXT TEMP");
      oled.display();
      delay(Disp_wait);
    }
    if (Rx_data.ROV_EX_Temp <= 5.0) {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 5);
      oled.println("WARNING !!");
      oled.setCursor(0, 20);
      oled.println("LOW EXT TEMP");
      oled.display();
      delay(Disp_wait);
    }
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("Ext Temp *C");
    oled.setCursor(0, 20);
    oled.println(Rx_data.ROV_EX_Temp);
    oled.display();
    delay(Disp_wait);

    //ROV EXT Pres//
    if (Rx_data.ROV_EX_Press >= 25.93)  //P= (25.93psi fresh) or (26.63psi salt) water at 60ft depth
    {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 5);
      oled.println("WARNING !!");
      oled.setCursor(0, 20);
      oled.println("HIGH EXT PRESS");
      oled.display();
      delay(Disp_wait);
    }
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("Ext Press Psi");
    oled.setCursor(0, 20);
    oled.println(Rx_data.ROV_EX_Press);
    oled.display();
    delay(Disp_wait);

    //ROV DEPTH
    if (Rx_data.ROVDepth >= 60.0) {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 5);
      oled.println("WARNING !!");
      oled.setCursor(0, 20);
      oled.println("DEPTH > 60 FT.");
      oled.display();
      delay(Disp_wait);
    }
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("Depth in Ft.");
    oled.setCursor(0, 20);
    oled.println(Rx_data.ROVDepth);
    oled.display();
    delay(Disp_wait);

    //ROV DIR
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("Heading Deg");
    oled.setCursor(0, 20);
    oled.println(Rx_data.ROVDir);
    oled.display();
    delay(Disp_wait);

    //ROV Y_R_P
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 5);
    oled.println("YAW:ROLL:PITCH Deg");
    oled.setCursor(0, 20);
    oled.print(Rx_data.ROVyaw);
    oled.print("\t");
    oled.print(Rx_data.ROVroll);
    oled.print("\t");
    oled.print(Rx_data.ROVpitch);
    oled.display();
    delay(Disp_wait);
  }
}
