/*
  MENCO MEEN TOP V5_0
  6 X BLDC THRUSTERS
  MOD ON 13-Mar-2024(NOT TESTED)
  MCU:UNO_R3
  1 X PS2 SHLD
  1 X I2C_16X2_LCD DISP
  COMM RS422_MAX490
  =================PS2 GAME CTRL CONTROLS=======================
     CAM_TILT         L_XY             R_XY     SQR_BTN      CRO_P
     UP_PRESS         H_FF             SURF    HEADLIGHTS     HOLD
       A                A                A       ON_OFF     ON_OFF
       |                |                |
       G        YAW L<--L-->R   ROLL L<--R-->R  CIR_BTN     TRI_P
       |                |                |        CAM       DISP
       V                V                V       ON_OFF     ON_OFF
     DN_PRESS          H_RR             DIVE
  =============================================================
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
#include <SmartElexPs2Shield.h>
#include <EasyTransfer.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

//DELAYS
#define Disp_wait 108

EasyTransfer ETin, ETout;
SmartElexPs2Shield ps(9, 8);

//LCD I2C address
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int LX_N = 123;
const int LY_N = 123;
const int RX_N = 123;
const int RY_N = 123;
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
const int T_step = 5;
const int T_max = 115;
const int T_min = 65;
const int ml = 255;
const int mr = 255;

//pwm
int FHL_pwm;
int FHR_pwm;
int AHL_pwm;
int AHR_pwm;
int AVL_pwm;
int AVR_pwm;

const int FHL_N = 90;
const int FHR_N = 90;
const int AHL_N = 90;
const int AHR_N = 90;
const int AVL_N = 90;
const int AVR_N = 90;
const int ESC_trim = 3;


int LX;
int LY;
int RX;
int RY;
int RLU;
int RLD;
int RRU;
int RRD;
int Disp;

//Hold
volatile boolean Hold_OnOff = false;
int requireddir = 0;
int requireddepth = 0;

struct RECEIVE_DATA_STRUCTURE
{ //from ROV
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

struct SEND_DATA_STRUCTURE
{ //to ROV
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
  //CAM
  Tx_data.Cam_tilt = 90;
  Tx_data.CAM_OnOff = true;

  //LIGHT
  Tx_data.LED_OnOff = true;

  //LCD
  lcd.begin();
  lcd.backlight();

  Serial.begin(115200);
  ETin.begin(details(Rx_data), &Serial);
  ETout.begin(details(Tx_data), &Serial);

  //PS2
  ps.begin(9600);
  ps.SetController(ps.AnalogMode);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" TOP Ready ");
  lcd.setCursor(1, 1);
  lcd.print("Wait...");
  delay(150);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("TOGGLE Display");
  delay(Disp_wait);
  lcd.setCursor(1, 1);
  lcd.print("Press TRIANGLE");
  delay(150);
}

void loop() {

  //PS2
  ps.ReadControllerButtons();
  //Read L_R X_Y JS Val
  LX = ps.LEFT_X_AXIS; //LX
  LY = ps.LEFT_Y_AXIS; //LY
  RX = ps.RIGHT_X_AXIS; //RX
  RY = ps.RIGHT_Y_AXIS; //RY
  RLU = ps.L1_Pressure;//R_left_up
  RRU = ps.R1_Pressure;//R_right_up
  Disp = ps.TRIANGLE_Pressure; //Disp on_off

  if ((LX <= (LX_N + T)) && (LX >= (LX_N - T)) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //H_Stop
  {
    FHL_pwm = FHL_N;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = AHR_N;

  }
  if ((RX <= (RX_N + T)) && (RX >= (RX_N - T)) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //V_Stop
  {
    AVL_pwm = AVL_N;
    AVR_pwm = AVR_N;

  }
  //LEFT_JS
  if ((LX >= 115) && (LX <= (LX_N - T)) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //Y_Left_min1
  {
    FHL_pwm = CW_min1;
    FHR_pwm = CCW_min1;
    AHL_pwm = CCW_min1;
    AHR_pwm = CW_min1;
  }
  if ((LX >= 95) && (LX <= 114) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //Y_Left_min2
  {
    FHL_pwm = CW_min2;
    FHR_pwm = CCW_min2;
    AHL_pwm = CCW_min2;
    AHR_pwm = CW_min2;
  }
  if ((LX >= 75) && (LX <= 94) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //Y_Left_mid1
  {
    FHL_pwm = CW_mid1;
    FHR_pwm = CCW_mid1;
    AHL_pwm = CCW_mid1;
    AHR_pwm = CW_mid1;
  }
  if ((LX >= 55) && (LX <= 74) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //Y_Left_mid2
  {
    FHL_pwm = CW_mid2;
    FHR_pwm = CCW_mid2;
    AHL_pwm = CCW_mid2;
    AHR_pwm = CW_mid2;
  }
  if ((LX >= 25) && (LX <= 54) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //Y_Left_max1
  {
    FHL_pwm = CW_max1;
    FHR_pwm = CCW_max1;
    AHL_pwm = CCW_max1;
    AHR_pwm = CW_max1;
  }
  if ((LX >= 0) && (LX <= 24) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //Y_Left_max2
  {
    FHL_pwm = CW_max2;
    FHR_pwm = CCW_max2;
    AHL_pwm = CCW_max2;
    AHR_pwm = CW_max2;
  }
  if ((LX >= (LX_N + T)) && (LX <= 140) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //Y_Right_min1
  {
    FHL_pwm = CCW_min1;
    FHR_pwm = CW_min1;
    AHL_pwm = CW_min1;
    AHR_pwm = CCW_min1;
  }
  if ((LX >= 141) && (LX <= 160) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //Y_Right_min2
  {
    FHL_pwm = CCW_min2;
    FHR_pwm = CW_min2;
    AHL_pwm = CW_min2;
    AHR_pwm = CCW_min2;
  }
  if ((LX >= 161) && (LX <= 180) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //Y_Right_mid1
  {
    FHL_pwm = CCW_mid1;
    FHR_pwm = CW_mid1;
    AHL_pwm = CW_mid1;
    AHR_pwm = CCW_mid1;
  }
  if ((LX >= 181) && (LX <= 200) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //Y_Right_mid2
  {
    FHL_pwm = CCW_mid2;
    FHR_pwm = CW_mid2;
    AHL_pwm = CW_mid2;
    AHR_pwm = CCW_mid2;
  }
  if ((LX >= 201) && (LX <= 230) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //Y_Right_max1
  {
    FHL_pwm = CCW_max1;
    FHR_pwm = CW_max1;
    AHL_pwm = CW_max1;
    AHR_pwm = CCW_max1;
  }
  if ((LX >= 231) && (LX <= 255) && (LY <= (LY_N + T)) && (LY >= (LY_N - T))) //Y_Right_max2
  {
    FHL_pwm = CCW_max2;
    FHR_pwm = CW_max2;
    AHL_pwm = CW_max2;
    AHR_pwm = CCW_max2;
  }
  if ((LY <= (LY_N - T)) && (LY >= 115) && (LX <= (LX_N + T)) && (LX >= (LX_N - T))) //H_Fwd_min1
  {
    FHL_pwm = CW_min1;
    FHR_pwm = CW_min1;
    AHL_pwm = CW_min1;
    AHR_pwm = CW_min1;
  }
  if ((LY <= 114) && (LY >= 95) && (LX <= (LX_N + T)) && (LX >= (LX_N - T))) //H_Fwd_min2
  {
    FHL_pwm = CW_min2;
    FHR_pwm = CW_min2;
    AHL_pwm = CW_min2;
    AHR_pwm = CW_min2;
  }
  if ((LY <= 94) && (LY >= 75) && (LX <= (LX_N + T)) && (LX >= (LX_N - T))) //H_Fwd_mid1
  {
    FHL_pwm = CW_mid1;
    FHR_pwm = CW_mid1;
    AHL_pwm = CW_mid1;
    AHR_pwm = CW_mid1;
  }
  if ((LY <= 74) && (LY >= 55) && (LX <= (LX_N + T)) && (LX >= (LX_N - T))) //H_Fwd_mid2
  {
    FHL_pwm = CW_mid2;
    FHR_pwm = CW_mid2;
    AHL_pwm = CW_mid2;
    AHR_pwm = CW_mid2;
  }
  if ((LY <= 54) && (LY >= 25) && (LX <= (LX_N + T)) && (LX >= (LX_N - T))) //H_Fwd_max1
  {
    FHL_pwm = CW_max1;
    FHR_pwm = CW_max1;
    AHL_pwm = CW_max1;
    AHR_pwm = CW_max1;
  }
  if ((LY <= 24) && (LY >= 0) && (LX <= (LX_N + T)) && (LX >= (LX_N - T))) //H_Fwd_max2
  {
    FHL_pwm = CW_max2;
    FHR_pwm = CW_max2;
    AHL_pwm = CW_max2;
    AHR_pwm = CW_max2;
  }
  if ((LY >= (LY_N + T)) && (LY <= 140) && (LX <= (LX_N + T)) && (LX >= (LX_N - T))) //H_Rev_min1
  {
    FHL_pwm = CCW_min1;
    FHR_pwm = CCW_min1;
    AHL_pwm = CCW_min1;
    AHR_pwm = CCW_min1;
  }
  if ((LY >= 141) && (LY <= 160) && (LX <= (LX_N + T)) && (LX >= (LX_N - T))) //H_Rev_min2
  {
    FHL_pwm = CCW_min2;
    FHR_pwm = CCW_min2;
    AHL_pwm = CCW_min2;
    AHR_pwm = CCW_min2;
  }
  if ((LY >= 161) && (LY <= 180) && (LX <= (LX_N + T)) && (LX >= (LX_N - T))) //H_Rev_mid1
  {
    FHL_pwm = CCW_mid1;
    FHR_pwm = CCW_mid1;
    AHL_pwm = CCW_mid1;
    AHR_pwm = CCW_mid1;
  }
  if ((LY >= 181) && (LY <= 200) && (LX <= (LX_N + T)) && (LX >= (LX_N - T))) //H_Rev_mid2
  {
    FHL_pwm = CCW_mid2;
    FHR_pwm = CCW_mid2;
    AHL_pwm = CCW_mid2;
    AHR_pwm = CCW_mid2;
  }
  if ((LY >= 201) && (LY <= 230) && (LX <= (LX_N + T)) && (LX >= (LX_N - T))) //H_Rev_max1
  {
    FHL_pwm = CCW_max1;
    FHR_pwm = CCW_max1;
    AHL_pwm = CCW_max1;
    AHR_pwm = CCW_max1;
  }
  if ((LY >= 231) && (LY <= 255) && (LX <= (LX_N + T)) && (LX >= (LX_N - T))) //H_Rev_max2
  {
    FHL_pwm = CCW_max2;
    FHR_pwm = CCW_max2;
    AHL_pwm = CCW_max2;
    AHR_pwm = CCW_max2;
  }
  ///LJS_diagonal
  if ((LX >= (LX_N + T)) && (LX <= 139) && (LY <= (LY_N - T)) && (LY >= 116)) //LQ1_min1
  {
    FHL_pwm = CW_min1;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_min1;
  }
  if ((LX >= 140) && (LX <= 159) && (LY <= 115) && (LY >= 96)) //LQ1_min2
  {
    FHL_pwm = CW_min2;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_min2;
  }
  if ((LX >= 160) && (LX <= 179) && (LY <= 95) && (LY >= 76)) //LQ1_med1
  {
    FHL_pwm = CW_mid1;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_mid1;
  }
  if ((LX >= 180) && (LX <= 199) && (LY <= 75) && (LY >= 56)) //LQ1_med2
  {
    FHL_pwm = CW_mid2;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_mid2;
  }
  if ((LX >= 200) && (LX <= 229) && (LY <= 55) && (LY >= 26)) //LQ1_max1
  {
    FHL_pwm = CW_max1;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_max1;
  }
  if ((LX >= 230) && (LX <= 255) && (LY <= 25) && (LY >= 0)) //LQ1_max2
  {
    FHL_pwm = CW_max2;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_max2;
  }
  ////
  if ((LX >= (LX_N + T)) && (LX <= 140) && (LY >= (LY_N + T)) && (LY <= 140)) //LQ2_mi1
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_min1;
    AHL_pwm = CCW_min1;
    AHR_pwm = AHR_N;
  }
  if ((LX >= 141) && (LX <= 160) && (LY >= 141) && (LY <= 160)) //LQ2_min2
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_min2;
    AHL_pwm = CCW_min2;
    AHR_pwm = AHR_N;
  }
  if ((LX >= 161) && (LX <= 180) && (LY >= 161) && (LY <= 180)) //LQ2_med1
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_mid1;
    AHL_pwm = CCW_mid1;
    AHR_pwm = AHR_N;
  }
  if ((LX >= 181) && (LX <= 200) && (LY >= 181) && (LY <= 200)) //LQ2_med2
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_mid2;
    AHL_pwm = CCW_mid2;
    AHR_pwm = AHR_N;
  }
  if ((LX >= 201) && (LX <= 230) && (LY >= 201) && (LY <= 230)) //LQ2_max1
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_max1;
    AHL_pwm = CCW_max1;
    AHR_pwm = AHR_N;
  }
  if ((LX >= 231) && (LX <= 255) && (LY >= 231) && (LY <= 255)) //LQ2_max2
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_max2;
    AHL_pwm = CCW_max2;
    AHR_pwm = AHR_N;
  }
  ////
  if ((LX >= 115) && (LX <= (LX_N - T)) && (LY <= 140) && (LY >= (LY_N + T))) //LQ3_min1
  {
    FHL_pwm = CCW_min1;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_min1;
  }
  if ((LX >= 95) && (LX <= 114) && (LY <= 160) && (LY >= 141)) //LQ3_min2
  {
    FHL_pwm = CCW_min2;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_min2;
  }
  if ((LX >= 75) && (LX <= 94) && (LY <= 180) && (LY >= 161)) //LQ3_med1
  {
    FHL_pwm = CCW_mid1;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_mid1;
  }
  if ((LX >= 55) && (LX <= 74) && (LY <= 200) && (LY >= 181)) //LQ3_med2
  {
    FHL_pwm = CCW_mid2;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_mid2;
  }
  if ((LX >= 25) && (LX <= 54) && (LY <= 230) && (LY >= 201)) //LQ3_max1
  {
    FHL_pwm = CCW_max1;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_max1;
  }
  if ((LX >= 0) && (LX <= 24) && (LY <= 255) && (LY >= 231)) //LQ3_max2
  {
    FHL_pwm = CCW_max2;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_max2;
  }
  ////
  if ((LX <= (LX_N - T)) && (LX >= 115) && (LY <= (LY_N - T)) && (LY >= 115)) //LQ4_min1
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_min1;
    AHL_pwm = CW_min1;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 114) && (LX >= 95) && (LY <= 114) && (LY >= 95)) //LQ4_min2
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_min2;
    AHL_pwm = CW_min2;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 94) && (LX >= 75) && (LY <= 94) && (LY >= 75)) //LQ4_med1
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_mid1;
    AHL_pwm = CW_mid1;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 74) && (LX >= 55) && (LY <= 74) && (LY >= 55)) //LQ4_med2
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_mid2;
    AHL_pwm = CW_mid2;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 54) && (LX >= 25) && (LY <= 54) && (LY >= 25)) //LQ4_max1
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_max1;
    AHL_pwm = CW_max1;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 24) && (LX >= 0) && (LY <= 24) && (LY >= 0)) //LQ4_max2
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_max2;
    AHL_pwm = CW_max2;
    AHR_pwm = AHR_N;
  }
  //RIGHT JS
  if ((RX >= 115) && (RX <= (RX_N - T)) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //R_Left_min1
  {
    AVL_pwm = CCW_min1;
    AVR_pwm =  CCW_min1;
  }
  if ((RX >= 95) && (RX <= 114) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //R_Left_min2
  {
    AVL_pwm = CCW_min2;
    AVR_pwm =  CCW_min2;
  }
  if ((RX >= 75) && (RX <= 94) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //R_Left_med1
  {
    AVL_pwm = CCW_mid1;
    AVR_pwm =  CCW_mid1;
  }
  if ((RX >= 55) && (RX <= 74) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //R_Left_med2
  {
    AVL_pwm = CCW_mid2;
    AVR_pwm =  CCW_mid2;
  }
  if ((RX >= 25) && (RX <= 54) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //R_Left_max1
  {
    AVL_pwm = CCW_max1;
    AVR_pwm =  CCW_max1;
  }
  if ((RX >= 0) && (RX <= 24) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //R_Left_max2
  {
    AVL_pwm = CCW_max2;
    AVR_pwm =  CCW_max2;
  }
  ////
  if ((RX >= (RX_N + T)) && (RX <= 140) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //R_Right_min1
  {
    AVL_pwm = CW_min1;
    AVR_pwm =  CW_min1;
  }
  if ((RX >= 141) && (RX <= 160) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //R_Right_min2
  {
    AVL_pwm = CW_min2;
    AVR_pwm =  CW_min2;
  }
  if ((RX >= 161) && (RX <= 180) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //R_Right_med1
  {
    AVL_pwm = CW_mid1;
    AVR_pwm =  CW_mid1;
  }
  if ((RX >= 181) && (RX <= 200) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //R_Right_med2
  {
    AVL_pwm = CW_mid2;
    AVR_pwm =  CW_mid2;
  }
  if ((RX >= 201) && (RX <= 230) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //R_Right_max1
  {
    AVL_pwm = CW_max1;
    AVR_pwm =  CW_max1;
  }
  if ((RX >= 231) && (RX <= 255) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //R_Right_max2
  {
    AVL_pwm = CW_max2;
    AVR_pwm =  CW_max2;
  }
  ////
  if ((RY <= (RY_N - T)) && (RY >= 115) && (RX >= (RX_N - T)) && (RX <= (RX_N + T))) //Surf_min1
  {
    AVL_pwm = CW_min1;
    AVR_pwm =  CCW_min1;
  }
  if ((RY <= 114) && (RY >= 95) && (RX >= (RX_N - T)) && (RX <= (RX_N + T))) //Surf_min2
  {
    AVL_pwm = CW_min2;
    AVR_pwm =  CCW_min2;
  }
  if ((RY <= 94) && (RY >= 75) && (RX >= (RX_N - T)) && (RX <= (RX_N + T))) //Surf_mid1
  {
    AVL_pwm = CW_mid1;
    AVR_pwm =  CCW_mid1;
  }
  if ((RY <= 74) && (RY >= 55) && (RX >= (RX_N - T)) && (RX <= (RX_N + T))) //Surf_mid2
  {
    AVL_pwm = CW_mid2;
    AVR_pwm =  CCW_mid2;
  }
  if ((RY <= 54) && (RY >= 25) && (RX >= (RX_N - T)) && (RX <= (RX_N + T))) //Surf_max1
  {
    AVL_pwm = CW_max1;
    AVR_pwm =  CCW_max1;
  }
  if ((RY <= 24) && (RY >= 0) && (RX >= (RX_N - T)) && (RX <= (RX_N + T))) //Surf_max2
  {
    AVL_pwm = CW_max2;
    AVR_pwm =  CCW_max2;
  }
  ////
  if ((RY >= (RY_N + T)) && (RY <= 140) && (RX >= (RX_N - T)) && (RX <= (RX_N + T))) //Dive_min1
  {
    AVL_pwm = CCW_min1;
    AVR_pwm =  CW_min1;
  }
  if ((RY >= 141) && (RY <= 160) && (RX >= (RX_N - T)) && (RX <= (RX_N + T))) //Dive_min2
  {
    AVL_pwm = CCW_min2;
    AVR_pwm =  CW_min2;
  }
  if ((RY >= 161) && (RY <= 180) && (RX >= (RX_N - T)) && (RX <= (RX_N + T))) //Dive_mid1
  {
    AVL_pwm = CCW_mid1;
    AVR_pwm =  CW_mid1;
  }
  if ((RY >= 181) && (RY <= 200) && (RX >= (RX_N - T)) && (RX <= (RX_N + T))) //Dive_mid2
  {
    AVL_pwm = CCW_mid2;
    AVR_pwm =  CW_mid2;
  }
  if ((RY >= 201) && (RY <= 230) && (RX >= (RX_N - T)) && (RX <= (RX_N + T))) //Dive_max1
  {
    AVL_pwm = CCW_max1;
    AVR_pwm =  CW_max1;
  }
  if ((RY >= 231) && (RY <= 255) && (RX >= (RX_N - T)) && (RX <= (RX_N + T))) //Dive_max2
  {
    AVL_pwm = CCW_max2;
    AVR_pwm =  CW_max2;
  }
  ////
  if ((RRU >= 5) && (RLU <= 3) && (RX <= (RX_N + T)) && (RX >= (RX_N - T)) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //Roll_Right_up
  {
    AVL_pwm = CW_max2;

  }
  ////
  if ((RLU >= 5) && (RRU <= 3) && (RX <= (RX_N + T)) && (RX >= (RX_N - T)) && (RY <= (RY_N + T)) && (RY >= (RY_N - T))) //Roll_Left_up
  {
    AVR_pwm = CCW_max2;
  }
  ////
  //RJS_diagonal
  if ((RX >= (RX_N + T)) && (RX <= 140) && (RY <= (RY_N - T)) && (RY >= 115)) //Y_Right_Surf_min1
  {
    FHL_pwm = CCW_min1;
    FHR_pwm = CW_min1;
    AHL_pwm = CW_min1;
    AHR_pwm = CCW_min1;
    AVL_pwm = CW_min1;
    AVR_pwm =  CCW_min1;
  }
  if ((RX >= 141) && (RX <= 160) && (RY <= 114) && (RY >= 95)) //Y_Right_Surf_min2
  {
    FHL_pwm = CCW_min2;
    FHR_pwm = CW_min2;
    AHL_pwm = CW_min2;
    AHR_pwm = CCW_min2;
    AVL_pwm = CW_min2;
    AVR_pwm =  CCW_min2;
  }
  if ((RX >= 161) && (RX <= 180) && (RY <= 94) && (RY >= 75)) //Y_Right_Surf_mid1
  {
    FHL_pwm = CCW_mid1;
    FHR_pwm = CW_mid1;
    AHL_pwm = CW_mid1;
    AHR_pwm = CCW_mid1;
    AVL_pwm = CW_mid1;
    AVR_pwm =  CCW_mid1;
  }
  if ((RX >= 181) && (RX <= 200) && (RY <= 74) && (RY >= 55)) //Y_Right_Surf_mid2
  {
    FHL_pwm = CCW_mid2;
    FHR_pwm = CW_mid2;
    AHL_pwm = CW_mid2;
    AHR_pwm = CCW_mid2;
    AVL_pwm = CW_mid2;
    AVR_pwm =  CCW_mid2;
  }
  if ((RX >= 201) && (RY <= 230) && (RY <= 54) && (RY >= 25)) //Y_Right_Surf_max1
  {
    FHL_pwm = CCW_max1;
    FHR_pwm = CW_max1;
    AHL_pwm = CW_max1;
    AHR_pwm = CCW_max1;
    AVL_pwm = CW_max1;
    AVR_pwm =  CCW_max1;
  }
  if ((RX >= 231) && (RY <= 255) && (RY <= 24) && (RY >= 0)) //Y_Right_Surf_max2
  {
    FHL_pwm = CCW_max2;
    FHR_pwm = CW_max2;
    AHL_pwm = CW_max2;
    AHR_pwm = CCW_max2;
    AVL_pwm = CW_max2;
    AVR_pwm =  CCW_max2;
  }
  ////
  if ((RX >= (RX_N + T)) && (RX <= 140) && (RY >= (RY_N + T)) && (RY <= 140)) //Y_Right_Dive_min1
  {
    FHL_pwm = CCW_min1;
    FHR_pwm = CW_min1;
    AHL_pwm = CW_min1;
    AHR_pwm = CCW_min1;
    AVL_pwm = CCW_min1;
    AVR_pwm =  CW_min1;
  }
  if ((RX >= 141) && (RX <= 160) && (RY >= 141) && (RY <= 160)) //Y_Right_Dive_min2
  {
    FHL_pwm = CCW_min2;
    FHR_pwm = CW_min2;
    AHL_pwm = CW_min2;
    AHR_pwm = CCW_min2;
    AVL_pwm = CCW_min2;
    AVR_pwm =  CW_min2;
  }
  if ((RX >= 161) && (RX <= 180) && (RY >= 161) && (RY <= 180)) //Y_Right_Dive_med1
  {
    FHL_pwm = CCW_mid1;
    FHR_pwm = CW_mid1;
    AHL_pwm = CW_mid1;
    AHR_pwm = CCW_mid1;
    AVL_pwm = CCW_mid1;
    AVR_pwm =  CW_mid1;
  }
  if ((RX >= 181) && (RX <= 200) && (RY >= 181) && (RY <= 200)) //Y_Right_Dive_med2
  {
    FHL_pwm = CCW_mid2;
    FHR_pwm = CW_mid2;
    AHL_pwm = CW_mid2;
    AHR_pwm = CCW_mid2;
    AVL_pwm = CCW_mid2;
    AVR_pwm =  CW_mid2;
  }
  if ((RX >= 201) && (RX <= 230) && (RY >= 201) && (RY <= 230)) //Y_Right_Dive_max1
  {
    FHL_pwm = CCW_max1;
    FHR_pwm = CW_max1;
    AHL_pwm = CW_max1;
    AHR_pwm = CCW_max1;
    AVL_pwm = CCW_max1;
    AVR_pwm =  CW_max1;
  }
  if ((RX >= 231) && (RX <= 255) && (RY >= 231) && (RY <= 255)) //Y_Right_Dive_max2
  {
    FHL_pwm = CCW_max2;
    FHR_pwm = CW_max2;
    AHL_pwm = CW_max2;
    AHR_pwm = CCW_max2;
    AVL_pwm = CCW_max2;
    AVR_pwm =  CW_max2;
  }
  ////
  if ((RX >= 115) && (RX <= (RX_N - T)) && (RY <= 140) && (RY >= (RY_N + T))) //Y_Left_Dive_min1
  {
    FHL_pwm = CW_min1;
    FHR_pwm = CCW_min1;
    AHL_pwm = CCW_min1;
    AHR_pwm = CW_min1;
    AVL_pwm = CCW_min1;
    AVR_pwm = CW_min1;
  }
  if ((RX >= 95) && (RX <= 114) && (RY <= 160) && (RY >= 141)) //Y_Left_Dive_min2
  {
    FHL_pwm = CW_min2;
    FHR_pwm = CCW_min2;
    AHL_pwm = CCW_min2;
    AHR_pwm = CW_min2;
    AVL_pwm = CCW_min2;
    AVR_pwm = CW_min2;
  }
  if ((RX >= 75) && (RX <= 94) && (RY <= 180) && (RY >= 161)) //Y_Left_Dive_med1
  {
    FHL_pwm = CW_mid1;
    FHR_pwm = CCW_mid1;
    AHL_pwm = CCW_mid1;
    AHR_pwm = CW_mid1;
    AVL_pwm = CCW_mid1;
    AVR_pwm = CW_mid1;
  }
  if ((RX >= 55) && (RX <= 74) && (RY <= 200) && (RY >= 181)) //Y_Left_Dive_med2
  {
    FHL_pwm = CW_mid2;
    FHR_pwm = CCW_mid2;
    AHL_pwm = CCW_mid2;
    AHR_pwm = CW_mid2;
    AVL_pwm = CCW_mid2;
    AVR_pwm = CW_mid2;
  }
  if ((RX >= 25) && (RX <= 54) && (RY <= 230) && (RY >= 201)) //Y_Left_Dive_max1
  {
    FHL_pwm = CW_max1;
    FHR_pwm = CCW_max1;
    AHL_pwm = CCW_max1;
    AHR_pwm = CW_max1;
    AVL_pwm = CCW_max1;
    AVR_pwm = CW_max1;
  }
  if ((RX >= 0) && (RX <= 24) && (RY <= 255) && (RY >= 231)) //Y_Left_Dive_max2
  {
    FHL_pwm = CW_max2;
    FHR_pwm = CCW_max2;
    AHL_pwm = CCW_max2;
    AHR_pwm = CW_max2;
    AVL_pwm = CCW_max2;
    AVR_pwm = CW_max2;
  }
  ////
  if ((RX <= (RX_N - T)) && (RX >= 115) && (RY <= (RY_N - T)) && (RY >= 115)) //Y_Left_Surf_min1
  {
    FHL_pwm = CW_min1;
    FHR_pwm = CCW_min1;
    AHL_pwm = CCW_min1;
    AHR_pwm = CW_min1;
    AVL_pwm = CW_min1;
    AVR_pwm =  CCW_min1;
  }
  if ((RX <= 114) && (RX >= 95) && (RY <= 114) && (RY >= 95)) //Y_Left_Surf_min2
  {
    FHL_pwm = CW_min2;
    FHR_pwm = CCW_min2;
    AHL_pwm = CCW_min2;
    AHR_pwm = CW_min2;
    AVL_pwm = CW_min2;
    AVR_pwm =  CCW_min2;
  }
  if ((RX <= 94) && (RX >= 75) && (RY <= 94) && (RY >= 75)) //Y_Left_Surf_med1
  {
    FHL_pwm = CW_mid1;
    FHR_pwm = CCW_mid1;
    AHL_pwm = CCW_mid1;
    AHR_pwm = CW_mid1;
    AVL_pwm = CW_mid1;
    AVR_pwm =  CCW_mid1;
  }
  if ((RX <= 74) && (RX >= 55) && (RY <= 74) && (RY >= 55)) //Y_Left_Surf_med2
  {
    FHL_pwm = CW_mid2;
    FHR_pwm = CCW_mid2;
    AHL_pwm = CCW_mid2;
    AHR_pwm = CW_mid2;
    AVL_pwm = CW_mid2;
    AVR_pwm =  CCW_mid2;
  }
  if ((RX <= 54) && (RY <= 25) && (RY <= 54) && (RY >= 25)) //Y_Left_Surf_max1
  {
    FHL_pwm = CW_max1;
    FHR_pwm = CCW_max1;
    AHL_pwm = CCW_max1;
    AHR_pwm = CW_max1;
    AVL_pwm = CW_max1;
    AVR_pwm =  CCW_max1;
  }
  if ((RX <= 24) && (RX >= 0) && (RY <= 24) && (RY >= 0)) //Y_Left_Surf_max2
  {
    FHL_pwm = CW_max2;
    FHR_pwm = CCW_max2;
    AHL_pwm = CCW_max2;
    AHR_pwm = CW_max2;
    AVL_pwm = CW_max2;
    AVR_pwm =  CCW_max2;
  }

  Tx_data.FHL_pwm = FHL_pwm;
  Tx_data.FHR_pwm = FHR_pwm;
  Tx_data.AHL_pwm = AHL_pwm;
  Tx_data.AHR_pwm = AHR_pwm;
  Tx_data.AVL_pwm = AVL_pwm;
  Tx_data.AVR_pwm = AVR_pwm;
  //Hold_read

  //Toggle HOLD CRO_P
  if (ps.CROSS == 0) {
    requireddir = Rx_data.ROVDir;
    requireddepth = Rx_data.ROVDepth;
  }
  if (ps.CROSS_Pressure >= p_max) {
    Hold_OnOff = Hold_OnOff;
  }
  //Toggle SQR_P
  if (ps.SQUARE_Pressure >= p_max) {
    Tx_data.LED_OnOff = !Tx_data.LED_OnOff;
  }
  //Toggle CAM CIR_P
  if (ps.CIRCLE_Pressure >= p_max) {
    Tx_data.CAM_OnOff = !Tx_data.CAM_OnOff;
  }
  //CAM_TILT begin
  if (ps.UP_Pressure >= p_max && ps.DOWN_Pressure <= p_min)
  {
    Tx_data.Cam_tilt = Tx_data.Cam_tilt + T_step;
  }
  if (ps.DOWN_Pressure >= p_max && ps.UP_Pressure <= p_min)
  {
    Tx_data.Cam_tilt = Tx_data.Cam_tilt - T_step;
  }
  Tx_data.Cam_tilt = constrain(Tx_data.Cam_tilt, T_min, T_max);

  // LIGHT
  int LED_Pot_Value = analogRead(A3);
  if (LED_Pot_Value >= 0 && LED_Pot_Value <= 106)
  {
    Tx_data.LED_bright = 30;
  }
  if (LED_Pot_Value >= 107 && LED_Pot_Value <= 208)
  {
    Tx_data.LED_bright = 55;
  }
  if (LED_Pot_Value >= 209 && LED_Pot_Value <= 310)
  {
    Tx_data.LED_bright = 80;
  }
  if (LED_Pot_Value >= 311 && LED_Pot_Value <= 412)
  {
    Tx_data.LED_bright = 105;
  }
  if (LED_Pot_Value >= 413 && LED_Pot_Value <= 514)
  {
    Tx_data.LED_bright = 130;
  }
  if (LED_Pot_Value >= 515 && LED_Pot_Value <= 616)
  {
    Tx_data.LED_bright = 155;
  }
  if (LED_Pot_Value >= 617 && LED_Pot_Value <= 718)
  {
    Tx_data.LED_bright = 180;
  }
  if (LED_Pot_Value >= 719 && LED_Pot_Value <= 820)
  {
    Tx_data.LED_bright = 205;
  }
  if (LED_Pot_Value >= 821 && LED_Pot_Value <= 922)
  {
    Tx_data.LED_bright = 230;
  }
  if (LED_Pot_Value >= 923 && LED_Pot_Value <= 1024)
  {
    Tx_data.LED_bright = 255;
  }

  //ET IN_OUT
  ETout.sendData();
  //TOP<-ROV
  for (uint8_t i = 0; i < 5; i++) {
    ETin.receiveData();
    delay(10);
  }

  //DISP
  if (Disp >= p_max)
  {
    //ROV STATUS
    if (Rx_data.ROVPing == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ROV LINK OKAY");
      lcd.setCursor(1, 1);
      lcd.print("TOP>>>O<<<ROV");
      delay(25);
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ROV LINK LOST");
      lcd.setCursor(1, 1);
      lcd.print("TOP--|X|--ROV");
      ps.VibrateMotors(ml, mr);
      delay(25);
    }
    //JS_POS
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(":LX::LY::RX::RY:");
    lcd.setCursor(1, 1);
    lcd.print(LX);
    lcd.print("\t");
    lcd.print(LY);
    lcd.print("\t");
    lcd.print(RX);
    lcd.print("\t");
    lcd.print(RY);
    delay(Disp_wait);

    //CAM
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CAM::TILT");
    lcd.setCursor(1, 1);
    if (Tx_data.CAM_OnOff == 1) {
      lcd.print(" ON ");
    }
    else {
      lcd.print(" OFF");
    }
    lcd.print("\t");
    lcd.print("\t");
    lcd.print("\t");
    lcd.print(Tx_data.Cam_tilt);
    delay(Disp_wait);

    //LIGHTS
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Lights Level");
    lcd.setCursor(1, 1);
    lcd.print("\t");
    lcd.print("\t");
    lcd.print(Tx_data.LED_bright, DEC);
    lcd.setCursor(9, 1);
    if (Tx_data.LED_OnOff == 1) {
      lcd.print(" ON ");
    }
    else {
      lcd.print(" OFF");
    }
    delay(Disp_wait);

    //HOLD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("HOLD_Head_Depth");
    lcd.setCursor(5, 1);
    if (Hold_OnOff == 1) {
      lcd.print(" ON ");
    }
    else {
      lcd.print(" OFF");
    }
    delay(Disp_wait);

    //ROV BATT VOLT
    if (Rx_data.ROVBattVolt <= 9.8) //9.6V 3s lipo low cut-off
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WARNING !!");
      lcd.setCursor(0, 1);
      lcd.print("LOW VOLT");
      delay(Disp_wait);
    }
    if (Rx_data.ROVBattVolt >= 12.4) //12.6V 3s lipo high cut-off
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WARNING !!");
      lcd.setCursor(0, 1);
      lcd.print("HIGH VOLT");
      delay(Disp_wait);
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" V:");
    lcd.setCursor(5, 0);
    lcd.print(Rx_data.ROVBattVolt);
    lcd.setCursor(10, 0);
    lcd.print("Volts");
    delay(Disp_wait);

    //ROV BATT AMPS
    if (Rx_data.ROVAmp >= 10.5)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WARNING !!");
      lcd.setCursor(0, 1);
      lcd.print("HIGH AMPS");
      delay(Disp_wait);
    }
    lcd.setCursor(0, 1);
    lcd.print(" I:");
    lcd.setCursor(5, 1);
    lcd.print(Rx_data.ROVAmp);
    lcd.setCursor(10, 1);
    lcd.print("Amps");
    delay(Disp_wait);

    //ROV ONBD TEMP
    if (Rx_data.ROV_ON_Temp >= 28.4) //normal elecx operating Tmax=29.4C
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WARNING !!");
      lcd.setCursor(0, 1);
      lcd.print("IN TEMP HIGH");
      delay(Disp_wait);
    }
    if (Rx_data.ROV_ON_Temp <= 11.0) { //normal elecx operating Tmin=10C
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WARNING !!");
      lcd.setCursor(0, 1);
      lcd.print("IN TEMP LOW");
      delay(Disp_wait);
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("In T:");
    lcd.setCursor(5, 0);
    lcd.print(Rx_data.ROV_ON_Temp);
    lcd.setCursor(10, 0);
    lcd.print("*C");
    delay(Disp_wait);

    //ROV ONBD PRESS
    if (Rx_data.ROV_ON_Press >= 44.0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WARNING !!");
      lcd.setCursor(0, 1);
      lcd.print("IN PRES HIGH");
      delay(Disp_wait);
    }
    lcd.setCursor(0, 1);
    lcd.print("In P:");
    lcd.setCursor(7, 1);
    lcd.print(Rx_data.ROV_ON_Press);
    lcd.setCursor(12, 1);
    lcd.print("Psi");
    delay(Disp_wait);

    //ROV EXT TEMP
    if (Rx_data.ROV_EX_Temp >= 35.0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WARNING !!");
      lcd.setCursor(0, 1);
      lcd.print("EX TEMP HIGH");
      delay(Disp_wait);
    }
    if (Rx_data.ROV_EX_Temp <= 5.0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WARNING !!");
      lcd.setCursor(0, 1);
      lcd.print("EX TEMP LOW");
      delay(Disp_wait);
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ex T:");
    lcd.setCursor(6, 0);
    lcd.print(Rx_data.ROV_EX_Temp);
    lcd.setCursor(11, 0);
    lcd.print("*C");
    delay(Disp_wait);

    //ROV EXT Pres//
    if (Rx_data.ROV_EX_Press >= 25.93)//P= (25.93psi fresh) or (26.63psi salt) water at 60ft depth
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WARNING !!");
      lcd.setCursor(0, 1);
      lcd.print("HIGH EX PRES");
      delay(Disp_wait);
    }
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Ex P:");
    lcd.setCursor(6, 1);
    lcd.print(Rx_data.ROV_EX_Press);
    lcd.setCursor(12, 1);
    lcd.print("Psi");
    delay(Disp_wait);

    //ROV DEPTH
    if (Rx_data.ROVDepth >= 60.0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WARNING !!");
      lcd.setCursor(0, 1);
      lcd.print("DEPTH > 60Ft");
      delay(Disp_wait);
    }
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Depth:");
    lcd.setCursor(7, 1);
    lcd.print(Rx_data.ROVDepth);
    lcd.setCursor(12, 1);
    lcd.print("Ft.");
    delay(Disp_wait);

    //ROV DIR
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("HeadDir:");
    lcd.setCursor(8, 0);
    lcd.print(Rx_data.ROVDir);
    lcd.setCursor(12, 0);
    lcd.print("Deg");
    delay(Disp_wait);

  }
}
