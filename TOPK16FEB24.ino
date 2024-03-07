/*
  MENCO MEEN TOP V5_0
  6 X BLDC THRUSTERS
  MOD ON 16-FEB-2024(TESTED - ok)
  MCU:UNO_R3
  1 X PS2 SHLD
  1 X I2C_16X2_LCD DISP
  COMM RS422_MAX490
  =================PS2 GAME CTRL CONTROLS=======================
     CAM_TILT         L_XY             R_XY     SQR_BTN      CRO_P
     UP_PRESS         H_FF             SURF    HEADLIGHTS     HOLD
       A                A                A       ON_OFF     ON_OFF
       |                |                |
       G        YAW L<--L-->R   ROLL L<--R-->R  CIR_BTN
       |                |                |        CAM
       V                V                V       ON_OFF
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
#define Disp_wait 115

EasyTransfer ETin, ETout;
SmartElexPs2Shield ps(9, 8);

//LCD I2C address
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int LX_N = 123;
const int LY_N = 123;
const int RX_N = 133;
const int RY_N = 122;
const int T = 3;

const int CW_max = 180;
const int CW_mid = 145;
const int CW_min = 115;

const int CCW_max = 0;
const int CCW_mid = 55;
const int CCW_min = 25;

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
  long ROV_EX_Press;//psi
  long ROVDepth; //Ft
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
  RLD = ps.L2_Pressure;//R_left_dn
  RRD = ps.R2_Pressure;//R_right_dn
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
    //FVL_pwm = FVL_N;
    //FVR_pwm = FVR_N;
    AVL_pwm = AVL_N;
    AVR_pwm = AVR_N;

  }
  if ((LX >= 80) && (LX <= (LX_N - T))) //Y_Left_min
  {
    FHL_pwm = CW_min;
    FHR_pwm = CCW_min;
    AHL_pwm = CCW_min;
    AHR_pwm = CW_min;
  }
  if ((LX >= 25) && (LX <= 79)) //Y_Left_mid
  {
    FHL_pwm = CW_mid;
    FHR_pwm = CCW_mid;
    AHL_pwm = CCW_mid;
    AHR_pwm = CW_mid;
  }
  if ((LX >= 0) && (LX <= 24)) //Y_Left_max
  {
    FHL_pwm = CW_max;
    FHR_pwm = CCW_max;
    AHL_pwm = CCW_max;
    AHR_pwm = CW_max;
  }
  if ((LX >= (LX_N + T)) && (LX <= 160)) //Y_Right_min
  {
    FHL_pwm = CCW_min;
    FHR_pwm = CW_min;
    AHL_pwm = CW_min;
    AHR_pwm = CCW_min;
  }
  if ((LX >= 161) && (LX <= 230)) //Y_Right_mid
  {
    FHL_pwm = CCW_mid;
    FHR_pwm = CW_mid;
    AHL_pwm = CW_mid;
    AHR_pwm = CCW_mid;
  }
  if ((LX >= 231) && (LX <= 255)) //Y_Right_max
  {
    FHL_pwm = CCW_max;
    FHR_pwm = CW_max;
    AHL_pwm = CW_max;
    AHR_pwm = CCW_max;
  }
  if ((LY <= (LY_N - T)) && (LY >= 95)) //H_Fwd_min
  {
    FHL_pwm = CW_min;
    FHR_pwm = CW_min;
    AHL_pwm = CW_min;
    AHR_pwm = CW_min;
  }
  if ((LY <= 94) && (LY >= 25)) //H_Fwd_mid
  {
    FHL_pwm = CW_mid;
    FHR_pwm = CW_mid;
    AHL_pwm = CW_mid;
    AHR_pwm = CW_mid;
  }
  if ((LY <= 24) && (LY >= 0)) //H_Fwd_max
  {
    FHL_pwm = CW_max;
    FHR_pwm = CW_max;
    AHL_pwm = CW_max;
    AHR_pwm = CW_max;
  }
  if ((LY >= (LY_N + T)) && (LY <= 160)) //H_Rev_min
  {
    FHL_pwm = CCW_min;
    FHR_pwm = CCW_min;
    AHL_pwm = CCW_min;
    AHR_pwm = CCW_min;
  }
  if ((LY >= 161) && (LY <= 230)) //H_Rev_mid
  {
    FHL_pwm = CCW_mid;
    FHR_pwm = CCW_mid;
    AHL_pwm = CCW_mid;
    AHR_pwm = CCW_mid;
  }
  if ((LY >= 231) && (LY <= 255)) //H_Rev_max
  {
    FHL_pwm = CCW_max;
    FHR_pwm = CCW_max;
    AHL_pwm = CCW_max;
    AHR_pwm = CCW_max;
  }
  ///LJS_diagonal
  if ((LX >= 245) && (LY <= 10)) //LQ1_max
  {
    FHL_pwm = CW_max;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_max;
  }
  if ((LX >= 175) && (LX <= 244) && (LY <= 80) && (LY >= 11) ) //LQ1_med
  {
    FHL_pwm = CW_mid;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_mid;
  }
  if ((LX >= 155) && (LX <= 174) && (LY <= 100) && (LY >= 79) ) //LQ1_min
  {
    FHL_pwm = CW_min;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CW_min;
  }
  if ((LX >= 245) && (LY >= 245)) //LQ2_max
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_max;
    AHL_pwm = CCW_max;
    AHR_pwm = AHR_N;
  }
  if ((LX >= 175) && (LX <= 244) && (LY >= 175) && (LY <= 244)) //LQ2_med
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_mid;
    AHL_pwm = CCW_mid;
    AHR_pwm = AHR_N;
  }
  if ((LX >= 155) && (LX <= 174) && (LY >= 155) && (LY <= 174)) //LQ2_min
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CCW_min;
    AHL_pwm = CCW_min;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 10) && (LY >= 245)) //LQ3_max
  {
    FHL_pwm = CCW_max;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_max;
  }
  if ((LX >= 11) && (LX <= 80) && (LY <= 244) && (LY >= 175)) //LQ3_med
  {
    FHL_pwm = CCW_mid;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_mid;
  }
  if ((LX >= 81) && (LX <= 100) && (LY <= 174) && (LY >= 155)) //LQ3_min
  {
    FHL_pwm = CCW_min;
    FHR_pwm = FHR_N;
    AHL_pwm = AHL_N;
    AHR_pwm = CCW_min;
  }
  if ((LX <= 10) && (LY <= 10)) //LQ4_max
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_max;
    AHL_pwm = CW_max;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 80) && (LX >= 11) && (LY <= 80) && (LY >= 11)) //LQ4_med
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_mid;
    AHL_pwm = CW_mid;
    AHR_pwm = AHR_N;
  }
  if ((LX <= 100) && (LX >= 81) && (LY <= 100) && (LY >= 81)) //LQ4_min
  {
    FHL_pwm = FHL_N;
    FHR_pwm = CW_min;
    AHL_pwm = CW_min;
    AHR_pwm = AHR_N;
  }
  ///
  if ((RX >= 0) && (RX <= (RX_N - T))) //R_Left_med
  {
    AVL_pwm = CCW_mid;
    AVR_pwm = CCW_mid;
  }
  if ((RX >= 0) && (RX <= (RX_N - T))) //R_Left_max
  {
    AVL_pwm = CCW_max;
    AVR_pwm = CCW_max;
  }
  if ((RX >= (RX_N + T)) && (RX <= 255)) //R_Right_med
  {
    AVL_pwm = CW_mid;
    AVR_pwm = CW_mid;
  }
  if ((RX >= (RX_N + T)) && (RX <= 255)) //R_Right_max
  {
    AVL_pwm = CW_max;
    AVR_pwm = CW_max;
  }
  if ((RY <= (RY_N - T)) && (RY >= 0)) //Surf_min
  {
    AVL_pwm = CW_min;
    AVR_pwm = CCW_min;
  }
  if ((RY <= (RY_N - T)) && (RY >= 0)) //Surf_mid
  {
    AVL_pwm = CW_mid;
    AVR_pwm =  CCW_mid;
  }
  if ((RY <= (RY_N - T)) && (RY >= 0)) //Surf_max
  {
    AVL_pwm = CW_max;
    AVR_pwm = CCW_max;
  }
  if ((RY >= (RY_N + T)) && (RY <= 255)) //Dive_min
  {
    AVL_pwm = CCW_min;
    AVR_pwm = CW_min;
  }
  if ((RY >= (RY_N + T)) && (RY <= 255)) //Dive_mid
  {
    AVL_pwm = CCW_mid;
    AVR_pwm = CW_mid;
  }
  if ((RY >= (RY_N + T)) && (RY <= 255)) //Dive_max
  {
    AVL_pwm = CCW_max;
    AVR_pwm = CW_max;
  }

  if ((RRU >= 5) && (RLU <= 3) && (RX <= (RX_N + T)) && (RX >= (RX_N - T)) ) //Roll_Right_up
  {
    AVL_pwm = CW_max;

  }
  if ((RLU >= 5) && (RRU <= 3) && (RX <= (RX_N + T)) && (RX >= (RX_N - T))) //Roll_Left_up
  {
    AVR_pwm = CCW_max;
  }
  if ((RRD >= 5) && (RLD <= 3) && (RX <= (RX_N + T)) && (RX >= (RX_N - T)) ) //Roll_Right_dn
  {
    AVL_pwm = CCW_max;

  }
  if ((RLD >= 5) && (RRD <= 3) && (RX <= (RX_N + T)) && (RX >= (RX_N - T))) //Roll_Left_dn
  {
    AVR_pwm = CW_max;
  }
  ///RJS_diagonal
  if ((RX >= 245) && (RY <= 10)) //Y_Right_Surf_max
  {
    FHL_pwm = CCW_max;
    FHR_pwm = CW_max;
    AHL_pwm = CW_max;
    AHR_pwm = CCW_max;
    AVL_pwm = CW_max;
    AVR_pwm =  CCW_max;
  }
  if ((RX >= 175) && (RX <= 244) && (RY <= 80) && (RY >= 11) ) //Y_Right_Surf_mid
  {
    FHL_pwm = CCW_mid;
    FHR_pwm = CW_mid;
    AHL_pwm = CW_mid;
    AHR_pwm = CCW_mid;
    AVL_pwm = CW_mid;
    AVR_pwm = CCW_mid;
  }
  if ((RX >= 155) && (RX <= 174) && (RY <= 100) && (RY >= 79) ) //Y_Right_Surf_mid
  {
    FHL_pwm = CCW_min;
    FHR_pwm = CW_min;
    AHL_pwm = CW_min;
    AHR_pwm = CCW_min;
    AVL_pwm = CW_min;
    AVR_pwm = CCW_min;
  }
  if ((RX >= 245) && (RY >= 245)) //Y_Right_Dive_max
  {
    FHL_pwm = CCW_max;
    FHR_pwm = CW_max;
    AHL_pwm = CW_max;
    AHR_pwm = CCW_max;
    AVL_pwm = CCW_max;
    AVR_pwm = CW_max;
  }
  if ((RX >= 175) && (RX <= 244) && (RY >= 175) && (RY <= 244)) //Y_Right_Dive_med
  {
    FHL_pwm = CCW_mid;
    FHR_pwm = CW_mid;
    AHL_pwm = CW_mid;
    AHR_pwm = CCW_mid;
    AVL_pwm = CCW_mid;
    AVR_pwm = CW_mid;
  }
  if ((RX >= 155) && (RX <= 174) && (RY >= 155) && (RY <= 174)) //Y_Right_Dive_min
  {
    FHL_pwm = CCW_min;
    FHR_pwm = CW_min;
    AHL_pwm = CW_min;
    AHR_pwm = CCW_min;
    AVL_pwm = CCW_min;
    AVR_pwm = CW_min;
  }
  if ((RX <= 10) && (RY >= 245)) //Y_Left_Dive_max
  {
    FHL_pwm = CW_max;
    FHR_pwm = CCW_max;
    AHL_pwm = CCW_max;
    AHR_pwm = CW_max;
    AVL_pwm = CCW_max;
    AVR_pwm = CW_max;
  }
  if ((RX >= 11) && (RX <= 80) && (RY <= 244) && (RY >= 175)) //Y_Left_Dive_med
  {
    FHL_pwm = CW_mid;
    FHR_pwm = CCW_mid;
    AHL_pwm = CCW_mid;
    AHR_pwm = CW_mid;
    AVL_pwm = CCW_mid;
    AVR_pwm = CW_mid;
  }
  if ((RX >= 81) && (RX <= 100) && (RY <= 174) && (RY >= 155)) //Y_Left_Dive_min
  {
    FHL_pwm = CW_min;
    FHR_pwm = CCW_min;
    AHL_pwm = CCW_min;
    AHR_pwm = CW_min;
    AVL_pwm = CCW_min;
    AVR_pwm = CW_min;
  }
  if ((RX <= 10) && (RY <= 10)) //Y_Left_Surf_max
  {
    FHL_pwm = CW_max;
    FHR_pwm = CCW_max;
    AHL_pwm = CCW_max;
    AHR_pwm = CW_max;
    AVL_pwm = CW_max;
    AVR_pwm = CCW_max;
  }
  if ((RX <= 80) && (RX >= 11) && (RY <= 80) && (RY >= 11)) //Y_Left_Surf_med
  {
    FHL_pwm = CW_mid;
    FHR_pwm = CCW_mid;
    AHL_pwm = CCW_mid;
    AHR_pwm = CW_mid;
    AVL_pwm = CW_mid;
    AVR_pwm = CCW_mid;
  }
  if ((RX <= 100) && (RX >= 81) && (RY <= 100) && (RY >= 81)) //Y_Left_Surf_min
  {
    FHL_pwm = CW_min;
    FHR_pwm = CCW_min;
    AHL_pwm = CCW_min;
    AHR_pwm = CW_min;
    AVL_pwm = CW_min;
    AVR_pwm = CCW_min;
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
    if (Rx_data.ROV_ON_Temp <= 11) { //normal elecx operating Tmin=10C
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
    if (Rx_data.ROV_ON_Press >= 44)
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
    if (Rx_data.ROV_EX_Temp >= 35)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WARNING !!");
      lcd.setCursor(0, 1);
      lcd.print("EX TEMP HIGH");
      delay(Disp_wait);
    }
    if (Rx_data.ROV_EX_Temp <= 5)
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
    if (Rx_data.ROVDepth >= 60)
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
