// -------------------------- LIBS ---------------------------
//#include <avr/pgmspace.h>   // PROGMEM library
#include "Wire.h"

// -------------------------- LIBS ---------------------------

// ------------------------------ VARIABLES ---------------------------------
boolean btnState, btn_flag, hold_flag, rgbBtnState, rgb_btn_flag, rgb_hold_flag;
byte btn_counter, rgb_btn_counter;
unsigned long btn_timer, rgb_btn_timer, PULSE_timer, swing_timer, swing_timeout, battery_timer, bzzTimer;
byte nowNumber;
byte LEDcolor;  // 0 - red, 1 - green, 2 - blue, 3 - pink, 4 - yellow, 5 - ice blue
byte nowColor, red, green, blue, redOffset, greenOffset, blueOffset;
boolean swing_flag, swing_allow, strike_flag, HUMmode=false;
float voltage;
int PULSEOffset;
int hue = 0;
int mode = 0;
uint8_t gHue = 0;
// ------------------------------ VARIABLES ---------------------------------

// ---------------------------- SETTINGS -------------------------------
#define BTN_TIMEOUT 800     // button hold delay, ms
#define RGB_BTN_TIMEOUT 400     // button hold delay, ms
#define DEBUG 1             // debug information in Serial (1 - allow, 0 - disallow)
// ---------------------------- SETTINGS -------------------------------
#define BTN              23         // A0
#define RGBBTN           22       // A1
#define modeCt  3



void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void rgbBtnTick() {
  rgbBtnState = !digitalRead(RGBBTN);
  if (rgbBtnState && !rgb_btn_flag) {
    if (DEBUG) Serial.println(F("BTN PRESS"));
    rgb_btn_flag = 1;
    rgb_btn_counter++;
    rgb_btn_timer = millis();
  }

   
  if (!rgbBtnState && rgb_btn_flag) {
    rgb_btn_flag = 0;
    rgb_hold_flag = 0;
  }

  if (rgb_btn_flag && rgbBtnState && (millis() - rgb_btn_timer > RGB_BTN_TIMEOUT) && !hold_flag) {
    rgb_hold_flag = 1;
    rgb_btn_counter = 0;
  }

  if ((millis() - rgb_btn_timer > BTN_TIMEOUT) && (rgb_btn_counter != 0)) {
    
      if (rgb_btn_counter == 3) {               // 3 press count
       
      }
      if ( rgb_btn_counter == 5) 
        {
          }
      }
      
      
      
    
    rgb_btn_counter = 0;
  }


void btnTick() {
  btnState = !digitalRead(BTN);
  if (btnState && !btn_flag) {
    if (DEBUG) Serial.println(F("BTN PRESS"));
    btn_flag = 1;
    btn_counter++;
    btn_timer = millis();
  }
  if (!btnState && btn_flag) {
    btn_flag = 0;
    hold_flag = 0;
  }

  if (btn_flag && btnState && (millis() - btn_timer > BTN_TIMEOUT) && !hold_flag) {
  //  ls_chg_state = 1;                     // flag to change saber state (on/off)
    hold_flag = 1;
    btn_counter = 0;
  }

  
  if ((millis() - btn_timer > BTN_TIMEOUT) && (btn_counter != 0)) {
    if (btn_counter == 3) {               // 3 press count
        mode++;                         // change mode
        if (mode > modeCt) mode = 0;
      }
      
      if (btn_counter == 5) {               // 5 press count         

    }
    btn_counter = 0;
  }
}
