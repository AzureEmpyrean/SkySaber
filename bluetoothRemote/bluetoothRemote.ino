//Add in bt connection waiting code


// -------------------------- LIBS ---------------------------
//#include <avr/pgmspace.h>   // PROGMEM library
#include "Wire.h"
#include "Adafruit_DRV2605.h"
// ------------------------- LIBS ---------------------------

// ------------------------------ VARIABLES ---------------------------------
boolean btnState, btn_flag, hold_flag, rgbBtnState, rgb_btn_flag, rgb_hold_flag, allowBtnState,allow_btn_flag, allow_hold_flag;
byte btn_counter, rgb_btn_counter, allow_btn_counter;
unsigned long btn_timer, rgb_btn_timer, allow_btn_timer, PULSE_timer, swing_timer, swing_timeout, battery_timer, bzzTimer;
byte nowNumber;
byte LEDcolor;  // 0 - red, 1 - green, 2 - blue, 3 - pink, 4 - yellow, 5 - ice blue
byte nowColor, red, green, blue, redOffset, greenOffset, blueOffset;
boolean swing_flag, swing_allow, strike_flag, HUMmode=false, allow=false;
float voltage;
int PULSEOffset;
int hue = 0;
int mode = 0;
uint8_t gHue = 0;
const byte BTpin = 4;
boolean BTconnected = false;
// ------------------------------ VARIABLES ---------------------------------

// ---------------------------- SETTINGS -------------------------------
#define BTN_TIMEOUT 800     // button hold delay, ms
#define RGB_BTN_TIMEOUT 800     // button hold delay, ms
#define DEBUG 1             // debug information in Serial (1 - allow, 0 - disallow)
// ---------------------------- SETTINGS -------------------------------
#define BTN              A0         // A0
#define RGBBTN           A1       // A1
#define ALLOWBTN         A2 
#define modeCt  3

#define HWSERIAL Serial1
Adafruit_DRV2605 drv;


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    HWSERIAL.begin(38400);
    pinMode(BTN, INPUT_PULLUP);
    pinMode(RGBBTN, INPUT_PULLUP);
    pinMode(ALLOWBTN, INPUT_PULLUP);
    pinMode(BTpin, INPUT_PULLUP); 
    
    //Haptic feedback
      drv.begin();  
      drv.selectLibrary(1);
      drv.setMode(DRV2605_MODE_INTTRIG); 
    //Haptic feedback
    
}

void loop() {
	
	 while (!BTconnected)
	    {
		 	 drv.setWaveform(0, 52);  
		 	 drv.setWaveform(1, 0);       
		 	 drv.go();
	      if ( digitalRead(BTpin)==HIGH)  { BTconnected = true;};
	    }
	 
	allowBtnTick();
    rgbBtnTick();
    btnTick();
}

void tx(String trans){  
   Serial.println(trans);
   HWSERIAL.println(trans);
}

void rx(){
  
}

void haptic(int press){
	
	switch (press){
	case(1): //allow button holding			
		  drv.setWaveform(0, 0);  // play 0 
		  drv.setWaveform(1, 0);       // end waveform
		break;
	case(2): //any press
		  drv.setWaveform(0, 0);  // play 0 
		  drv.setWaveform(1, 0);       // end waveform
		break;
	case(3): //any triple click
		  drv.setWaveform(0, 12);  // play 0 
    	  drv.setWaveform(1, 0);       // end waveform
		break;
	case(4): //any 5 click
		  drv.setWaveform(0, 12);  // play 0
		  drv.setWaveform(0, 10);  // play 0
		  drv.setWaveform(1, 0);       // end waveform	
		break;
	case(5):
		  drv.setWaveform(0, 0);  // play 0 
		  drv.setWaveform(1, 0);       // end waveform	
		break;
	case(6): 
		  drv.setWaveform(0, 0);  // play 0 
		  drv.setWaveform(1, 0);       // end waveform	
		break;
	case(7): 
		  drv.setWaveform(0, 0);  // play 0 
		  drv.setWaveform(1, 0);       // end waveform	
		break;
	case(8):
		  drv.setWaveform(0, 0);  // play 0 
		  drv.setWaveform(1, 0);       // end waveform	
		  break;
	}
	drv.go();
}

void rgbBtnTick() {
  rgbBtnState = !digitalRead(RGBBTN);
  if (rgbBtnState && !rgb_btn_flag) {
    if (DEBUG) Serial.println(F("RGB BTN PRESS"));
    rgb_btn_flag = 1;
    rgb_btn_counter++;
    rgb_btn_timer = millis();
    Serial.println("rgb btn");
    delay(250);
  }

   
  if (!rgbBtnState && rgb_btn_flag) {
    rgb_btn_flag = 0;
    rgb_hold_flag = 0;
  }

  if (rgb_btn_flag && rgbBtnState && (millis() - rgb_btn_timer > RGB_BTN_TIMEOUT) && !rgb_hold_flag) {
    rgb_hold_flag = 1;
    rgb_btn_counter = 0;
    
  }
  while(rgb_hold_flag==1){
     tx("rgbHold");
     rgbBtnState = !digitalRead(RGBBTN);
     if (!rgbBtnState && rgb_btn_flag) {
    rgb_btn_flag = 0;
    rgb_hold_flag = 0;
  }
  }
  if ((millis() - rgb_btn_timer > BTN_TIMEOUT) && (rgb_btn_counter != 0)) {
    if (rgb_btn_counter == 1) {               // single press count
          tx("rgbx1");
         }
      if (rgb_btn_counter == 3) {               // 3 press count
       tx("rgbx3");
      }
      if ( rgb_btn_counter == 5) 
        {
        tx("rgbx5");
          }
       
    rgb_btn_counter = 0;
  }
  }

void btnTick() {
  btnState = !digitalRead(BTN);
  if (btnState && !btn_flag) {
    if (DEBUG) Serial.println(F("BTN PRESS"));
    btn_flag = 1;
    btn_counter++;
    btn_timer = millis();
    delay(250);
  }
  if (!btnState && btn_flag) {
    btn_flag = 0;
    hold_flag = 0;
  }

  if (btn_flag && btnState && (millis() - btn_timer > BTN_TIMEOUT) && !hold_flag) {
  //  ls_chg_state = 1;                     // flag to change saber state (on/off)
    hold_flag = 1;
    btn_counter = 0;
    tx("btnHold");
  }

  
  if ((millis() - btn_timer > BTN_TIMEOUT) && (btn_counter != 0)) {
    if (btn_counter == 1) {               // single press count
            tx("btnx1");
          }
    if (btn_counter == 3) {               // 3 press count
        mode++;                         // change mode
        if (mode > modeCt) mode = 0;
        //serial.print();
        tx("btnx3");
       // tx("m" + (String)mode );
      }
      
      if (btn_counter == 5) {               // 5 press count         
      tx("btnx5");
    }
    btn_counter = 0;
  }
}

void allowBtnTick() {
  allowBtnState = !digitalRead(ALLOWBTN);
  if (allowBtnState && !allow_btn_flag) {
    if (DEBUG) Serial.println(F("Allow BTN PRESS"));
    allow_btn_flag = 1;
    allow_btn_counter++;
    allow_btn_timer = millis();
    allow= !allow;
    delay(250);
  }
  if (!allowBtnState && allow_btn_flag) {
    allow_btn_flag = 0;
    allow_hold_flag = 0;
  }

  if (allow_btn_flag && allowBtnState && (millis() - allow_btn_timer > BTN_TIMEOUT) && !allow_hold_flag) {
  //  ls_chg_state = 1;                     // flag to change saber state (on/off)
    allow_hold_flag = 1;
    allow_btn_counter = 0;
  }

  
  if ((millis() - allow_btn_timer > BTN_TIMEOUT) && (allow_btn_counter != 0)) {
    if (allow_btn_counter == 1) {               // single press count
            tx("allowed");
          }
    if (allow_btn_counter == 3) {               // 3 press count
        tx("allowBtnx3");
      }
      
      if (allow_btn_counter == 5) {               // 5 press count         
      tx("allowBtnx5");
    }
    allow_btn_counter = 0;
  }

//    if(allow){
//      btnTick();
//      rgbBtnTick();
//    }
  
}
