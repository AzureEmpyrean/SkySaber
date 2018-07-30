

//   https://github.com/thefirebrandforge/EnchantOS/blob/master/Source/Board.h
//   https://thefirebrandforge.com/lightsaber/electronics/

// -------------------------- LIBS ---------------------------
//#include <avr/pgmspace.h>   // PROGMEM library
#include "SD.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "FastLED.h"        // addressable LED library
#include "Audio.h"
#include "SPI.h"
#include "SerialFlash.h"

// -------------------------- LIBS ---------------------------


// ---------------------------- SETTINGS -------------------------------
#define NUM_LEDS 45         // number of leds
#define BTN_TIMEOUT 800     // button hold delay, ms
#define RGB_BTN_TIMEOUT 400     // button hold delay, ms
#define BRIGHTNESS 255      // max LED brightness (0 - 255)

#define SWING_TIMEOUT 500   // timeout between swings
#define SWING_L_THR 150     // swing angle speed threshold
#define SWING_THR 300       // fast swing angle speed threshold
#define STRIKE_THR 150      // hit acceleration threshold
#define STRIKE_S_THR 320    // hard hit acceleration threshold
#define FLASH_DELAY 80      // flash time while hit

#define PULSE_ALLOW 1       // blade pulsation (1 - allow, 0 - disallow)
#define PULSE_AMPL 20       // pulse amplitude
#define PULSE_DELAY 30      // delay between pulses

#define R1 100000           // voltage divider real resistance
#define R2 51000            // voltage divider real resistance
#define BATTERY_SAFE 1      // battery monitoring (1 - allow, 0 - disallow)

#define DEBUG 1             // debug information in Serial (1 - allow, 0 - disallow)
// ---------------------------- SETTINGS -------------------------------

#define ENABLE_AMP_PIN    2

#define freePin           3

#define freePin7           7

#define ENABLE_5V_PIN     8
#define LED_PIN           9          //
#define LED_PIN2          10   

#define SDCARD_MOSI_PIN  11
#define SDCARD_MISO_PIN  12

#define freePin13        13

#define SDCARD_SCK_PIN   14
#define SDCARD_CS_PIN    15

#define freePin16        16
#define freePin17        17
#define freePin18        18
#define freePin19        19
#define freePin20        20
#define fetPin           21

#define BTN              23         // A0
#define RGBBTN           22       // A1



#define modeCt 3

AudioPlaySdWav           playBoot; 
AudioPlaySdWav           playHum;        //saber hum
AudioPlaySdWav           playStrike1;    //Strike
AudioPlaySdWav           playStrike2;    //Strike S
AudioPlaySdWav           playSwing1;     //Swing
AudioPlaySdWav           playSwing2;     //Swing L
AudioMixer4              mixer1;         //xy=371,158
AudioMixer4              mixer2;         //xy=373,278
AudioOutputAnalog        dac1;           //xy=632,263

AudioConnection          patchCord1(playStrike1, 0, mixer1, 1);
AudioConnection          patchCord2(playHum, 0, mixer1, 0);
AudioConnection          patchCord3(playStrike2, 0, mixer1, 2);
AudioConnection          patchCord4(playSwing1, 0, mixer1, 3);
AudioConnection          patchCord5(playSwing2, 0, mixer2, 0);
AudioConnection          patchcord6(playBoot, 0, mixer2, 0);
AudioConnection          patchCord7(mixer1, dac1);
AudioConnection          patchCord8(mixer2, dac1);


CRGB leds[NUM_LEDS];
CRGB dupe[NUM_LEDS];
MPU6050 accelgyro;


const int chipSelect = 8;



// ------------------------------ VARIABLES ---------------------------------
int16_t ax, ay, az;
int16_t gx, gy, gz;
unsigned long ACC, GYR, COMPL;
int gyroX, gyroY, gyroZ, accelX, accelY, accelZ, freq, freq_f = 0;
float k = 0.2;
unsigned long humTimer = -9000, mpuTimer, nowTimer;
int stopTimer;
boolean bzzz_flag, ls_chg_state, ls_state;
boolean btnState, btn_flag, hold_flag, rgbBtnState, rgb_btn_flag, rgb_hold_flag;
byte btn_counter, rgb_btn_counter;
unsigned long btn_timer, rgb_btn_timer, PULSE_timer, swing_timer, swing_timeout, battery_timer, bzzTimer;
byte nowNumber;
byte LEDcolor;  // 0 - red, 1 - green, 2 - blue, 3 - pink, 4 - yellow, 5 - ice blue
byte nowColor, red, green, blue, redOffset, greenOffset, blueOffset;
boolean swing_flag, swing_allow, strike_flag, mute=false;
float voltage;
int PULSEOffset;
int hue = 0;
int mode = 0;
uint8_t gHue = 0;
// ------------------------------ VARIABLES ---------------------------------

// --------------------------------- SOUNDS ----------------------------------
const char strike1[] PROGMEM = "SK1.wav";
const char strike2[] PROGMEM = "SK2.wav";
const char strike3[] PROGMEM = "SK3.wav";
const char strike4[] PROGMEM = "SK4.wav";
const char strike5[] PROGMEM = "SK5.wav";
const char strike6[] PROGMEM = "SK6.wav";
const char strike7[] PROGMEM = "SK7.wav";
const char strike8[] PROGMEM = "SK8.wav";

const char* const strikes[] PROGMEM  = {
  strike1, strike2, strike3, strike4, strike5, strike6, strike7, strike8
};

int strike_time[8] = {779, 563, 687, 702, 673, 661, 666, 635};

const char strike_s1[] PROGMEM = "SKS1.wav";
const char strike_s2[] PROGMEM = "SKS2.wav";
const char strike_s3[] PROGMEM = "SKS3.wav";
const char strike_s4[] PROGMEM = "SKS4.wav";
const char strike_s5[] PROGMEM = "SKS5.wav";
const char strike_s6[] PROGMEM = "SKS6.wav";
const char strike_s7[] PROGMEM = "SKS7.wav";
const char strike_s8[] PROGMEM = "SKS8.wav";

const char* const strikes_short[] PROGMEM = {
  strike_s1, strike_s2, strike_s3, strike_s4,
  strike_s5, strike_s6, strike_s7, strike_s8
};
int strike_s_time[8] = {270, 167, 186, 250, 252, 255, 250, 238};

const char swing1[] PROGMEM = "SWS1.wav";
const char swing2[] PROGMEM = "SWS2.wav";
const char swing3[] PROGMEM = "SWS3.wav";
const char swing4[] PROGMEM = "SWS4.wav";
const char swing5[] PROGMEM = "SWS5.wav";

const char* const swings[] PROGMEM  = {
  swing1, swing2, swing3, swing4, swing5
};
int swing_time[8] = {389, 372, 360, 366, 337};

const char swingL1[] PROGMEM = "SWL1.wav";
const char swingL2[] PROGMEM = "SWL2.wav";
const char swingL3[] PROGMEM = "SWL3.wav";
const char swingL4[] PROGMEM = "SWL4.wav";

const char* const swings_L[] PROGMEM  = {
  swingL1, swingL2, swingL3, swingL4
};
int swing_time_L[8] = {636, 441, 772, 702};

char BUFFER[10];
// --------------------------------- SOUNDS ---------------------------------

void setup() {
  delay(2000);  
  FastLED.addLeds<WS2811, LED_PIN, GRB>(leds, 0,1);
  FastLED.addLeds<WS2812B, LED_PIN2, RGB>(leds, 1, NUM_LEDS);
  FastLED.setBrightness(100);  // ~40% of LED strip brightness
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Booting up");

  pinMode(BTN, INPUT_PULLUP);
  pinMode(RGBBTN, INPUT_PULLUP);

  //digitalWrite(10, HIGH);
  randomSeed(analogRead(2));    // starting point for random generator

    // IMU initialization
  accelgyro.initialize();
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  if (DEBUG) {
    if (accelgyro.testConnection()) Serial.println(F("MPU6050 OK"));
    else {
     // leds[2] = CHSV(200, 255, 255);
          FastLED.show();
      Serial.println(F("MPU6050 fail"));
    }
  }
    // SD initialization

//  if (DEBUG) {
//    if (SD.begin(15)) Serial.println(F("SD OK"));
//    else Serial.println(F("SD fail"));
//  } else {
//    SD.begin(15);
//  }

  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  SPI.setMISO(SDCARD_MISO_PIN);
  
   if (!(SD.begin(15))) {
    // stop here, but print a message repetitively
    while (1) {
      leds[1] = CHSV(240, 255, 255);
          FastLED.show();
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
  pinMode(ENABLE_AMP_PIN,  INPUT_PULLUP);
  digitalWrite(ENABLE_AMP_PIN, HIGH);
  pinMode(ENABLE_5V_PIN,  INPUT_PULLUP);
  digitalWrite(ENABLE_5V_PIN, HIGH);

  delay(1000);                         // 1 second to show battery level
  FastLED.setBrightness(BRIGHTNESS);   // set bright
 // leds[0] = CRGB::Purple;
 // leds[1] = CHSV(hue, 255, 255);
  FastLED.show();
  
  
  AudioMemory(18);
  pinMode(fetPin, OUTPUT);
  digitalWrite(fetPin, HIGH);
  //Bluetooth begin
  Serial1.begin(38400);
  
  //Bluetooth
  
}

String readstring;
bool rgbHold=false;

void Rx(){
   
  if (Serial1.available()){
       readstring = Serial1.readStringUntil('/0');
             readstring = readstring.trim();
            // Serial.print("green");
          //   Serial.println(readstring);
           //  Serial.println("green");
            if (readstring.equals("rgbHold"))
            {rgbHold=true;
              Serial.println("holding");}
              
            }
            
    

//                          Serial.println("rgb hold");
//                      }
//                        if (readstring == "rgbPressx1"){
//                        Serial.println("rgb press");  
//                          rgb_btn_flag = 1;
//                      }
//                        if (readstring == "rgbPressx3/n"){
//                        rgb_btn_counter = 3;
//                        Serial.println("rgb press x3");
//                        }
//                          if (readstring == "rgbPressx5"){
//                          Serial.println("rgb press x5");
//                            rgb_btn_counter = 5;
//                          }
//                    
//
//              if (readstring == "btnHold"){
//                Serial.println("button hold");
//                //hold_flag = 1;
//              }
//              if (readstring == "btnPressx1"){
//                Serial.println("button press");
//               // ls_chg_state = 1;
//                btn_flag = 1;
//            }
//              if (readstring == "btnPressx3"){
//                Serial.println("button press x3");
//                //btn_counter = 3;
//              }
//              if (readstring == "btnPressx5"){
//                Serial.println("button press x5");
//                //btn_counter = 5;
//              }
                Serial1.flush();
  }

// --- MAIN LOOP---
void loop() {
  
  //randomPULSE();
  getFreq();
  Rx();
  on_off_sound();
  btnTick();
  rgbBtnTick();
  if(ls_state)  strikeTick();
  swingTick();
//  
//  if (readstring == "rgbHold")
//     Serial.print(readstring);
  //Serial.println("green");
switch (mode) {
    case 0:
      cycle();
      break;
    case 1:
      randCycle();
      break;
    case 2:
      rainbowCycle();
      break;
    case 3:
      rainbow();
      break;
    case 4:
      sinelon();
      break;
   case 5:
      cylon();
      break;
  }

 readstring="";
  }

// --- MAIN LOOP---

void muteAll(int x){ 
  if(x == 1) playHum.stop();        //saber hum
  if(x == 2) playStrike1.stop();    //Strike
  if(x == 3) playStrike2.stop();    //Strike S
  if(x == 4) playSwing1.stop();     //Swing
  if(x == 5) playSwing2.stop();     //Swing L
  
  if(x == 6){
	  playBoot.stop();
    playHum.stop();        //saber hum
    playStrike1.stop();    //Strike
    playStrike2.stop();    //Strike S
    playSwing1.stop();     //Swing
    playSwing2.stop();     //Swing L
  }
}


void rgbBtnTick() {

  
  rgbBtnState = !digitalRead(RGBBTN);
  if ( (rgbBtnState && !rgb_btn_flag) || (readstring == "rgbx1") ){
    //if (DEBUG) Serial.println(F("RGB BTN PRESS"));
    rgb_btn_flag = 1;
    rgb_btn_counter++;
    rgb_btn_timer = millis();
  }

   
  if ((!rgbBtnState && rgb_btn_flag) || !rgbHold){
    rgb_btn_flag = 0;
    rgb_hold_flag = 0;
  }
     Serial.print("rgbHold: ");
     Serial.println(rgbHold);
     Serial.print("rgb hold flag: ");
     Serial.println(rgb_hold_flag);
  if ( rgbHold || (rgb_btn_flag && rgbBtnState && (millis() - rgb_btn_timer > RGB_BTN_TIMEOUT) && !hold_flag)) {
   
    rgb_hold_flag = 1;
    rgb_btn_counter = 0;
    
  }
  if ( ((millis() - rgb_btn_timer > BTN_TIMEOUT) && (rgb_btn_counter != 0)) || (readstring == "rgbx3") || (readstring == "rgbx5") ) {
    
      if (rgb_btn_counter == 3) {               // 3 press count
       
      }
      if ( rgb_btn_counter == 5) 
        {
          }
      }    
      rgbHold=false;
    rgb_btn_counter = 0;
  }



void btnTick() {  
  btnState = !digitalRead(BTN);
  if ( (btnState && !btn_flag) || (readstring == "btnx1") ) {
    if (DEBUG) Serial.println(F("BTN PRESS"));
    btn_flag = 1;
    btn_counter++;
    btn_timer = millis();
  }
  if (!btnState && btn_flag) {
    btn_flag = 0;
    hold_flag = 0;
  }

  if ( (btn_flag && btnState && (millis() - btn_timer > BTN_TIMEOUT) && !hold_flag) || (readstring == "btnHold")){
    hold_flag = 1;
    btn_counter = 0;
  }

  
  if ( ((millis() - btn_timer > BTN_TIMEOUT) && (btn_counter != 0)) || (readstring == "btnx1") || (readstring == "btnx3") || (readstring == "btnx5") ) {

    if (btn_counter == 1) {
          ls_chg_state = 1;                     // flag to change saber state (on/off)
    }
    if ((btn_counter == 3) || (readstring == "btnx3")){               // 3 press count
        mode++;                         // change mode
        if (mode > modeCt) mode = 0;
      }
      
      if ((btn_counter == 5)  || (readstring == "btnx5")) {               // 5 press count         
      mute = !mute;
        if (!mute) {
       playHum.play("HUM.wav");
        } else {
        muteAll(6);
        }
    }
    btn_counter = 0;
  }
}


void on_off_sound() {
  if (ls_chg_state) {                // if change flag
    if (!ls_state) {                 // if GyverSaber is turned off
        if (DEBUG) Serial.println(F("SABER ON"));
       playBoot.play("ON.wav");
        delay(200);
        light_up();
        delay(200);
        bzzz_flag = 1;
        ls_state = true;               // remember that turned on
        //playBoot.stop();
        if (!mute) {
          playHum.play("HUM.wav");
        } else {
          muteAll(6);  
        }
    } else {                         // if GyverSaber is turned on
      bzzz_flag = 0;
     playBoot.play("OFF.wav");
      delay(300);
      light_down();
      delay(300);
      muteAll(6);
    if (DEBUG) Serial.println(F("SABER OFF"));
      ls_state = false;      
    }
    ls_chg_state = 0;
  }

  if (((millis() - humTimer) > 9000) && bzzz_flag && mute) {
    playHum.play("HUM.wav");
    humTimer = millis();
    swing_flag = 1;
    strike_flag = 0;
  }
  long delta = millis() - bzzTimer;
  if ((delta > 3) && bzzz_flag && !mute) {
    if (strike_flag) {
      muteAll(1);
      muteAll(2);
      strike_flag = 0;
    }
    bzzTimer = millis();
  }
}

void randomPULSE() {
  if (PULSE_ALLOW && ls_state && (millis() - PULSE_timer > PULSE_DELAY)) {
    PULSE_timer = millis();
    PULSEOffset = PULSEOffset * k + random(-PULSE_AMPL, PULSE_AMPL) * (1 - k);
    if (nowColor == 0) PULSEOffset = constrain(PULSEOffset, -15, 5);
    redOffset = constrain(red + PULSEOffset, 0, 255);
    greenOffset = constrain(green + PULSEOffset, 0, 255);
    blueOffset = constrain(blue + PULSEOffset, 0, 255);
    setAll(redOffset, greenOffset, blueOffset);
  }
}

void strikeTick() {
  if ((ACC > STRIKE_THR) && (ACC < STRIKE_S_THR)) {
  //  if (!mute) noToneAC();
    nowNumber = random(8);

    strcpy_P(BUFFER, (char*)pgm_read_word(&(strikes_short[nowNumber])));
  playStrike1.play(BUFFER);
    hit_flash();
    if (!mute)
      bzzTimer = millis() + strike_s_time[nowNumber] - FLASH_DELAY;
    else
      humTimer = millis() - 9000 + strike_s_time[nowNumber] - FLASH_DELAY;
    strike_flag = 1;
  }
  
  
  if (ACC >= STRIKE_S_THR) {
 //   if (!mute) noToneAC();
    nowNumber = random(8);

    strcpy_P(BUFFER, (char*)pgm_read_word(&(strikes[nowNumber])));
   playStrike2.play(BUFFER);
    hit_flash();
    if (!mute)
      bzzTimer = millis() + strike_time[nowNumber] - FLASH_DELAY;
    else
      humTimer = millis() - 9000 + strike_time[nowNumber] - FLASH_DELAY;
    strike_flag = 1;
  }
}

void swingTick() {
  if (GYR > 80 && (millis() - swing_timeout > 100) && mute) {
    swing_timeout = millis();
    if (((millis() - swing_timer) > SWING_TIMEOUT) && swing_flag && !strike_flag) {
      if (GYR >= SWING_THR) {      
        nowNumber = random(5);          

        strcpy_P(BUFFER, (char*)pgm_read_word(&(swings[nowNumber])));
        playSwing1.play(BUFFER);               
        humTimer = millis() - 9000 + swing_time[nowNumber];
        swing_flag = 0;
        swing_timer = millis();
        swing_allow = 0;
      }
      if ((GYR > SWING_L_THR) && (GYR < SWING_THR)) {
        nowNumber = random(5);            

        strcpy_P(BUFFER, (char*)pgm_read_word(&(swings_L[nowNumber])));
      playSwing2.play(BUFFER);              
        humTimer = millis() - 9000 + swing_time_L[nowNumber];
        swing_flag = 0;
        swing_timer = millis();
        swing_allow = 0;
      }
    }
  }
}

void getFreq() {
 // if (ls_state) {                                               // if GyverSaber is on
    if (millis() - mpuTimer > 500) {                            
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);       

      // find absolute and divide on 100
      gyroX = abs(gx / 100);
      gyroY = abs(gy / 100);
      gyroZ = abs(gz / 100);
      accelX = abs(ax / 100);
      accelY = abs(ay / 100);
      accelZ = abs(az / 100);

      // vector sum
      ACC = sq((long)accelX) + sq((long)accelY) + sq((long)accelZ);
      ACC = sqrt(ACC);
      GYR = sq((long)gyroX) + sq((long)gyroY) + sq((long)gyroZ);
      GYR = sqrt((long)GYR);
      COMPL = ACC + GYR;
      

//         Serial.print("$");
//         Serial.print(gyroX);
//         Serial.print(" ");
//         Serial.print(gyroY);
//         Serial.print(" ");
//         Serial.print(gyroZ);
//         Serial.println(";");
      
      freq = (long)COMPL * COMPL / 1500;                        // parabolic tone change
      freq = constrain(freq, 18, 300);                          
      freq_f = freq * k + freq_f * (1 - k);                     // smooth filter
      mpuTimer = micros();                                     
    }
 // }
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
  leds[Pixel].r = red;
  leds[Pixel].g = green;
  leds[Pixel].b = blue;
}

void setAll(byte red, byte green, byte blue) {
  for (int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue);
  }
  FastLED.show();
}

void light_up() {

  if (mode==3){
	  for (int i = 0; i < NUM_LEDS; i++) {
		  	  	// hue=dupe[i];
	            leds[i] = dupe[i];
	            FastLED.show();
	            delay(25);
	          }
  }
  else{
  for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CHSV(hue, 255, 255);
          FastLED.show();
          delay(25);
        }
  }
}

void light_down() {
  for (int i = NUM_LEDS -1 ; i >= 0 ; i--) {
	  	  hue=dupe[i];
          leds[i] = CHSV(hue, 255, 0);
          FastLED.show();
          delay(50);
        }
}

void hit_flash() {
  for (int i = 0; i < NUM_LEDS; i++) {
          leds[i]= -leds[i]; 
        }
   FastLED.show();            
  delay(FLASH_DELAY);                
  for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = dupe[i]; 
        }
   FastLED.show();        
}


void cycle() {
 
  if ((readstring == "rgbHold" || rgb_hold_flag))
  {
    Serial.println(hue);
    hue++;
    if (hue>=256) hue=0;
    if (!ls_state) {
      leds[0] = CHSV(hue, 255, 255);

    }
    if (ls_state) {
      fill_solid(leds, NUM_LEDS,  CHSV(hue, 255, 255) );
      FastLED.show();
    }
       for (int i = 0; i < NUM_LEDS; i++) {
          dupe[i] = leds[i]; 
        }
     FastLED.show();
    delay(80);
  }

}

void randCycle(){
  
    hue++;
    delay(10);

    if (!ls_state) leds[0] = CHSV(hue, 255, 255);

    if (ls_state) {
      fill_solid(leds, NUM_LEDS,  CHSV(hue, 255, 255) );
      
    }
    FastLED.show();
  }

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}



void sinelon()
{
  static uint16_t num = 0;
  static bool runonce = false; // Used to turn on the saber only he button.
  static uint32_t mytime = millis();
  static uint16_t mydelay = 20; // Change this to speed up/slow down your led effect.'once' while you hold t

   if (ls_state) {
      // a colored dot sweeping back and forth, with fading trails
      fadeToBlackBy( leds, NUM_LEDS, 2);
      int pos = beatsin16( 13, 0, NUM_LEDS - 1 );
      leds[pos] += CHSV( gHue, 255, 192);
      EVERY_N_MILLISECONDS( 8 ) {
        gHue++;
      }
   }
  FastLED.show();
}

void cylon(){
  static uint16_t num = 0;
  static bool runonce = false; // Used to turn on the saber only 'once' while you hold the button.
  static uint32_t mytime = millis();
  static uint16_t mydelay = 20; // Change this to speed up/slow down your led effect.  
  
   if (ls_state) {      
 static uint8_t hue = 0;
 // Serial.print("x");
  // First slide the led in one direction
  for(int i = 0; i < NUM_LEDS; i++) {
    // Set the i'th led to red 
    leds[i] = CHSV(hue++, 255, 255);
    // Show the leds
    FastLED.show(); 
    // now that we've shown the leds, reset the i'th led to black
    // leds[i] = CRGB::Black;
    fadeall();
    // Wait a little bit before we loop around and do it again
    delay(10);
  }
//  Serial.print("x");

  // Now go in the other direction.  
  for(int i = (NUM_LEDS)-1; i >= 0; i--) {
    // Set the i'th led to red 
    leds[i] = CHSV(hue++, 255, 255);
    // Show the leds
    FastLED.show();
    // now that we've shown the leds, reset the i'th led to black
    // leds[i] = CRGB::Black;
    fadeall();
    // Wait a little bit before we loop around and do it again
    delay(10);
  }
   }
  FastLED.show();
}

void rainbowCycle() {
  byte *c;
  uint16_t i, j;
  static uint16_t num = 0;
  static bool runonce = false; // Used to turn on the saber only 'once' while you hold the button.
  static uint32_t mytime = millis();
  static uint16_t mydelay = 20; // Change this to speed up/slow down your led effect.
    for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = dupe[i]; 
        }
   if (ls_state) {
  //for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
  j = (millis() / 10) % (256 * 5);
  for(i=0; i< NUM_LEDS; i++) {
    c=Wheel(((i * 256 / NUM_LEDS) + j) & 255);
    setPixel(i, *c, *(c+1), *(c+2));
  }
  FastLED.show();
}
   }


byte * Wheel(byte WheelPos) {
  static byte c[3];
  
  if(WheelPos < 85) {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  } else {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }
  return c;
}

void fadeall() {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i].nscale8(250);
  }
}


