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
#include <SparkFunLSM6DS3.h>
// -------------------------- LIBS ---------------------------


// ---------------------------- SETTINGS -------------------------------
#define NUM_LEDS 300         // number of leds
#define BTN_TIMEOUT 800     // button hold delay, ms
#define RGB_BTN_TIMEOUT 800     // button hold delay, ms
#define BRIGHTNESS 255      // max LED brightness (0 - 255)

#define SWING_TIMEOUT 300   // timeout between swings
#define SWING_L_THR 150     // swing angle speed threshold
#define SWING_THR 300       // fast swing angle speed threshold
#define STRIKE_THR 150      // hit acceleration threshold
#define STRIKE_S_THR 320    // hard hit acceleration threshold
#define FLASH_DELAY 80      // flash time while hit

#define PULSE_ALLOW 1       // blade pulsation (1 - allow, 0 - disallow)
#define PULSE_AMPL 20       // pulse amplitude
#define PULSE_DELAY 30      // delay between pulses

#define DEBUG 1             // debug information in Serial (1 - allow, 0 - disallow)
// ---------------------------- SETTINGS -------------------------------

#define ENABLE_AMP_PIN    2

#define freePin           3

#define freePin7           7

#define ENABLE_5V_PIN     8
#define LED_PIN           19          //button
#define LED_PIN2          18         //blade

#define SDCARD_MOSI_PIN  11
#define SDCARD_MISO_PIN  12

#define freePin13        13

#define SDCARD_SCK_PIN   14
#define SDCARD_CS_PIN    15

#define IMUSCL           16
#define IMUSDA           17

#define freePin18        18
#define freePin19        19
#define freePin20        20
#define fetPin           21

#define BTN              29         // dac1
#define RGBBTN           28       // A1


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
//MPU6050 accelgyro;

LSM6DS3 accelgyro;

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

String string;
String string2;


// ------------------------------ VARIABLES ---------------------------------

void bootCheck(){
  
}

void setup() {
  delay(2000);  
  FastLED.addLeds<WS2811, LED_PIN, RGB>(leds, 0,2);
  FastLED.addLeds<WS2812B, LED_PIN2, GRB>(leds, 1, NUM_LEDS);
  FastLED.setBrightness(100);  // ~40% of LED strip brightness
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Booting up");

  pinMode(BTN, INPUT_PULLUP);
  pinMode(RGBBTN, INPUT_PULLUP);
    fill_solid(leds, NUM_LEDS,  CHSV(055, 255, 255) );
           FastLED.show();
  randomSeed(analogRead(2));    // starting point for random generator

    // For MPU6050
 /* accelgyro.initialize();
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
  */

  accelgyro.begin();
  
    // SD initialization

//  if (DEBUG) {
//    if (SD.begin(15)) Serial.println(F("SD OK"));
//    else Serial.println(F("SD fail"));
//  } else {
//    SD.begin(15);
//  }


  
//   if (!(SD.begin(15))) {
//    // stop here, but print a message repetitively
//    while (1) {
//      leds[1] = CHSV(240, 255, 255);
//          FastLED.show();
//      Serial.println("Unable to access the SD card");
//      delay(500);
//    }
//  }

  
   //Enable the EnchantFX board amp and 5v logic for leds
  pinMode(ENABLE_AMP_PIN,  INPUT_PULLUP);
  digitalWrite(ENABLE_AMP_PIN, HIGH);
  pinMode(ENABLE_5V_PIN,  INPUT_PULLUP);
  digitalWrite(ENABLE_5V_PIN, HIGH);

  FastLED.setBrightness(BRIGHTNESS);   // set bright
  //leds[0] = CHSV(hue, 255, 255);
  FastLED.show();
  
  dac1.analogReference(EXTERNAL);
  AudioMemory(18);
  
//  pinMode(fetPin, OUTPUT);  
//  digitalWrite(fetPin, HIGH); //Turn on the power to the bluetooth module.
  
  //Bluetooth begin
  Serial1.begin(9600);
  //Serial1.begin(19200);
  //Bluetooth


 //SD.open("/AUDIO");
  //  findFonts();

}

// --- MAIN LOOP---
void loop() {
  
  FastLED.setBrightness(100);
  //randomPULSE();
  Rx();
  btnTick();
 rgbBtnTick();

//switch (mode) {
//    case 0:
//      cycle();
//      break;
//    case 1:
//      randCycle();
//      break;
//    case 2:
//      rainbowCycle();
//      break;
//    case 3:
//      rainbow();
//      break;
//    case 4:
//      sinelon();
//      break;
//   case 5:
//      cylon();
//      break;
//  }

 string="";
  }

// --- MAIN LOOP---
void Rx(){
     
  if (Serial1.available()){
       string = Serial1.readStringUntil('\n');
             string.trim();
                 Serial.println(string); 
            }
// Serial.println(string); 
   Serial1.flush();

  }
  
void rgbBtnTick() {
  rgbBtnState = !digitalRead(RGBBTN);
  if ( (rgbBtnState && !rgb_btn_flag && (millis() - rgb_btn_timer > RGB_BTN_TIMEOUT)  ) || (string == "rgbx1") ){
     if (DEBUG) Serial.println(F("This is pin 29******"));
    rgb_btn_flag = 1;
    rgb_btn_counter++;
    rgb_btn_timer = millis();
       if (DEBUG) Serial.print(("rgb counter: "));
   if (DEBUG) Serial.println((rgb_btn_counter ));
  }
   
  if ((!rgbBtnState && rgb_btn_flag)){
    rgb_btn_flag = 0;
    rgb_hold_flag = 0;
    if (DEBUG) Serial.println(("rgb counter is reset"));
  }

   if ( (rgb_btn_flag && rgbBtnState && (millis() - rgb_btn_timer > RGB_BTN_TIMEOUT) && !hold_flag)) {
    rgb_hold_flag = 1;
    rgb_btn_counter = 0;
     //  if (DEBUG) Serial.println(("rgb counter is reset"));
  }
  

   
  if ( ((millis() - rgb_btn_timer > RGB_BTN_TIMEOUT) && (rgb_btn_counter != 0)) || (string == "rgbx1") || (string == "rgbx3") || (string == "rgbx5") ) {
           if (DEBUG) Serial.print(("rgb counter: "));
   if (DEBUG) Serial.println((rgb_btn_counter ));
   
      if (rgb_btn_counter == 1) {
     // Serial1.println("this should light up");
            hue = 100;
            fill_solid(leds, NUM_LEDS,  CHSV(hue, 255, 255) );
           FastLED.show();
           delay(1000);
           hue = 155;
           fill_solid(leds, NUM_LEDS,  CHSV(hue, 255, 255) );
           FastLED.show();            
          }
      if (rgb_btn_counter == 3) {               // 3 press count
       
          }
      if ( rgb_btn_counter == 5){
        
          }
       rgb_btn_counter = 0;
      }    

  }


void btnTick() {  
  btnState = !digitalRead(BTN);
  if ( (btnState && !btn_flag && (millis() - btn_timer > BTN_TIMEOUT)  ) || (string == "btnx1") ) {
    if (DEBUG) Serial.println(F("This is pin 28@@@@"));
    btn_flag = 1;
    btn_counter++;
    btn_timer = millis();
  }
  if (!btnState && btn_flag) {
    btn_flag = 0; 
    hold_flag = 0;
  }

  if ( (btn_flag && btnState && (millis() - btn_timer > BTN_TIMEOUT) && !hold_flag) || (string == "btnHold")){
    hold_flag = 1;
    btn_counter = 0;
  }

  if ( ((millis() - btn_timer > BTN_TIMEOUT) && (btn_counter != 0)) || (string == "btnx1") || (string == "btnx3") || (string == "btnx5") ) {

    if (btn_counter == 1) {
          ls_chg_state = 1;                     // flag to change saber state (on/off)
          Serial.println(ls_chg_state);
    }
    if ((btn_counter == 3) || (string == "btnx3")){               // 3 press count
        mode++;                         // change mode
        if (mode > modeCt) mode = 0;
      }
      
      if ((btn_counter == 5)  || (string == "btnx5")) {               // 5 press count         
      mute = !mute;
        if (!mute) {
       playHum.play("HUM.wav");
        } else {
     //   muteAll(6);
        }
    }
    btn_counter = 0;
  }
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
          dupe[i] = CHSV(hue, 255, 255);
          FastLED.show();
          delay(25);
        }
  }
}

void light_down() {
  for (int i = NUM_LEDS ; i >= 1 ; i--) {
       // hue=dupe[i];
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
 
  if ((string.equals("rgbHold") || rgb_hold_flag))
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

