
#define BTN              12     
#define RGBBTN           13      
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
int btn=0, rgb=0, bootCheck=0;
#define NUM_LEDS 146         // number of leds
#define BTN_TIMEOUT 600     // button hold delay, ms
#define RGB_BTN_TIMEOUT 400     // button hold delay, ms
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
#define modeCt 3
#define DEBUG 1             // debug information in Serial (1 - allow, 0 - disallow)




void setup() {

    Serial.begin(9600);
  pinMode(BTN, INPUT_PULLUP);
  pinMode(RGBBTN, INPUT_PULLUP);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  rgbBtnTick();
  btnTick();
}


void rgbBtnTick() {
  rgbBtnState = !digitalRead(RGBBTN);
  if ( (rgbBtnState && !rgb_btn_flag) || (string == "rgbx1") ){
    Serial.println(F("RGB BTN PRESS"));
    rgb_btn_flag = 1;
    rgb_btn_counter++;
    rgb_btn_timer = millis();
  }
   
  if ((!rgbBtnState && rgb_btn_flag)){
    rgb_btn_flag = 0;
    rgb_hold_flag = 0;
  }

   if ( (rgb_btn_flag && rgbBtnState && (millis() - rgb_btn_timer > RGB_BTN_TIMEOUT) && !hold_flag)) {
    rgb_hold_flag = 1;
    rgb_btn_counter = 0;
    Serial.println("RGB Hold ");
  }

  if ( ((millis() - rgb_btn_timer > BTN_TIMEOUT) && (rgb_btn_counter != 0)) || (string == "rgbx3") || (string == "rgbx5") ) {
    
      if (rgb_btn_counter == 3) {               // 3 press count
       Serial.println("3z RGB Press ");
      }
      if ( rgb_btn_counter == 5){
          Serial.println("5x RGB Press ");
          }
        rgb_btn_counter = 0;
      }    

  }


void btnTick() {  
  btnState = !digitalRead(BTN);
  if ( (btnState && !btn_flag) || (string == "btnx1") ) {
    Serial.println(F("BTN PRESS"));
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
    Serial.println(" BTN Hold ");
  }

  if ( ((millis() - btn_timer > BTN_TIMEOUT) && (btn_counter != 0)) || (string == "btnx1") || (string == "btnx3") || (string == "btnx5") ) {

    if (btn_counter == 1) {
          ls_chg_state = 1;                     // flag to change saber state (on/off)
    }
    if ((btn_counter == 3) || (string == "btnx3")){               // 3 press count
      Serial.println(" 3x BTN Press ");
        mode++;                         // change mode
        if (mode > modeCt) mode = 0;
      }
      
      if ((btn_counter == 5)  || (string == "btnx5")) {               // 5 press count         
          Serial.println("5x BTN Press ");
    }
    btn_counter = 0;
  }
}
