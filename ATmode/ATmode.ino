//#include <SoftwareSerial.h>

//SoftwareSerial BTSerial(0, 1); // RX | TX
//#define btSERIAL Serial1
// address of bt board connected to teensy: 98d3:61:fd46d4
// address of bt board connected to Itsy Bitsy: 
void setup()
{
    delay(2000);
  //pin 4 for itsy bitsy, 21 for teensy
 // pinMode(3, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode
 // pinMode(4, OUTPUT);
 // pinMode(7, INPUT); 
 // digitalWrite(4, HIGH);
 // digitalWrite(3, LOW);
//  digitalWrite(3, LOW);
  //delay(8000);
//  digitalWrite(3, HIGH);
  //digitalWrite(4, LOW);
  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  Serial1.begin(9600);  // HC-05 default speed in AT command more
}

void loop()
{  
 // digitalWrite(4, digitalRead(23) );
  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (Serial1.available())
    Serial.write(Serial1.read());

  // Keep reading from Arduino Serial Monitor and send to HC-05
  if (Serial.available())
    Serial1.write(Serial.read());
}
