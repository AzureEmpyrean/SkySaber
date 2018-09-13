/*
  Listfiles

 This example shows how print out the files in a
 directory on a SD card

 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe
 modified 2 Feb 2014
 by Scott Fitzgerald

 This example code is in the public domain.

 */
#include <SPI.h>
#include <SD.h>

File root;
String fontList[21];


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  root = SD.open("/");

  addFonts(root);

  Serial.println("done!");
}

void loop() {
  // nothing happens after setup finishes.
}

void addFonts(File dir) {

int folderNum =0;

while (true) {

    File entry =  dir.openNextFile();
    if (! entry || folderNum = 20) {
      fontList[20]= (String)folderNum;
      // no more files
      break;
    }

    Serial.print(entry.name());
    if (entry.isDirectory()) {
        fontList[folderNum]=entry.concat("/");
        
      folderNum++;
    }
    entry.close();
  }
    
}


void addFiles(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
    }
    entry.close();
  }
    
}

void Font(){
  String Swing[20];
  String Clash[20];
  String Boot[20];
  
  
  
}


