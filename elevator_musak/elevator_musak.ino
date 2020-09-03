/*************************************************** 
 Created 2020-09-02
 By Christopher Brito (me@christopherbrito.com)
 
 ****************************************************/
 
#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Teensy
#define LED_PIN 11
#define SERIAL_PORT Serial1
// SCL Wire Port on 2.0 is 5
// SDA Wire Port on 2.0 is 6
// UART RX Port on 2.0 is 7
// UART TX Port on 2.0 is 8

#define ACC_SENSOR_I2C_ADDRESS 0x18

DFRobotDFPlayerMini myDFPlayer;
Adafruit_LIS3DH accSensor = Adafruit_LIS3DH(); // Sensor I2C Comm
void printDetail(uint8_t type, int value);

void setup()
{
  pinMode(LED_PIN, OUTPUT);

  // Prep the Serial ports for device and USB console
  SERIAL_PORT.begin(9600);
  Serial.begin(9600);

  delay(1500); // Allow some time for things to settle, especially at first boot.

  setupAccSensor();

  randomSeed(analogRead(0)); // Otherwise you see to get 7,9,3,8,0,2...
  
  Serial.println(F(""));
  Serial.println(F("DFPlayer Mini Start-up"));
  Serial.println(F("Initializing DFPlayer... "));
  digitalWrite(LED_PIN, HIGH); 

//  int status = myDFPlayer.begin(SERIAL_PORT);
  
  if (!myDFPlayer.begin(SERIAL_PORT)) {  
    Serial.println(F("Error - unable to start:"));
    Serial.println(F("Check the connection!"));
    Serial.println(F("Check SD card!"));
    while(true){
      // Flash the LED as an error indicator
      flashError();
      // TO DO: Need to find someway to reset / recover from here after some delay.
    }
  }
  Serial.println(F("DFPlayer Mini Online"));
  digitalWrite(LED_PIN, LOW); 

  myDFPlayer.volume(20); // Set initial volume

}

void loop()
{
  
  if (checkIfBoardMoved()) {
    digitalWrite(LED_PIN, HIGH); 
    playMusic();
    delay(10000);
    digitalWrite(LED_PIN, LOW);
    myDFPlayer.stop();
    delay(5000);
  }
  else {
    delay(30000); // Put in some delay so it does not just keep looping to a random number...
  }
  
  if (myDFPlayer.available()) {
    //Print the detail message from DFPlayer to handle different errors and states.
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); 
    flashError();
  }
}

void flashError(){
      digitalWrite(LED_PIN, LOW);
      delay(250); 
      digitalWrite(LED_PIN, HIGH); 
      delay(250); 
}

void setupAccSensor(){
  if (! accSensor.begin(ACC_SENSOR_I2C_ADDRESS)) {   
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH sensor found!");
  
  accSensor.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
}

void playMusic(){
  // We should read the number of available tracks and do a rand to choose one to play.
  myDFPlayer.volume(30); // Always set volume
  delay(100);
  myDFPlayer.play(1); // Play the first mp3
}

/* 
 *  Checks if the board has moved since it's last check. 
 *  This should happen nearly constantly until the end of a song and then we should
 *  idle for a while.
 *  
 *  A more advanced version of this can stop playing the mysic once the elevator has stopped
 *  by using a flag in the calling routine to not do anything play/stop while we detected a move
 *  and are stillmoving.
 */

bool checkIfBoardMoved() {
  Serial.println(F("Checking if the board has moved."));

  getAccSensorMeasurements();

  int randNum = random(10); // Roll a die!
  Serial.print("Rolled a: ");
  Serial.println(randNum);

  if(randNum % 2 == 0){
    // Number was even
    Serial.println(F("Board has NOT moved."));
    return false;
  }
  else {
    // Number was odd
    Serial.println(F("Broad has moved!"));
    return true;
  }

}

void getAccSensorMeasurements() {
  sensors_event_t event; 
  accSensor.getEvent(&event);
  
  /* Display the results (acceleration is measured in m/s^2) */

  Serial.print("Z-axis accelerated: "); Serial.print(event.acceleration.z); 

  Serial.println();

}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
  
}
