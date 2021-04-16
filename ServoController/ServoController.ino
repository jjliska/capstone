#include <Servo.h>
#include "StringSplitter.h"
#include <EEPROM.h>

// Arm rotors 
Servo a1Servo;
Servo a2Servo;
Servo a3Servo;
Servo a4Servo;

const int servoPins[] = {20,21,22,23};

// Arm angles
float a1 = 142.0;
float a2 = 180.0-76.0;
float a3 = 142.0;
float a4 = 90.0/1.5;

// Arm angles
float a1Last = a1;
float a2Last = a2;
float a3Last = a3;
float a4Last = a4;

float cura1 = a1;
float cura2 = a2;
float cura3 = a3;
float cura4 = a4;

unsigned long servoTimer = micros();
unsigned long eepromTimer = millis();
const int eepromDelay = 2000;
const int delayTime = 100;

//Average time (in ms) between serial.write() on python script
const int timeForMessages = 10000;

//Rate of which the machine is smoothed so i runs every "milisecond" it could run faster
//to create even smoother lines but since its run on parsecs and we only have a degree of
//accuracy of about 1.2 +- some deflection it should be fine 
const float smoothingRate = float(delayTime)/float(timeForMessages);

int readMessages = 0;

unsigned long messageTimer = 0;

void setup() {
  while(!Serial){}
  a1Servo.attach(servoPins[0]);
  a2Servo.attach(servoPins[1]);
  a3Servo.attach(servoPins[2]);
  a4Servo.attach(servoPins[3]);

  //
  String angles = readStringFromEEPROM(0);
  StringSplitter *splitter = new StringSplitter(angles, ',', 4);
  String a1temp = splitter->getItemAtIndex(0);
  String a2temp = splitter->getItemAtIndex(1);
  String a3temp = splitter->getItemAtIndex(2);
  String a4temp = splitter->getItemAtIndex(3);
  
  if(a1temp >= 0 || a1temp <= 180) cura1 = a1temp.toFloat();
  if(a2temp >= 45 || a2temp <= 180) cura2 = a2temp.toFloat();
  if(a3temp >= 45 || a3temp <= 180) cura3 = a3temp.toFloat();
  if(a2temp >= 0 || a2temp <= 270) cura4 = a4temp.toFloat();

  Serial.print(String(cura1)+","+String(cura2)+","+String(cura3)+","+String(cura4));
  messageTimer = micros();
}

void loop() {
  //Reads from serial and parses into
  if (Serial.available() > 0) {
    String angles = Serial.readStringUntil('\n');
    StringSplitter *splitter = new StringSplitter(angles, ',', 4);
    String a1temp = splitter->getItemAtIndex(0);
    String a2temp = splitter->getItemAtIndex(1);
    String a3temp = splitter->getItemAtIndex(2);
    String a4temp = splitter->getItemAtIndex(3);
    // Angle checks as it seems if the python script throws an error in the buffer it will
    // throw a massive angle and the system will try to go to that massive angle
    if(a1temp >= 0 || a1temp <= 180) a1 = a1temp.toFloat();
    if(a2temp >= 45 || a2temp <= 180) a2 = abs(180.0-a2temp.toFloat());
    if(a3temp >= 45 || a3temp <= 180) a3 = a3temp.toFloat();
    if(a2temp >= 0 || a2temp <= 270) a4 = (a4temp.toFloat()/1.5);
    String eepString = String(a1) + "," + String(a2) + "," + String(a3) + "," + String(a4);
    if(millis()-eepromTimer >= eepromDelay){
      writeStringToEEPROM(0, eepString);
    }
  }
  // Controlls the speed at which the easing moves
  if((micros()-servoTimer) >= delayTime){
    angleHandler();
    servoTimer = micros();
  }
}

//Essentially if the angle is not the angle
void angleHandler(){
  if(cura1 != a1){
    cura1 = moveToAngle(a1,cura1,a1Servo);
    //Serial.print(a1);Serial.print(",");Serial.println(cura1);
  }
  if(cura2 != a2){
    cura2 = moveToAngle(a2,cura2,a2Servo);
    //Serial.print(a2);Serial.print(",");Serial.println(cura2);
  }
  if(cura3 != a3){
    cura3 = moveToAngle(a3,cura3,a3Servo);
    //Serial.print(a3);Serial.print(",");Serial.println(cura3);
  }
  if(cura4 != a4){
    cura4 = moveToAngle(a4,cura4,a4Servo);
    //Serial.print(a4);Serial.print(",");Serial.println(cura4);
  }
  //Serial.print(cura1);Serial.print(",");Serial.print(cura2);Serial.print(",");Serial.print(cura3);Serial.print(",");Serial.println(cura4);
}

//Coasts an angle from point a to point b using linear smoothing so instead of moving in one large jump it does several small jumps to attempt to give smoother movement
// You can create even more smoothing by adding a greater smoothing effect but this gives a delayed buffer that makes the algorithm overshoot
float moveToAngle(float input, float currentAngle, Servo servoName){
  float retFloat;
  float difference = abs(input-currentAngle);
  if(currentAngle > input){
    if((currentAngle - input) > (difference*smoothingRate)){
      retFloat = currentAngle-(difference*smoothingRate);
      writeToServo(servoName,retFloat);
      return retFloat;
    }
    else{
      writeToServo(servoName,input);
      return input;
    }
  }
  else if(currentAngle < input){
    if((currentAngle - input) < (-1*(difference*smoothingRate))){
      retFloat = currentAngle+(difference*smoothingRate);
      writeToServo(servoName,retFloat);
      return retFloat;
    }
    else{
      writeToServo(servoName,input);
      return input;
    }
  }
}

//Writes to the given servo a specific angle translated into parsecs to get the best degree of accuracy
void writeToServo(Servo servoName, float angle){
  int microSecondsTemp = int(2000.0*((angle)/180.0)+500.0);
  servoName.writeMicroseconds(microSecondsTemp);
  //Serial.println(microSecondsTemp);
}

String readStringFromEEPROM(int addrOffset){
  int newStrLen = EEPROM.read(addrOffset);
  char data[newStrLen + 1];
  for (int i = 0; i < newStrLen; i++){
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\0';
  return String(data);
}

void writeStringToEEPROM(int addrOffset, const String &strToWrite){
  byte len = strToWrite.length();
  EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++){
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
}
