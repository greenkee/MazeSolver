#include "Roomba.h"
#include "Arduino.h"
#include <SoftwareSerial.h>

inline void Roomba::intToHexBytes(byte returnArray[], int num,  int arrLength){
    String numString = "";
  int targetLength = 4;//4 is the length we want to break into high and low bytes
  if(num >= 0){
    numString = String(num, HEX);
    while(numString.length() < targetLength){ 
      numString = "0"+numString;
    }
  }
  else{
    numString = String(-num, HEX);
    while(numString.length() < targetLength){ 
      numString = "0"+numString;
    }
    String newString = "";
    for(int x = 0; x < numString.length(); x++){
        char temp[] = {numString[x]};
        int digit = 15 - (int)(strtol(temp, NULL, 16)); // # F - positive hex digit to convert to neg
        if(x == (numString.length()-1)){
          digit += 1;
        }
        newString += String(digit, HEX);
    }
    numString = newString;

  }
  char tempByte[2];
  numString.substring(0,2).toCharArray(tempByte, 3);
  returnArray[0] = (int)strtol(tempByte, NULL, 16);
  numString.substring(2).toCharArray(tempByte, 3);
  returnArray[1] = (int)strtol(tempByte, NULL, 16);
}

inline void Roomba::update(){
  if(mySerial -> available()){
    Serial.write(mySerial ->read());
    t2 = millis()-t1;
  }
}

inline void Roomba::writeArray(byte bArray[], int len){
  mySerial -> write(bArray, len);
}

inline void Roomba::turn(float power, int duration){
  if(abs(power) > 1){
    Serial.println("TOO MUCH TURN POWER");
    return;
  }
  int pairSize = 2;
  int dataSize = 5;
  int index = 0;
  byte dataArray[dataSize];
  int max_pwm = 255;
  dataArray[index++] = 146;
  byte tempArray[pairSize];
  int speed = power*max_pwm;
  intToHexBytes(tempArray, -speed, pairSize);
  memcpy(dataArray + index, tempArray, pairSize);
  index += pairSize;
  intToHexBytes(tempArray, speed, pairSize);
  memcpy(dataArray + index, tempArray, pairSize);
  index += pairSize;

  printArray(dataArray, dataSize);
  writeArray(dataArray, dataSize);
  
  delay(duration);
  Serial.println("FINISHED");
  if(abs(power) > 0){
    stopRobot();
  }
}

inline void Roomba::printArray(byte array[], int len){
  Serial.println("ARRAY START:");
  for (int i = 0; i < len; i+= 1){
    Serial.println(array[i]);
  }
  Serial.println("ARRAY END");
}

inline Roomba::Roomba(float x, float y, float a, int rx, int tx){
  xPos = x;
  yPos = y;
  angle = a;
  Serial.println("CONSTRUCTOR");
  *mySerial = new SoftwareSerial(rx, tx);
  mySerial -> begin(115200);
  t1 = millis();
}

inline void Roomba::start(){
  byte dataArray[] = {128,131};
  writeArray(dataArray, 2);
  delay(100);
}

inline void Roomba::stopRobot(){
  turn(0,0);
}

