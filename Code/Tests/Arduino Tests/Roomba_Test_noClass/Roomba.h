#ifndef Roomba_h
#define Roomba_h

#include "Arduino.h"
#include <SoftwareSerial.h>

class Roomba
{
  public:
    Roomba(float x, float y, float a, int rx, int tx);
    void start();
    void turn(float power, int duration);
    void stopRobot();
    void writeArray(byte bArray[], int len);
    void intToHexBytes(byte returnArray[], int num,  int arrLength = 2);
    void update();
    void printArray(byte array[], int len);
    
    
  private:
    int xPos;
    int yPos;
    int angle;
    double t1, t2;
    SoftwareSerial *mySerial;
};

#endif
