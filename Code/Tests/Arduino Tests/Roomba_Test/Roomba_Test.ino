/*
 * RoombaBumpTurn 
 * --------------
 * Implement the RoombaComm BumpTurn program in Arduino
 * A simple algorithm that allows the Roomba to drive around 
 * and avoid obstacles.
 * 
 * Arduino pin 10 (RX) is connected to Roomba TXD
 * Arduino pin 11 (TX) is connected to Roomba RXD
 * 
 * Updated 20 November 2006
 * - changed Serial.prints() to use single print(v,BYTE) calls instead of 
 *    character arrays until Arduino settles on a style of raw byte arrays
 *
 * Created 1 August 2006
 * copyleft 2006 Tod E. Kurt <tod@todbot.com>
 * http://hackingroomba.com/
 */

int ledPin = 13;
char sensorbytes[10];

#define bumpright (sensorbytes[0] & 0x01)
#define bumpleft  (sensorbytes[0] & 0x02)
#include <SoftwareSerial.h>
#include <Roomba.h>

//SoftwareSerial mySerial(10, 11); //RX, TX
Roomba robot(0, 0, 0, 10, 11);

void setup() {
//  pinMode(txPin,  OUTPUT);
//  pinMode(ddPin,  OUTPUT);   // sets the pins as output

  pinMode(ledPin, OUTPUT);   // sets the pins as output
  Serial.begin(115200); //default rate
  Serial.println("STARTED");
  
//  mySerial.begin(115200); //default baud rate
    
  digitalWrite(ledPin, HIGH); // say we're alive

  //start robot
  robot.start();
  Serial.println("ROOMBA INIT");
  digitalWrite(ledPin, LOW);
  //robot.turn(-1, 500);

  /*
  // wake up the robot
  digitalWrite(ddPin, HIGH);
  delay(100);
  digitalWrite(ddPin, LOW);
  delay(500);
  digitalWrite(ddPin, HIGH);
  delay(2000);
  // set up ROI to receive commands  
  Serial.print(128, BYTE);  // START
  delay(50);
  Serial.print(130, BYTE);  // CONTROL
  delay(50);
  digitalWrite(ledPin, LOW);  // say we've finished setup
  */
} 

void loop() {
  robot.update();
  digitalWrite(ledPin, HIGH);
  robot.turn(.2, 0);
  //robot.turn(1, 1000);
  
  digitalWrite(ledPin, LOW);
  //robot.turn(-1, 1000);

  /*
  digitalWrite(ledPin, HIGH); // say we're starting loop
  updateSensors();
  digitalWrite(ledPin, LOW);  // say we're after updateSensors
  if(bumpleft) {
    turn(-1, 500);
  }
  else if(bumpright) {
    turn(1, 500);
  }
  goForward();*/
}
/*
void intToHexBytes(byte returnArray[], int num,  int arrLength =2){
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

void printArray(byte array[], int len){
  Serial.println("ARRAY START:");
  for (int i = 0; i < len; i+= 1){
    Serial.println(array[i]);
  }
  Serial.println("ARRAY END");
}

void writeArray(byte bArray[], int len){
  mySerial.write(bArray, len);
}

void start(){
  byte dataArray[] = {128,131};
  writeArray(dataArray, 2);
  delay(100);
}

void turn(float power, int duration){//1 is clockwise, -1 counter
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


void stopRobot(){
  turn(0,0);
}
*/

/*
void moveRobot(int power, int angle){
  int index = 0;
  int max_speed = 500;
  byte dataArray[5];
  dataArray[index++] = 137;
  angleArray = []
  if (angle == 0):
      angleArray += [127, 255]
      
  speedArray = intToHexBytes(power* max_speed)
  dataArray += (speedArray + angleArray)
  writeArray(dataArray)
}
void goForward() {
  moveRobot(1, 0);
}
void goBackward() {
  mySerial.write(137);   // DRIVE
  mySerial.write(0xff);   // 0xff38 == -200
  mySerial.write(0x38);
  mySerial.write(0x80);
  mySerial.write(0x00);
}
void spinLeft() {
  mySerial.write(137);   // DRIVE
  mySerial.write(0x00);   // 0x00c8 == 200
  mySerial.write(0xc8);
  mySerial.write(0x00);
  mySerial.write(0x01);   // 0x0001 == spin left
}
void spinRight() {
  mySerial.write(137);   // DRIVE
  mySerial.write(0x00);   // 0x00c8 == 200
  mySerial.write(0xc8);
  mySerial.write(0xff);
  mySerial.write(0xff);   // 0xffff == -1 == spin right
}
void updateSensors() {
  mySerial.write(142);
  mySerial.write(1);  // sensor packet 1, 10 bytes
  delay(100); // wait for sensors 
  char i = 0;
  while(mySerial.available()) {
    int c = mySerial.read();
    if( c==-1 ) {
      for( int i=0; i<5; i ++ ) {   // say we had an error via the LED
        digitalWrite(ledPin, HIGH); 
        delay(50);
        digitalWrite(ledPin, LOW);  
        delay(50);
      }
    }
    sensorbytes[i++] = c;
  }    
}
*/
