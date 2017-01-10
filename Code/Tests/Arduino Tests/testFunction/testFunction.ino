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
  printArray(returnArray, arrLength);
}

void printArray(byte array[], int len){
  Serial.println("ARRAY START:");
  for (int i = 0; i < len; i+= 1){
    Serial.println(array[i]);
  }
  Serial.println("ARRAY END");
}
int ledPin = 13;

void setup() {
  Serial.begin(115200); //default rate
  Serial.println("START");
  byte dataArray[3];
  dataArray[0] = 127;
  
  byte byteArray[2];
  intToHexBytes(byteArray, -300);
  memcpy(dataArray+1, byteArray, 2);
  intToHexBytes(byteArray, 400, 2);
  memcpy(dataArray+3, byteArray,2);
  
  printArray(dataArray, 5);
  pinMode(ledPin, OUTPUT);
  Serial.println("DONE");
  /*
  printArray(byteArray, 2);
  intToHexBytes(byteArray, 2, -300);
  printArray(byteArray, 2);
  */
}


void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("LOOPING");
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);
}

