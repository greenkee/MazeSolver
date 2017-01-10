import struct
def bytesToLong(high, low):
    return (ord(high) << 8) + ord(low)

class Arduino:
    def __init__(self, serial):
        self.serial = serial

    def end(self):
        self.serial.flushInput()
        self.serial.close()

    def getReadings(self):
        fullReading = []
        curr = ""
        #check for end of previous message, to ensure full reading (nothing cut off)
        while(curr != '@'): #ends with @
            curr = self.serial.read()     #discard until @ reached
        fullReading.append(self.readValue('!')) #gyro reading ends with !
        fullReading.append(self.readValue('@')) #lidar reading ends with @

        self.serial.flushInput() #need to flush to get newest serial data, since arduino writes faster than code reads
        return fullReading

    def readValue(self, endChar):
        reading = ""
        curr = ""
        while(self.serial.inWaiting > 0):#get reading
            curr = self.serial.read()
            if(curr == endChar): #data is separated by ! or @
                break
            reading += curr
            #print curr
        return reading

    def readAngle(self):
        reading = ""
        curr = ""
        #check for end of previous message, to ensure full reading (nothing cut off)
        while(curr != '!'):
            curr = self.serial.read()     #discard until ! reached   
        while(self.serial.inWaiting > 0):#get reading
            curr = self.serial.read()
            if(curr == '!'): #data is separated by !
                break
            reading += curr
            #print curr
        self.serial.flushInput() #need to flush to get newest serial data, since arduino writes faster than code reads
        return reading
    
    
