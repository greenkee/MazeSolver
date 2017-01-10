import time, math
def intToHexBytes(integer):
        numString = ""
        if(integer >= 0):
            numString = '{0:04X}'.format(integer)
        else:
            numString = '{0:04X}'.format(-integer)
            newNum = [0 for x in range(len(numString))]
            for x in range(len(numString)):
                num = 15 - int(numString[x], 16) # F - positive hex digit to convert to neg
                newNum[x] = num
                if(x == (len(numString)-1)):
                    num += 1
                    if(num == 16): #in case last digit is already F, need to carry over
    			currDigit = 16
    			while(currDigit == 16): #bump value by one (carrying)
    				newNum[x] = 0
    				x -= 1
    				newNum[x] += 1
    				currDigit = newNum[x]
		    else:
			newNum[x] = num
	    newString = ""
	    for x in range(len(newNum)):
		    newString += '{0:01X}'.format(newNum[x])
            numString = newString
        byteArray = [ int(numString[:2],16), int(numString[2:], 16) ]
        return byteArray



def bytesToLong(high, low):
    return (ord(high) << 8) + ord(low)
    
class Roomba:
    def updateGyroEst(self, angleRate): #rate of gyro spinning
        angle = self.gyroEst[2]#gets last angle
        
        gyroError = 0 #to compensate for gyro drift
        angleRate += gyroError
        
        angle += angleRate#gets new angle
        estimate = [0 for i in range(3)]#x, y, angle
        dCenter = ( self.encoderVals[1] + self.encoderVals[3]) / 2 #distance center of robot moved

        if(valid):
            self.gyroEst[3] = dCenter * math.cos(angle) 
            self.gyroEst[4] = dCenter* math.sin(angle)
            self.gyroEst[5] = angleRate
        else:
            print "USE LAST VALUES"

        #add rates to last vals to get new vals
        self.gyroEst[0] += self.gyroEst[3]
        self.gyroEst[1] += self.gyroEst[4]
        self.gyroEst[2] += self.gyroEst[5]
            
            
        
    def updateOdomEst(self, valid, elapsed):
        dist_between_wheels = 230.0  #in mm
        angle = self.odomEst[2] #gets last angle
        '''
        print(F("Robot vals"))
        print( self.encoderVals[0])
        print( self.encoderVals[1])
        print( self.encoderVals[2])
        print( self.encoderVals[3])'''
        
        
        if(valid):#max is 500mm/sec units should be < 500mm*2*time/1000
            #update position with odometry
            if( abs( self.encoderVals[3] - self.encoderVals[1]) < 10): #tolerance robot moving straight
                self.odomEst[3] = ( self.encoderVals[1]+ self.encoderVals[3])/2 * math.cos(angle) #gets dx (averages data from both wheels to get speed)
                self.odomEst[4] = ( self.encoderVals[1]+ self.encoderVals[3])/2 *math.sin(angle) #gets dy (using average dist moved)
                self.odomEst[5] = 0 #if going straight, no change in angle
               
            else: #robot travelling in arc   
            
                dCenter = ( self.encoderVals[1] + self.encoderVals[3]) / 2 #distance center of robot moved
                phi = ( self.encoderVals[3] - self.encoderVals[1] ) / dist_between_wheels #angle that robot moved around center
                rCenter = dCenter/phi #solve for center r*theta = arcLength        
                    
                self.odomEst[3] = rCenter*( math.sin(phi + angle) -math.sin(angle) )  #calculate dx based on geometry
                self.odomEst[4] = rCenter*( math.cos(angle) - math.cos(phi + angle) ) #calculate dy based on geometry
                self.odomEst[5] = phi #dAngle = phi
                

        else: #if not valid, pretend robot moved same as it did the last time, keep things the same
            print "LAST VALS PRESERVED"
                
            #print("TOO LARGE STEP")
                    
            '''
            print(F("START"))
            print(self.prevState[3])
            print(self.prevState[4])
            print(self.prevState[5])'''
        #add rates to last vals to get new vals
        self.odomEst[0] += self.odomEst[3]
        self.odomEst[1] += self.odomEst[4]
        self.odomEst[2] += self.odomEst[5]

        tau = 6.28318
        self.odomeEst[2] %= tau
        


    def updateSensors(self, elapsed):
        #print("Update Sens")
        dataSize = 4 # call, num of values, 2 values
        dataArray = [149, (dataSize - 2), 43, 44] #left is 43, right is 44???
        self.writeArray(dataArray)
        time.sleep(.03) #wait for sensor values to return
        index = 0
        val = -1
        valid = True
        while(self.serial.inWaiting() > 0 and index < len(self.encoderVals)): #gets both encoder values  - 600 is about 300 mm (2 units is 1 mm)
            high = self.serial.read()
            low = self.serial.read()
            reading = bytesToLong(high, low)
            
            val = reading - self.encoderVals[index]
            
            if(val < -1000):
                val += 65535 #encoder jumped value add max val for encoder to compensate
            
            if(abs(val) > elapsed*2000): #convert time to ms, then check if over twice time
                valid = False
                print("BAD READING")
                print val, elapsed
            self.encoderVals[index] = reading
            index+= 1

            self.encoderVals[index] = round(val/2) #divide to convert to mm
            index += 1
        #print(valid)
        #print self.encoderVals
        return valid

    def updatePosition(self, valid, angle):
        #print "EVALS", self.encoderVals[1], self.encoderVals[3]
        if(valid):
            self.vals[3] = ( self.encoderVals[1]+ self.encoderVals[3])/2 * math.cos(angle)
            self.vals[4] = ( self.encoderVals[1]+ self.encoderVals[3])/2 *math.sin(angle)
            self.vals[5] = angle - self.vals[2]
        else:
            print "USE LAST VALS"
        self.vals[0] += self.vals[3]
        self.vals[1] += self.vals[4]
        


    def updateLidar(self, reading):
        self.lidar = reading
    
    def update(self, elapsed, angle, lidar, updatePos):
        #print(F("UPDATE"))
        self.vals[2] = angle
        self.updateLidar(lidar)
        valid = self.updateSensors(elapsed)
        if(updatePos): #whether we want to update based on encoder data
            self.updatePosition(valid, angle)
        

        '''
        self.updateOdomEst(valid, elapsed) #valid
        self.updateGyroEst(valid, elapsed)
            #updateKalman(valid, time, measurements)'''

    def initSensors(self):
        dataArray = [149, 2, 43, 44]
        self.writeArray(dataArray)
        time.sleep(.2) #wait for sensor values to return
        index = 0
        while(self.serial.inWaiting() > 0 and index < len(self.encoderVals)): #gets both encoder values  - 600 is about 300 mm (2 units is 1 mm)
            high = self.serial.read()
            low = self.serial.read()
            val = bytesToLong(high, low)
            self.encoderVals[index] = val
            index+=2
        print self.encoderVals

    def getX(self):
        return self.vals[0]
    def getY(self):
        return self.vals[1]
    def getAngle(self):
        return self.vals[2]
    def getLidar(self):
        return int(self.lidar)
    def setPlanning(self, b):
        self.planning = b
    def getPlanning(self):
        return self.planning

    def __init__(self, x, y, a, serial, wobbling, backwards, size):
        self.vals = [0 for i in range(6)]
        self.gyroEst = [0 for i in range(6)]
        self.odomEst = [0 for i in range(6)]
        self.encoderVals = [0 for i in range(4)] #left, dL, right, dR
        self.lidar = 0
        self.wobbling = wobbling
        self.wobbleCounter = 0 #<5 is left, > 5 is right #True is left, false is right
        self.backwards = backwards #direction roomba is driving
        self.robotSize = size

        self.planning = False

        #current guess
        self.vals[0] = x
        self.vals[1] = y
        self.vals[2] = a
        self.vals[3] = 0.0
        self.vals[4] = 0.0
        self.vals[5] = 0.0

        
        self.odomEst[0] = x
        self.odomEst[1] = y
        self.odomEst[2] = a
        self.odomEst[3] = 0.0
        self.odomEst[4] = 0.0
        self.odomEst[5] = 0.0

        self.gyroEst[0] = x
        self.gyroEst[1] = y
        self.gyroEst[2] = a
        self.gyroEst[3] = 0.0
        self.gyroEst[4] = 0.0
        self.gyroEst[5] = 0.0
        
        self.serial = serial

    
    def writeArray(self, arr):
        for x in range(len(arr)):
            self.serial.write(chr(int(arr[x])))

    def start(self):
        self.writeArray([128, 131])
        time.sleep(.1)
        self.initSensors()
        print "ROOMBA INIT"

    def drive(self, power, radius):
        speed = power*500 
        #send command
        if(radius != 0 and abs(radius*100) < 1): #.03 is min steering radius
            #Serial.println(F("TOO SMALL"))
            #Serial.println(radius)
            if(radius < 0):
                radius = -.01
            else:
                radius = .01
        dataArray = [137]
        dataArray += intToHexBytes((int)(power*500))
        if(radius == 0):
            dataArray+= intToHexBytes(32767)
        else:
            #print radius*2000
            dataArray+= intToHexBytes((int)(radius*2000))
        self.writeArray(dataArray)

    def tankDrive(self, power, powerDiff): #from -2 to 2; positive is left, negative right
        wobbleStep = 5
        speed = power*500
        if(self.backwards):
            speed*= -1
        dataArray =[145]
        rightSpeed = speed
        leftSpeed = speed
        if(powerDiff > 0): #wants to have more power to the right (turns left)
            leftSpeed *= (1-powerDiff) #limit right motor
        else: #more power to right (or equal)
            rightSpeed *= (1+powerDiff) #limit left motor
        if(self.backwards): #switches which motor is which
            temp = leftSpeed
            leftSpeed = rightSpeed
            rightSpeed = temp
        if(self.wobbling):
            if(self.wobbleCounter < wobbleStep):
                leftSpeed *= 0.5
            else:
                if(self.wobbleCounter >= (wobbleStep*2 - 1)):
                    self.wobbleCounter = -1 #so it will be 0 when incremented
                rightSpeed *= 0.5
            self.wobbleCounter += 1
        print rightSpeed, leftSpeed
        dataArray += intToHexBytes((int)(rightSpeed)) #set right motor
        dataArray += intToHexBytes( (int)(leftSpeed)) #set left motor
        self.writeArray(dataArray)
        
        

    def turn(self, speed, duration): #1 is counter(left), -1 clockwise(right)
        max_pwm = 255
        dataArray = [146]
        dataArray += intToHexBytes((int)(speed* max_pwm)) #set right motor
        dataArray += intToHexBytes( (int)(-speed* max_pwm)) #set left motor
        self.writeArray(dataArray)
        if(duration > 0):
            time.sleep(duration)
            self.stop()

    def stop(self):
        self.drive(0, 0)

    def end(self):
        self.stop()
        self.serial.close()
