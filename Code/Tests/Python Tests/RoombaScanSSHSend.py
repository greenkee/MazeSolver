import serial, time, math
from Arduino import Arduino
from Roomba import Roomba
from Utils import Node, PriorityQueue, Counter

import shapely.geometry as sg
import RoombaFunctASTAR
import RoombaFunctParseData
import Utils


accum_error = 0
diff_error = 0
cte = 0
prev_cte = 0

def PID(path, params, elapsed): #time elapsed in seconds
    global cte, prev_cte, accum_error, diff_error, pidIndex
    prev_cte = cte
    nextPoint = path[pidIndex+1]
    #print "NEXT", nextPoint
    '''
    xDist = roomba.getX() - path[pidIndex][0]
    yDist = roomba.getY() - path[pidIndex][1] #error for y

    xLength = path[pidIndex+1][0] - path[pidIndex%len(path)][0] #horiz length of current path
    yLength = path[pidIndex+1][1] - path[pidIndex%len(path)][1] #vert length of current path

    u = float( xDist * xLength + yDist*yLength) / float(xLength*xLength + yLength*yLength) #percent of path traveled - something to do with vector projection
    '''
    #cte = (float)yDist
    goalAngle = math.atan2( (nextPoint[1] - roomba.getY()),(nextPoint[0] - roomba.getX())) % Utils.tau
    angleDiff = Utils.findAngleDiff(goalAngle, roomba.getAngle())

    if(roomba.getPlanning() and angleDiff < .2):#basically reached target heading
        roomba.setPlanning(False)
    
    #cte = (float( yDist * xLength - xDist*yLength) / float(xLength*xLength + yLength*yLength)) -  angleDiff 
    cte = -angleDiff
    if(Utils.reachedPoint(roomba.getX(), roomba.getY(), nextPoint[0], nextPoint[1])):
        pidIndex  += 1#+= 1 
        #print "REACHED POINT", nextPoint[0], nextPoint[1]

    '''
    if(pidIndex == (len(path)-2)):
        #print cte, goalAngle, roomba.getAngle(), angleDiff
        if(reachedPoint(roomba.getX(), path[pidIndex+1][0], roomba.getY(), path[pidIndex+1][1])):
            print path[pidIndex%len(path)][0]
            print path[pidIndex%len(path)][1]
            print "REACHED POINT
            pidIndex  = -1#+= 1 

    elif(u > 1):
        pidIndex += 1
        print ("REACHED POINT")
        print path[pidIndex%len(path)][0]
        print path[pidIndex%len(path)][1]
        '''
  
    diff_error = float(cte - prev_cte)/elapsed # maybe multiply 3000 as constant
    accum_error += cte*elapsed

    
    steeringRad = -params[0] * cte - params[1] * diff_error - params[2] * accum_error
    
    #print "PID", cte, diff_error, steeringRad

    #steeringRad /= (params[0] * tau/2) #max error using only P vals - normalization?

    if(steeringRad > 2):
        steeringRad = 2
    elif(steeringRad < -2):
        steeringRad = -2
    #print "ADIFF", math.degrees(angleDiff), goalAngle, roomba.getAngle()
    #print steeringRad
    roomba.tankDrive(.3, steeringRad)







def replan(walls):
  global pidIndex, timeStep, robotSize, gapTolerance, path, t2, t1
  #print "REPLAN"
  roomba.stop()

  #determine turning direction
  goalAngle = math.atan2( (path[pidIndex+1][1] - roomba.getY()),(path[pidIndex+1][0] - roomba.getX())) % Utils.tau
  angleDiff = Utils.findAngleDiff(goalAngle, roomba.getAngle())
  direction = 0 #which direction robot should turn 1 is left, -1 is right
  if(abs(angleDiff) > .2): #significant enough difference; around 12 degrees
    if(angleDiff > 0):
      direction = 1
    else:
      direction = -1
  elif (path[pidIndex+1][0] - roomba.getX() != 0): #check for slope that is not undefined
    slope = (path[pidIndex+1][1] - roomba.getY()) / (path[pidIndex+1][0] - roomba.getX())
    if(slope > 0):
      direction = 1
    else:
      direction = -1
  else:
    direction = -1 #turns right by default
  print direction
  
  velocity = .3*direction#.3 is current turning speed, turns indefinitely

  #init vals
  prevAngle = roomba.getAngle()
  totalChange = 0
  
  foundOpening = False
  
  finalPoint = (-1, -1, -1)
  
  #startRadius = minDist*10+ robotSize / 2 #convert distance to mm + robot radius
  #startPoint = startPoint = (roomba.getX() + startRadius * math.cos(roomba.getAngle()), roomba.getY() + startRadius * math.sin(roomba.getAngle()), roomba.getAngle())#marks start of wall

  roomba.turn(velocity, 0)

  dataList = []
  
  while(totalChange < Utils.tau): #while not 360 degree turn
    #update pos 
    time.sleep(timeStep/2)
    t2 = time.time() - t1
    t1 = time.time()
    updateVals(t2, False) #ignore encoder data, use only gyro, lidar
    
    totalChange += (abs(Utils.findAngleDiff(prevAngle, roomba.getAngle())))
    #print "CHANGE", totalChange, prevAngle, roomba.getAngle()
    prevAngle = roomba.getAngle()
    dataList.append((roomba.getX(), roomba.getY(), roomba.getAngle(), roomba.getLidar()))
  #spun full circle
          
  roomba.stop()
  roomba.setPlanning(True)
  walls = RoombaFunctParseData.manageData(dataList, walls, (roomba.getX(), roomba.getY()), minDist, sightThreshold,
                                        angleTolerance, robotWidthAndBuffer, samePointDist)
  #print "NEW WALLS"
  printWalls(walls)

    
  return RoombaFunctASTAR.replanASTAR((roomba.getX(), roomba.getY()), goal, walls, robotWidthAndBuffer ) #goal is last entry in path, walls

  #path.insert(pidIndex + 1, targetPoint) #this is the new goal

  '''
  roomba.stop()
  while(True):
    time.sleep(5)'''


def updateVals(timeStep, updatePos):
    global start, kDrift, initAngle
    reading = arduino.getReadings() #reading = (arduino.readAngle(), 0)
    elapsed = time.time() - start
    angle = float(reading[0]) - initAngle + elapsed*-kDrift
    #print "READING", reading[0], " E:",elapsed, " CHANGE:", ( - initAngle + elapsed*-kDrift)
    radAngle = (math.radians(angle)) % Utils.tau #keeps angle within [0, 2pi]

    lidarDist = int(reading[1])
        
    roomba.update(t2, radAngle, lidarDist, updatePos)


def endProgram():
  printWalls(walls)
  roomba.end()
  arduino.end()

#variables
timeStep = .05
t1= 0
t2 = 0
initAngle = 0.0
prevAngle = 0.0
scalingFactor = .1

#search vars
start = (0,0)
goal = (2000, 0)
path = [start, goal]
walls = []
pidIndex = 0

#PID vars
tau_p = 3.0
tau_i = 0.0
tau_d = 0.0
params = [tau_p, tau_i, tau_d]

#Tolerance Vars
#all in mm
robotSize = 351
samePointDist = 30 #for points to be considered the same (and be averaged)
sightThreshold = 1000 #min dist for point to be considered as wall
angleTolerance = Utils.tau/8 #diff in slope

minDist = robotSize/1.5 #between points
wallBuffer = robotSize/3 #buffer around walls
robotWidthAndBuffer = robotSize/2 + wallBuffer

minDetect = robotSize * .7 #max val until stop

viaPoint = None

#Serial Ports

#for PC
#roombaSerName = 'COM3'
#arduinoSerName = 'COM4'

#for Linux
roombaSerName = '/dev/ttyUSB0'
arduinoSerName = '/dev/ttyACM0' 
roombaSer = serial.Serial(port = roombaSerName, baudrate = 115200, bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 1)
arduinoSer = serial.Serial(port = arduinoSerName, baudrate = 115200, bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 1)



    
roomba = Roomba(0, 0, 0, roombaSer, False, False, 343) #note: backwards doesn't work with position
arduino = Arduino(arduinoSer)
roomba.start()



try:
  reading = arduino.getReadings()
  initAngle = float(reading[0]) + roomba.getAngle()
  initDist = reading[1] #get initial lidar reading
  roomba.updateLidar(initDist)
except (KeyboardInterrupt, SystemExit):
    print "VALS", roomba.getX(), roomba.getY(), roomba.getAngle(), roomba.getLidar()
    
    roomba.end()
    arduino.end()
    raise Exception("STOPPED")

#CALCULATE DRIFT - find slope at 5 points, calc average
drift = []
t1= time.time()
for x in range(5):
    reading = arduino.getReadings()
    angle = float(reading[0]) - initAngle
    t2=time.time()-t1
    t1=time.time()
    diff = (angle - prevAngle)/t2 #gets change in angle/sec
    drift.append(diff)
    prevAngle = angle
    time.sleep(1)
kDrift = sum(drift)/len(drift)
#print "IA:", initAngle,  " KD:", kDrift

start = time.time()
t1 = start

while (pidIndex < (len(path)-1)):
    try:
        time.sleep(timeStep) #in seconds
        t2 = time.time() - t1
        t1 = time.time()
        updateVals(t2, True)

        #print "VALS", roomba.getX(), roomba.getY(), roomba.getAngle(), roomba.getLidar()
        
        #allows it to reach goal, even by wall; converts minDetect to mm
        if( (not roomba.getPlanning()) and (roomba.getLidar()*10 < minDetect)
            and (roomba.getLidar()*10 < Utils.getDist(roomba.getX(), roomba.getY(), path[pidIndex +1][0],  path[pidIndex+1][1])) ):
          #print "TOO CLOSE"
          path = replan(walls)
          #print "NEW PATH", path
          pidIndex = 0 #reset path to be where roomba is
        PID(path, params, t2)
        
    except (KeyboardInterrupt, SystemExit):
        #print "VALS", roomba.getX(), roomba.getY(), roomba.getAngle(), roomba.getLidar()
        endProgram()
        raise Exception("STOPPED")
        break
endProgram()
    
