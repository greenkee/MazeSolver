import serial, time, math
from Arduino import Arduino
from Roomba import Roomba
import turtle

def reachedPoint(x1, y1, x2, y2):
  tolerance = 100;
  return (  getDist(x1, y1, x2, y2) < tolerance)

def getDist(x1, y1, x2, y2):
  return math.sqrt( float((x1-x2)**2) + float((y1-y2)**2) )

#PID Vars
tau_p = 3.0
tau_i = 0.0
tau_d = 0.0

accum_error = 0
diff_error = 0
cte = 0
prev_cte = 0
params = [tau_p, tau_i, tau_d]

def findAngleDiff(goal, curr):
  angleDiff = (goal - curr) #direction robot needs to turn
  if(abs(angleDiff) > tau/2):
      if(angleDiff < 0):
        angleDiff += tau
      else:
        angleDiff -= tau
      #angleDiff = (angleDiff- tau)#%tau
  return angleDiff

def PID(path, params, elapsed): #time elapsed in seconds
    global cte, prev_cte, accum_error, diff_error, pidIndex, viaPoint
    prev_cte = cte
    nextPoint = path[pidIndex+1]
    if(viaPoint != None):
      nextPoint = viaPoint
    #print "NEXT", nextPoint
    '''
    xDist = roomba.getX() - path[pidIndex][0]
    yDist = roomba.getY() - path[pidIndex][1] #error for y

    xLength = path[pidIndex+1][0] - path[pidIndex%len(path)][0] #horiz length of current path
    yLength = path[pidIndex+1][1] - path[pidIndex%len(path)][1] #vert length of current path

    u = float( xDist * xLength + yDist*yLength) / float(xLength*xLength + yLength*yLength) #percent of path traveled - something to do with vector projection
    '''
    #cte = (float)yDist
    goalAngle = math.atan2( (nextPoint[1] - roomba.getY()),(nextPoint[0] - roomba.getX())) % tau
    angleDiff = findAngleDiff(goalAngle, roomba.getAngle())
    
    #cte = (float( yDist * xLength - xDist*yLength) / float(xLength*xLength + yLength*yLength)) -  angleDiff 
    cte = -angleDiff
    if(reachedPoint(roomba.getX(), roomba.getY(), nextPoint[0], nextPoint[1])):
        if(viaPoint != None):#get rid of via point
          viaPoint = None
        else:#or go to next path point
          pidIndex  += 1#+= 1 
        print nextPoint[0]
        print nextPoint[1]
        print "REACHED POINT"

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

    if(steeringRad > 1):
        steeringRad = 1
    elif(steeringRad < -1):
        steeringRad = -1
    #print "ADIFF", math.degrees(angleDiff), goalAngle, roomba.getAngle()
    #print steeringRad
    roomba.tankDrive(.3, steeringRad)

def replan():
  global pidIndex, timeStep, minDist, robotSize, gapTolerance, path, t2, t1
  print "REPLAN"
  roomba.stop()

  #determine turning direction
  goalAngle = math.atan2( (path[pidIndex+1][1] - roomba.getY()),(path[pidIndex+1][0] - roomba.getX())) % tau
  angleDiff = findAngleDiff(goalAngle, roomba.getAngle())
  direction = 0 #which direction robot should turn 1 is left, -1 is right
  if(abs(angleDiff) > .2): #significant enough difference; around 12 degrees
    print "ANGLE DIFF", goalAngle, roomba.getAngle(), roomba.getX(), roomba.getY()
    if(angleDiff > 0):
      direction = 1
    else:
      direction = -1
  elif (path[pidIndex+1][0] - roomba.getX() != 0): #check for slope that is not undefined
    slope = (path[pidIndex+1][1] - roomba.getY()) / (path[pidIndex+1][0] - roomba.getX())
    print "SLOPE"
    if(slope > 0):
      direction = 1
    else:
      direction = -1
  else:
    print "DEFAULT"
    direction = -1 #turns right by default
  print direction

  velocity = .3*direction#.3 is current turning speed, turns indefinitely

  #init vals
  currMin = minDist
  startRadius = -1
  foundOpening = False
  finalAngle = -1
  
  finalPoint = (-1, -1, -1)
  startRadius = minDist*10+ robotSize / 2 #convert distance to mm + robot radius
  startPoint = (roomba.getX(), roomba.getY(), roomba.getAngle())
  print roomba.getX(), roomba.getY()

  roomba.turn(velocity, 0) 
  
  while(finalAngle < 0): #while final angle doesn't exist
    time.sleep(timeStep)
    t2 = time.time() - t1
    t1 = time.time()
    updateVals(t2, False)
    if(not foundOpening): #add extra segment to threshold when checking for open space
      currMin = minDist + (startRadius/(math.cos(roomba.getAngle()-startPoint[2])) - startRadius)/10.0 #find extra, convert back to cm
      print "MIN VALS", currMin, (math.cos(roomba.getAngle()-startPoint[2]))
    if(roomba.getLidar() > currMin):
        if(not foundOpening):
          #from where first wall is to where opening is, draw a wall
          endWall = (roomba.getX(), roomba.getY(), roomba.getAngle())
          drawLines(linesDraw, startPoint, endWall)
          
          currMin = minDist
          foundOpening = True
          angle = roomba.getAngle()
          startPoint = (roomba.getX() + startRadius * math.cos(roomba.getAngle()), roomba.getY() + startRadius * math.sin(roomba.getAngle()), roomba.getAngle())
          print "START POINT:", startPoint, startRadius
          roomba.stop()
          time.sleep(.5)
          roomba.turn(velocity, 0)
        else:
          
          currPoint = (roomba.getX() + startRadius * math.cos(roomba.getAngle()), roomba.getY() + startRadius * math.sin(roomba.getAngle()), roomba.getAngle())
          '''
          roomba.stop()
          time.sleep(.1)
          roomba.turn(.3, 0)'''
          print getDist(startPoint[0], startPoint[1], currPoint[0], currPoint[1]), currPoint
          if(getDist(startPoint[0], startPoint[1], currPoint[0], currPoint[1]) > (robotSize + gapTolerance) ): #gap is large enough
            print "FOUND ANGLE"
            finalAngle = currPoint[2]
            finalPoint = currPoint
            roomba.stop()

    elif(foundOpening): #opening found, but then closed off again (gap too small)   
      print "CLOSED OFF", currMin, roomba.getLidar()
      currMin = minDist
      foundOpening = False
      finalAngle = -1
      startPoint = (roomba.getX(), roomba.getY(), roomba.getAngle())      
  
  #finally found open space
  roomba.stop()
  targetAngle = (finalAngle + startPoint[2]) / 2
  #targetAngle = finalAngle
  #print "TAR ANG", targetAngle, finalAngle, startPoint[2]
  targetPoint = (roomba.getX() + 1.1*startRadius * math.cos(targetAngle), roomba.getY() + 1.1*startRadius * math.sin(targetAngle)) #moves 1.5 lengths forward
  
  return targetPoint
  #path.insert(pidIndex + 1, targetPoint) #this is the new goal

  '''
  roomba.stop()
  while(True):
    time.sleep(5)'''
  

def updateVals(timeStep, updatePos):
    global start, kDrift, initAngle
    reading = arduino.getReadings() #reading = (arduino.readAngle(), 0)
    elapsed = time.time() - start
    angle = float(reading[0]) - initAngle + elapsed*kDrift
    radAngle = (math.radians(angle)) % tau #keeps angle within [0, 2pi]

    lidarDist = int(reading[1])
        
    roomba.update(t2, radAngle, lidarDist, updatePos)

#graphics  
window = turtle.Screen()
window.delay(0)

window.bgcolor('white')
roombaDraw = turtle.Turtle()
roombaDraw.shape('classic')
roombaDraw.color('green')
#roomba.resizemode('user')
#roombaDraw.shapesize(5, 5, .1)


linesDraw = turtle.Turtle()
linesDraw.shape('classic')
linesDraw.color('red')
linesDraw.resizemode('user')
linesDraw.shapesize(.3, .3, .3)

#roombaDraw.hideturtle()
roombaDraw.penup()
#linesDraw.hideturtle()

def drawRobot(roombaDraw, roomba):
  roombaDraw.goto(roomba.getX(), roomba.getY())
  roombaDraw.setHeading(roomba.getAngle())
  roombaDraw.stamp()

def drawLines(linesDraw, p1, p2):
  linesDraw.penup()
  linesDraw.goto(p1[0], p1[1])
  linesDraw.pendown()
  linesDraw.goto(p2[0], p2[1])
  

#variables
timeStep = .05
tau = 6.28318
minDist = 20 #dist (in cm) = reading -7; each unit is around a cm, but seems to be a min distance of like 5 cm (which is given as 12); so 
robotSize = 343
gapTolerance = 50

viaPoint = None

#Serial Ports
roombaSer = serial.Serial(port = 'COM3', baudrate = 115200, bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 1)
arduinoSer = serial.Serial(port = 'COM4', baudrate = 115200, bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 1)


start = (0,0)
goal = (1000, 0)
path =[start, goal]
pidIndex = 0
    
roomba = Roomba(0, 0, 0, roombaSer, False, False) #note: backwards doesn't work with position
arduino = Arduino(arduinoSer)
roomba.start()

t1= 0
t2 = 0
angle = 0.0


try:
  reading = arduino.getReadings()
  initAngle = float(reading[0]) + roomba.getAngle()
  initDist = reading[1] #get initial lidar reading
  roomba.updateLidar(initDist)

  start = time.time()
  t1 = start
  kDrift = -0.007    
except (KeyboardInterrupt, SystemExit):
    print "VALS", roomba.getX(), roomba.getY(), roomba.getAngle(), roomba.getLidar()
    roomba.end()
    arduino.end()
    raise Exception("STOPPED")

drawCounter = 1
drawMax = 10 #draws every 10 iterations
while (pidIndex < (len(path)-1)):
    try:
        time.sleep(timeStep) #in seconds
        t2 = time.time() - t1
        t1 = time.time()
        updateVals(t2, True)

        if(roomba.getLidar() < minDist):
          print "TOO CLOSE"
          viaPoint = replan()
        print "VALS", roomba.getX(), roomba.getY(), roomba.getAngle(), roomba.getLidar()
        PID(path, params, t2)

        if(drawCounter == drawMax):
          drawRobot(roombaDraw, roomba)
          drawCounter = 1 #counter starts at 1
        
        
    except (KeyboardInterrupt, SystemExit):
        print "VALS", roomba.getX(), roomba.getY(), roomba.getAngle(), roomba.getLidar()
        roomba.end()
        arduino.end()
        raise Exception("STOPPED")
        break

roomba.end()
arduino.end()
    
