import serial, time, math
from Arduino import Arduino
from Roomba import Roomba
from Utils import Node, PriorityQueue, Counter
import turtle
import Utils
import shapely.geometry as sg

def addNode(coord, currNode, goal, queue):
    endpoint = coord #should just be a tuple
    currCoord = currNode.getCoord()
    accumulatedDistance = Utils.getDist(currCoord[0], currCoord[1], endpoint[0], endpoint[1]) + currNode.getTravelledDist() #distance from current coord to wall endpoint + already covered distance from robot
    priority = accumulatedDistance + Utils.getDist(endpoint[0], endpoint[1], goal[0], goal[1]) #add heuristic to g-val

    nodeToAdd = Node((endpoint, accumulatedDistance, "direction"), currNode) #should only be accumulated value as currCost
    queue.push(nodeToAdd, priority)

def handleIntersection(currWall, currNode, walls, goal, openStack, addedWalls):
    print "INTERSECT", currWall
    point1 = currWall.coords[0]
    point2 = currWall.coords[1]
    currCoord = currNode.getCoord()
    
    #determine if horizontal or vertical wall
    dX = (point1[0]-point2[0])
    dY = (point1[1]-point2[1])

    pointList = [point1, point2] #make array to iterate through twice
    
    for x in range(len(pointList)): #should only be 2 nodes
        pAngle = math.atan2(pointList[x][1]-currCoord[1], pointList[x][0] - currCoord[0]) % tau
        print "START:", currCoord, " END:", pointList[x], pAngle
        if(((pAngle > tau/8.0) and (pAngle <= 3.0*tau/8.0)) or ((pAngle > 5.0*tau/8.0) and (pAngle <= 7.0*tau/8.0)) ): #upper/lower quadrant
            if(x==0): #first point
                if(dX < 0): #point 2 xVal greater than point 1's
                    addRoundNode(pointList[x], 3, currNode, walls, goal, openStack, addedWalls) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 xVal greater than point 2's
                    addRoundNode(pointList[x], 1, currNode, walls, goal, openStack, addedWalls)
            else: #second point
                if(dX < 0): #point 2 xVal greater than point 1's
                    addRoundNode(pointList[x], 1, currNode, walls, goal, openStack, addedWalls) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 xVal greater than point 2's
                    addRoundNode(pointList[x], 3, currNode, walls, goal, openStack, addedWalls)
        else: #means it's to the side of the robot
            if(x==0): #first point
                if(dY < 0): #point 2 yVal greater than point 1's
                    addRoundNode(pointList[x], 0, currNode, walls, goal, openStack, addedWalls) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 higher than point 2
                    addRoundNode(pointList[x], 2, currNode, walls, goal, openStack, addedWalls)
            else: #second point
                if(dY < 0): #point 2 yVal greater than point 1's
                    addRoundNode(pointList[x], 2, currNode, walls, goal, openStack, addedWalls) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 higher than point 2
                    addRoundNode(pointList[x], 0, currNode, walls, goal, openStack, addedWalls)

        '''
        if(abs(dX) > abs(dY)): #horizontal wall
            if(dX < 0): #point 2 xVal greater than point 1's
                addRoundNode(point1, 3, currNode, currCoord, goal, openStack) #0 is below, 1 is right, 2 is above, 3 is left
                addRoundNode(point2, 1, currNode, currCoord, goal, openStack)
            else: #point 1 xVal greater than point 2's
                addRoundNode(point1, 1, currNode, currCoord, goal, openStack) 
                addRoundNode(point2, 3, currNode, currCoord, goal, openStack)
        else: #vertical wall
            if(dY < 0): #point 2 yVal greater than point 1's
                addRoundNode(point1, 0, currNode, currCoord, goal, openStack) 
                addRoundNode(point2, 2, currNode, currCoord, goal, openStack)
            else: #point 1 higher than point 2
                addRoundNode(point1, 2, currNode, currCoord, goal, openStack) 
                addRoundNode(point2, 0, currNode, currCoord, goal, openStack)
                '''

def checkIntersection(currLine, currNode, walls, goal, openStack):
    intersections = PriorityQueue()
    currCoord = currNode.getCoord()
    for i in range(len(walls)): #find closest line
      if(currLine.intersects(walls[i])): #if there's an intersection to a line
        return True
    return False

def checkAndAddIntersection(currLine, currNode, walls, goal, openStack, addedWalls): #whether point should be added if no intersection or not
    intersections = PriorityQueue()
    currCoord = currNode.getCoord()
    for i in range(len(walls)): #find closest line
      if(currLine.intersects(walls[i])): #if there's an intersection to a line
        intersect = currLine.intersection(walls[i])
        intersectCoord = (intersect.x, intersect.y)
        intersections.push(walls[i], Utils.getDist(intersect.x, intersect.y, currCoord[0], currCoord[1])) #the wall being intersected, and the distance to intersection point
    if(len(intersections) > 0):
        print "INTERSECTIONS"
        intersections.display()
        currWall = intersections.sortAndPop()
        if(addedWalls[currWall] == 0):
            addedWalls[currWall] += 1
            handleIntersection(currWall, currNode, walls, goal, openStack, addedWalls)
        return True
    else:
        return False

def printWalls(walls):
  for x in range(len(walls)):
    print "WALL",walls[x].coords[0], walls[x].coords[1]

def replanASTAR(goalCoord, walls): #walls is made up of LineStrings
  path = []
  visitedPoints = Counter() #wall endpoints that have been visited
  openStack = PriorityQueue()
  listEmpty = False
  goal = goalCoord
  start = (roomba.getX(), roomba.getY())
  currNode = Node((start, 0), -1) #-1 means no parent
  #draw line to goal
  #if intersection:

  #addedWalls = Counter()
  while(listEmpty == False and currNode.getCoord() != goal):
    if(visitedPoints[currNode.getCoord()] == 0 ):
        #visitedPoints[currNode.getCoord()]+=1 #disabled for now
        currLine = sg.LineString([currNode.getCoord(), goal])
        print "ASTAR ITER"
        printWalls(walls)
        
        print currLine.coords[0], currLine.coords[1]
        #draw line from currNode to goal

        addedWalls = Counter() #create Counter to prevent infinite loop of adding

        if not(checkAndAddIntersection(currLine, currNode, walls, goal, openStack, addedWalls)):
            endpoint = currLine.coords[1]
            addNode(endpoint, currNode, goal, openStack) #coord, currNode, currCoord, goal, queue

    if(len(openStack) == 0):
      listEmpty = True
    currNode = openStack.sortAndPop() #sorted by cost
  
  if(currNode.getCoord() == goal):
    while(currNode.getCoord() != start):
          path.append(currNode.getCoord())
          currNode = currNode.getParent()
    path.append(currNode.getCoord()) #finally, add the start node, but don't try to get parent
    path.reverse()
  else:
    print "NO PATH"
  return path
  #return roundPath(path)


def findBounds(coords, direction): #0: lowest Y, 1: highest X, 2: highest Y, 3: lowest X
    extremeVal = 0
    bestPoint = coords[0]
    if(direction == 0):
        extremeVal = coords[0][1]
        for x in range(1, len(coords)): #everything but the first
            currCoord = coords[x]
            if(currCoord[1] < extremeVal):
                extremeVal = currCoord[1]
                bestPoint = currCoord
    elif(direction == 1):
        extremeVal = coords[0][0]
        for x in range(1, len(coords)): #everything but the first
            currCoord = coords[x]
            if(currCoord[0] > extremeVal):
                extremeVal = currCoord[0]
                bestPoint = currCoord
    elif (direction == 2):
        extremeVal = coords[0][1]
        for x in range(1, len(coords)): #everything but the first
            currCoord = coords[x]
            if(currCoord[1] > extremeVal):
                extremeVal = currCoord[1]
                bestPoint = currCoord
    elif (direction == 3):
        extremeVal = coords[0][0]
        for x in range(1, len(coords)): #everything but the first
            currCoord = coords[x]
            if(currCoord[0] < extremeVal):
                extremeVal = currCoord[0]
                bestPoint = currCoord
    
    return bestPoint     

def addRoundNode(nextCoord, direction, currNode, walls, goal, queue, addedWalls): #endpoint, currNode, walls, goal, openStack
  global robotSize, gapTolerance

  currCoord = currNode.getCoord()
  minRadius = robotSize + gapTolerance  #convert distance to mm + robot radius
  distance = Utils.getDist(currCoord[0], currCoord[1], nextCoord[0], nextCoord[1])

  #create circles; radius = buffer 
  c1 = sg.Point(currCoord[0], currCoord[1]).buffer(distance) #circle that tracks distance to wall
  c2 = sg.Point(nextCoord).buffer(minRadius) #circle to get enough area around obstacle
  intersection = c1.intersection(c2)

  print "DIR:", direction, currCoord
  #endpoint = point at which robot is far enough from wall (in given direction)
  endpoint = findBounds(intersection.exterior.coords, direction)#0 is below, 1 is right, 2 is above, 3 is left

  
  a1 = math.atan2(nextCoord[1]-currCoord[1], nextCoord[0]-currCoord[0]) % tau
  a2 = math.atan2(endpoint[1]-currCoord[1], endpoint[0]-currCoord[0]) % tau
  
  addAngle = Utils.findAngleDiff(a2, a1)/2.0 #angle between the 2; need to consider 0 = tau
  targetAngle = (a1 + addAngle) % tau
  print "ANGLES", a1, a2, addAngle, targetAngle

  #given avg angle of endPoint, currPoint; extend line - want robot to be in middle of gap, not end
  targetRadius = abs(distance/ math.cos(targetAngle)) #radii should always be positive value
  targetPoint = (currCoord[0] + targetRadius * math.cos(targetAngle), currCoord[1] + targetRadius * math.sin(targetAngle)) #moves 1.5 lengths forward

  print "POINTS:", currCoord, nextCoord, targetPoint, endpoint

  #check if currPos to endpoint intersects walls
  currLine = sg.LineString([currCoord, endpoint])
  if( checkAndAddIntersection(currLine, currNode, walls, goal, queue, addedWalls)):#find closest line, add endpoints instead
      print "CURR -> EP INTERSECT"
      return

  #check if endpoint to targetpoint intersects walls
  currLine = sg.LineString([endpoint, targetPoint])
  if( checkAndAddIntersection(currLine, currNode, walls, goal, queue, addedWalls)):#find closest line
      print "EP -> TP INTERSECT"
      return

  #adds endpoint, then targetpoint as node; goes from curr to endpoint to target
  accumulatedDistance = Utils.getDist(currCoord[0], currCoord[1], endpoint[0], endpoint[1]) + currNode.getTravelledDist()
  node1 = Node((endpoint, accumulatedDistance), currNode)

  currCoord = node1.getCoord()
  accumulatedDistance = Utils.getDist(currCoord[0], currCoord[1], targetPoint[0], targetPoint[1]) + node1.getTravelledDist()
  priority = accumulatedDistance + Utils.getDist(targetPoint[0], targetPoint[1], goal[0], goal[1])
  nodeToAdd = Node((targetPoint, accumulatedDistance), node1)
  print "ROUNDER", endpoint, targetPoint
  queue.push(nodeToAdd, priority) #adds endpoint as node
  ''' Perpendicular lines necesary?
  slope = (endPoint[1] - roomba.getY()) / (endPoint[0] - roomba.getX()) #y = m(x-x1) + y1 (roomba.getX(), roomba.getY())
  perpSlope = -1.0/slope #y = m(x-x1) + y1 (coord[0], coord[1])
  '''


#PID Vars
tau_p = 3.0
tau_i = 0.0
tau_d = 0.0

accum_error = 0
diff_error = 0
cte = 0
prev_cte = 0
params = [tau_p, tau_i, tau_d]



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
    goalAngle = math.atan2( (nextPoint[1] - roomba.getY()),(nextPoint[0] - roomba.getX())) % tau
    angleDiff = Utils.findAngleDiff(goalAngle, roomba.getAngle())
    
    #cte = (float( yDist * xLength - xDist*yLength) / float(xLength*xLength + yLength*yLength)) -  angleDiff 
    cte = -angleDiff
    if(Utils.reachedPoint(roomba.getX(), roomba.getY(), nextPoint[0], nextPoint[1])):
        pidIndex  += 1#+= 1 
        print "REACHED POINT", nextPoint[0], nextPoint[1]

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
  global pidIndex, timeStep, minDist, robotSize, gapTolerance, path, t2, t1
  print "REPLAN"
  roomba.stop()

  #move roomba backwards slightly?
  roomba.tankDrive(-.3, 0)
  t1 = time.time()
  time.sleep(.3)
  t2 = time.time() - t1
  t1 = time.time()
  updateVals(t2, False)
  roomba.stop()
    
  #determine turning direction
  goalAngle = math.atan2( (path[pidIndex+1][1] - roomba.getY()),(path[pidIndex+1][0] - roomba.getX())) % tau
  angleDiff = Utils.findAngleDiff(goalAngle, roomba.getAngle())
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
  startPoint = startPoint = (roomba.getX() + startRadius * math.cos(roomba.getAngle()), roomba.getY() + startRadius * math.sin(roomba.getAngle()), roomba.getAngle())
  print roomba.getX(), roomba.getY()

  roomba.turn(velocity, 0) 
  
  while(finalAngle < 0): #while final angle doesn't exist
    time.sleep(timeStep)
    t2 = time.time() - t1
    t1 = time.time()
    updateVals(t2, False)
    if(not foundOpening): #add extra segment to threshold when checking for open space
      currMin = minDist + (abs(startRadius/(math.cos(roomba.getAngle()-startPoint[2]))) - startRadius)/10.0 #find extra, convert back to cm
      print "MIN VALS", currMin, (math.cos(roomba.getAngle()-startPoint[2]))
    if(roomba.getLidar() > currMin):
        if(not foundOpening):
          #from where first wall is to where opening is, draw a wall
          endRadius = startRadius#abs(startRadius/(math.cos(roomba.getAngle()-startPoint[2])))
          endWall = (roomba.getX() + (endRadius) * math.cos(roomba.getAngle()), roomba.getY() + (endRadius) * math.sin(roomba.getAngle()), roomba.getAngle())
          print "ADD WALL- START:", startPoint, " END:", endWall
          #drawLines(lines, startPoint, endWall)
          walls.append( sg.LineString([(startPoint[0], startPoint[1]), (endWall[0], endWall[1])])  ) #add start and end coord to walls
          

          
          currMin = minDist
          foundOpening = True
          angle = roomba.getAngle()
          startPoint = endWall #new start point
          print "START POINT:", startPoint, startRadius


          roomba.stop()
          print "START", roomba.getLidar()
          time.sleep(1.5)
          roomba.turn(velocity, 0)
        else:
          
          currPoint = (roomba.getX() + startRadius * math.cos(roomba.getAngle()), roomba.getY() + startRadius * math.sin(roomba.getAngle()), roomba.getAngle())
          '''
          roomba.stop()
          time.sleep(.1)
          roomba.turn(.3, 0)'''
          print Utils.getDist(startPoint[0], startPoint[1], currPoint[0], currPoint[1]), currPoint
          if(Utils.getDist(startPoint[0], startPoint[1], currPoint[0], currPoint[1]) > (robotSize + gapTolerance) ): #gap is large enough
            print "FOUND ANGLE"
            finalAngle = currPoint[2]
            finalPoint = currPoint
            roomba.stop()

    elif(foundOpening): #opening found, but then closed off again (gap too small)   
      print "CLOSED OFF", currMin, roomba.getLidar()
      roomba.stop()
      time.sleep(1.5)
      roomba.turn(velocity, 0)
      currMin = minDist
      foundOpening = False
      finalAngle = -1
      startPoint = startPoint = (roomba.getX() + startRadius * math.cos(roomba.getAngle()), roomba.getY() + startRadius * math.sin(roomba.getAngle()), roomba.getAngle())    
  
  #finally found open space
  roomba.stop()
  return replanASTAR(path[len(path)-1], walls) #goal is last entry in path, walls

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

scalingFactor = .2
#graphics  
window = turtle.Screen()
window.delay(0)
window.bgcolor('white')

roombaDraw = turtle.Turtle()
roombaDraw.shape('classic')
roombaDraw.color('green')
#roombaDraw.resizemode('user')
#roombaDraw.shapesize(.3, .3, .1)


linesDraw = turtle.Turtle()
linesDraw.shape('classic')
linesDraw.color('red')
linesDraw.pensize(5)
#linesDraw.resizemode('user')
#linesDraw.shapesize(.3, .3, .3)

#roombaDraw.hideturtle()
roombaDraw.penup()
linesDraw.hideturtle()

pathDraw = turtle.Turtle()
pathDraw.shape('classic')
pathDraw.color('blue')
pathDraw.pensize(5)
pathDraw.hideturtle()

def drawPath(pen, path):
  pen.clear()
  for x in range(len(path)-1):#everything but last
    drawLines(pen, path[x], path[x+1])

def drawRobot(pen, roomba):
  global scalingFactor
  pen.goto(roomba.getX()*scalingFactor, roomba.getY()*scalingFactor)
  pen.setheading(math.degrees(roomba.getAngle()))
  pen.stamp()

def drawLines(pen, p1, p2):
  global scalingFactor
  pen.penup()
  pen.goto(p1[0]*scalingFactor, p1[1]*scalingFactor)
  pen.pendown()
  pen.goto(p2[0]*scalingFactor, p2[1]*scalingFactor)

def endProgram():
  printWalls(walls)
  roomba.end()
  arduino.end()
  turtle.done()  

#variables
timeStep = .05
tau = 6.28318
minDist = 30 #dist (in cm) = reading -7; each unit is around a cm, but seems to be a min distance of like 5 cm (which is given as 12); so 
robotSize = 343
gapTolerance = 50

viaPoint = None

#Serial Ports

#for PC
roombaSerName = 'COM3'
arduinoSerName = 'COM4'

#for Linux
#roombaSerName = '/dev/ttyUSB0'
#arduinoSerName = '/dev/ttyACM0' 
roombaSer = serial.Serial(port = roombaSerName, baudrate = 115200, bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 1)
arduinoSer = serial.Serial(port = arduinoSerName, baudrate = 115200, bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 1)


start = (0,0)
goal = (1000, 0)
path = [start, goal]
walls = []
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

#drawCounter = 1
#drawMax = 5 #draws every [drawMax] iterations
while (pidIndex < (len(path)-1)):
    try:
        time.sleep(timeStep) #in seconds
        t2 = time.time() - t1
        t1 = time.time()
        updateVals(t2, True)

        print "VALS", roomba.getX(), roomba.getY(), roomba.getAngle(), roomba.getLidar()
        
        if( (getDist(roomba.getX(), roomba.getY(), path[pidIndex +1][0], path[pidIndex+1][1]) > minDist*10) and (roomba.getLidar() < minDist)): #allows it to reach goal, even by wall; converts minDist to mm
          print "TOO CLOSE"
          path = replan(walls)
          print "NEW PATH", path
          #drawPath(path, path)
          pidIndex = 0 #reset path to be where roomba is
        PID(path, params, t2)

        '''
        if(drawCounter == drawMax):
          drawRobot(roombaDraw, roomba)
          drawCounter = 1 #counter starts at 1
        else:
          drawCounter += 1
        '''
        
    except (KeyboardInterrupt, SystemExit):
        print "VALS", roomba.getX(), roomba.getY(), roomba.getAngle(), roomba.getLidar()
        endProgram()
        raise Exception("STOPPED")
        break
endProgram()
    
