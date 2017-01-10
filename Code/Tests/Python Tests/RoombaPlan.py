import shapely.geometry as sg
from Utils import Node, PriorityQueue, Counter
import Utils, math


def replanASTAR(start, goalCoord, walls, robotSize, wallBuffer): #walls is made up of LineStrings
  path = []
  visitedPoints = Counter() #wall endpoints that have been visited
  openStack = PriorityQueue()
  listEmpty = False
  goal = goalCoord
  visitCoord = ((int)(start[0]), (int)(start[1]))
  currNode = Node((start, 0), -1) #-1 means no parent
  #draw line to goal
  #if intersection:

  #addedWalls = Counter()
  while(currNode.getCoord() != goal):
    if(visitedPoints[visitCoord] == 0 ):
        visitedPoints[visitCoord]+=1 #cast to ints to create loose buffer room
        print "VISIT", visitCoord, visitedPoints[currNode.getCoord()]
        currLine = sg.LineString([currNode.getCoord(), goal])
        print "ASTAR ITER"
        
        print currLine.coords[0], currLine.coords[1]
        #draw line from currNode to goal

        addedWalls = Counter() #create Counter to prevent infinite loop of adding

        if not(checkAndAddIntersection(currLine, currNode, walls, goal, openStack, addedWalls, robotSize, wallBuffer)):
            endpoint = currLine.coords[1]
            addNode(endpoint, currNode, goal, openStack) #coord, currNode, currCoord, goal, queue
        print "QUEUE"
        openStack.display()
    print "LEN:", len(openStack)
    if(len(openStack) == 0):
      #listEmpty = True
      break
    currNode = openStack.sortAndPop() #sorted by cost
    visitCoord = ((int)(currNode.getCoord()[0]), (int)(currNode.getCoord()[1]))
    
  
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

def addNode(coord, currNode, goal, queue):
    endpoint = coord #should just be a tuple
    currCoord = currNode.getCoord()
    accumulatedDistance = Utils.getDist(currCoord[0], currCoord[1], endpoint[0], endpoint[1]) + currNode.getTravelledDist() #distance from current coord to wall endpoint + already covered distance from robot
    priority = accumulatedDistance + Utils.getDist(endpoint[0], endpoint[1], goal[0], goal[1]) #add heuristic to g-val

    nodeToAdd = Node((endpoint, accumulatedDistance, "direction"), currNode) #should only be accumulated value as currCost
    queue.push(nodeToAdd, priority)

def handleIntersection(currWall, currNode, walls, goal, openStack, addedWalls, robotSize, wallBuffer):
    print "INTERSECT", currWall
    point1 = currWall.coords[0]
    point2 = currWall.coords[1]
    currCoord = currNode.getCoord()
    
    #determine if horizontal or vertical wall
    dX = (point1[0]-point2[0])
    dY = (point1[1]-point2[1])

    pointList = [point1, point2] #make array to iterate through twice
    
    for x in range(len(pointList)): #should only be 2 nodes
        pAngle = math.atan2(pointList[x][1]-currCoord[1], pointList[x][0] - currCoord[0]) % Utils.tau
        print "START:", currCoord, " END:", pointList[x], "ANGLE", pAngle
        if(((pAngle > Utils.tau/8.0) and (pAngle <= 3.0*Utils.tau/8.0)) or ((pAngle > 5.0*Utils.tau/8.0) and (pAngle <= 7.0*Utils.tau/8.0)) ): #upper/lower quadrant
            if(x==0): #first point
                if(dX < 0): #point 2 xVal greater than point 1's
                    addRoundNode(currWall, pointList[x], 3, currNode, walls, goal, openStack, addedWalls, robotSize, wallBuffer) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 xVal greater than point 2's
                    addRoundNode(currWall, pointList[x], 1, currNode, walls, goal, openStack, addedWalls, robotSize, wallBuffer)
            else: #second point
                if(dX < 0): #point 2 xVal greater than point 1's
                    addRoundNode(currWall, pointList[x], 1, currNode, walls, goal, openStack, addedWalls, robotSize, wallBuffer) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 xVal greater than point 2's
                    addRoundNode(currWall, pointList[x], 3, currNode, walls, goal, openStack, addedWalls, robotSize, wallBuffer)
        else: #means it's to the side of the robot
            if(x==0): #first point
                if(dY < 0): #point 2 yVal greater than point 1's
                    addRoundNode(currWall, pointList[x], 0, currNode, walls, goal, openStack, addedWalls, robotSize, wallBuffer) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 higher than point 2
                    addRoundNode(currWall, pointList[x], 2, currNode, walls, goal, openStack, addedWalls, robotSize, wallBuffer)
            else: #second point
                if(dY < 0): #point 2 yVal greater than point 1's
                    addRoundNode(currWall, pointList[x], 2, currNode, walls, goal, openStack, addedWalls, robotSize, wallBuffer) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 higher than point 2
                    addRoundNode(currWall, pointList[x], 0, currNode, walls, goal, openStack, addedWalls, robotSize, wallBuffer)

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

def checkAndAddIntersection(currLine, currNode, walls, goal, openStack, addedWalls, robotSize, wallBuffer): #whether point should be added if no intersection or not
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
            handleIntersection(currWall, currNode, walls, goal, openStack, addedWalls, robotSize, wallBuffer)
        return True
    else:
        return False


def getExtremeVals(coords, direction):
    minVal = 0
    maxVal = 0
    minPoint = coords[0]
    maxPoint = coords[0]
    if(direction == 0 or direction == 3): #need yVals
        minVal = coords[0][1]
        maxVal = coords[0][1]
        for x in range(1, len(coords)):
            currCoord = coords[x]
            if(currCoord[1] < minVal):
                minPoint = currCoord
                minVal = currCoord[1]
            elif(currCoord[1] > maxVal):
                maxPoint = currCoord
                maxVal = currCoord[1]
    else: #need xVals
        minVal = coords[0][0]
        maxVal = coords[0][0]
        for x in range(1, len(coords)):
            currCoord = coords[x]
            if(currCoord[0] < minVal):
                minPoint = currCoord
                minVal = currCoord[0]
            elif(currCoord[0] > maxVal):
                maxPoint = currCoord
                maxVal = currCoord[0]
    pointList = []
    if(direction == 1 or direction == 2):
        pointList = [maxPoint, minPoint] #try max first
    else:
        pointList = [minPoint, maxPoint]
    return pointList

def findBounds(currWall, currNode, coords, direction):
    currCoord = currNode.getCoord()
    intersections = getExtremeVals(coords, direction)
    for x in range(len(intersections)): #finds the point which does not intersect wall
        endLine = sg.LineString([currCoord, intersections[x]])
        if(not endLine.intersects(currWall)):
            return intersections[x]
    print "ERROR CIRCLE INTERSECT"
    return intersections[0] #default

def addRoundNode(currWall, nextCoord, direction, currNode, walls, goal, queue, addedWalls, robotSize, wallBuffer): #endpoint, currNode, walls, goal, openStack

  currCoord = currNode.getCoord()
  minRadius = robotSize + wallBuffer  #convert distance to mm + robot radius
  distance = Utils.getDist(currCoord[0], currCoord[1], nextCoord[0], nextCoord[1])

  #create circles; radius = buffer 
  c1 = sg.Point(currCoord[0], currCoord[1]).buffer(distance) #circle that tracks distance to wall
  c2 = sg.Point(nextCoord).buffer(minRadius) #circle to get enough area around obstacle
  intersection = c1.intersection(c2)

  print "DIR:", direction, currCoord
  #endpoint = point at which robot is far enough from wall (in given direction)
  endpoint = findBounds(currWall, currNode, intersection.exterior.coords, direction)#0 is below, 1 is right, 2 is above, 3 is left

  
  a1 = math.atan2(nextCoord[1]-currCoord[1], nextCoord[0]-currCoord[0]) % Utils.tau
  a2 = math.atan2(endpoint[1]-currCoord[1], endpoint[0]-currCoord[0]) % Utils.tau
  
  addAngle = Utils.findAngleDiff(a2, a1)/2.0 #angle between the 2; need to consider 0 = Utils.tau
  targetAngle = (a1 + addAngle) % Utils.tau
  print "ANGLES", a1, a2, addAngle, targetAngle

  #given avg angle of endPoint, currPoint; extend line - want robot to be in middle of gap, not end
  targetRadius = abs(distance/ math.cos(targetAngle)) #radii should always be positive value
  targetPoint = (currCoord[0] + targetRadius * math.cos(targetAngle), currCoord[1] + targetRadius * math.sin(targetAngle)) #moves 1.5 lengths forward

  print "POINTS:", currCoord, nextCoord, targetPoint, endpoint

  intersection = False

  #check if currPos to endpoint intersects walls
  currLine = sg.LineString([currCoord, endpoint])
  if( checkAndAddIntersection(currLine, currNode, walls, goal, queue, addedWalls, robotSize, wallBuffer)):#find closest line, add endpoints instead
      print "CURR -> EP INTERSECT"
      intersection = True

  #check if endpoint to targetpoint intersects walls
  currLine = sg.LineString([endpoint, targetPoint])
  if( checkAndAddIntersection(currLine, currNode, walls, goal, queue, addedWalls, robotSize, wallBuffer)):#find closest line
      print "EP -> TP INTERSECT"
      intersection = True

  if(not intersection):
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

#print replanASTAR((0,0), (100,0), [sg.LineString([(50, -50), (50, 50)])], 343, 50)
