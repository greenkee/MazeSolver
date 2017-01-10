class Node:
    def __init__(self, info, parent):
        self.infoList = info #info is (coord, accumulatedDistance, direction) to go around
        self.parentNode = parent

    def getCoord(self):
        return self.infoList[0]

    def getDirection(self):
        return self.infoList[2]

    def getTravelledDist(self):
      return self.infoList[1]
      '''
        if(self.parentNode != -1):
            return self.infoList[2] + self.parentNode.getCost()
        else:
            return 0
      '''
    def getParent(self):
        return self.parentNode #-1 means no parent
      
class PriorityQueue():
  def __init__(self):
    self.queue = []
    
  def push(self, node, cost):
    self.queue.append((cost, node))

  def sortAndPop(self): #gets lowest cost, removes from list and returns
    self.queue.sort()
    node = self.queue.pop(0)
    return node[1] #ignore cost, return only item

  def __len__(self):
      return (len(self.queue))

import shapely.geometry as sg
import math
def getDist(x1, y1, x2, y2):
  return math.sqrt( float((x1-x2)**2) + float((y1-y2)**2) )

def addToStack(nextNode, queue):
    endpoint = coord #should just be a tuple
    accumulatedDistance = getDist(currCoord[0], currCoord[1], endpoint[0], endpoint[1]) + currNode.getTravelledDist() #distance from current coord to wall endpoint + already covered distance from robot
    priority = accumulatedDistance + getDist(endpoint[0], endpoint[1], goal[0], goal[1]) #add heuristic to g-val

    nodeToAdd = Node((endpoint, accumulatedDistance, "direction"), currNode) #should only be accumulated value as currCost
    queue.push(nodeToAdd, priority)

def replanASTAR(goalCoord, walls): #walls is made up of LineStrings
  path = []
  openStack = PriorityQueue()
  listEmpty = False
  goal = goalCoord
  start = (0,0)#(roomba.getX(), roomba.getY())
  currNode = Node((start, 0, "?????"), -1) #-1 means no parent
  #draw line to goal
  #if intersection:
  while(listEmpty == False and currNode.getCoord() != goal):
    currCoord = currNode.getCoord()
    currLine = sg.LineString([currCoord, goal])

    print "ITER"
    print currNode.getCoord()
    print currLine.coords[0], currLine.coords[1]
    #draw line from currNode to goal

    intersections = PriorityQueue()
    for i in range(len(walls)): #find closest line
      if(not currLine.touches(walls[i]) and currLine.intersects(walls[i])): #if there's an intersection to a line
        intersect = currLine.intersection(walls[i])
        intersectCoord = (intersect.x, intersect.y)
        intersections.push(walls[i], getDist(intersect.x, intersect.y, currCoord[0], currCoord[1])) #the wall being intersected, and the distance to intersection point
    if(len(intersections) > 0):
        currWall = intersections.sortAndPop()
        for i in range(len(currWall.coords)):#should only happen twice
            if(i >= 2):
                print "WALL HAS TOO MANY POINTS ERROR"
            
            endpoint = currWall.coords[i] #should just be a tuple
            addRoundNode(endpoint, "direction???", currNode, currCoord, goal, openStack)#nextCoord, direction, currNode, currCoord, goal, queue
    else:
        endpoint = currLine.coords[1]
        addRoundNode(endpoint, "direction???", currNode, currCoord, goal, openStack)#nextCoord, direction, currNode, currCoord, goal, queue
    currNode = openStack.sortAndPop() #sorted by cost

    if(len(openStack) == 0):
      listEmpty = True
      
  
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

def roundPath(path):
    roundedPath = [path[0][0], path[len(path)-1][0]] #keep start and goal the same, but only coords
    for i in range(1, len(path)-1):
        pointsToAdd= addRoundPoint(roundedPath[i-1], path[i]) #returns list of points
        for j in range(len(pointsToAdd)):
            roundedPath.insert(len(path)-1, pointsToAdd[j]) #add right before the end
    return roundedPath

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
        

robotSize = 343
gapTolerance = 50
tau = 6.28318

def addRoundNode(nextCoord, direction, currNode, currCoord, goal, queue): #endpoint, currNode, currCoord, goal, openStack
  global robotSize, gapTolerance
  minRadius = robotSize + gapTolerance  #convert distance to mm + robot radius
  distance = getDist(currCoord[0], currCoord[1], nextCoord[0], nextCoord[1])

  #create circles; radius = buffer 
  c1 = sg.Point(currCoord[0], currCoord[1]).buffer(distance) #circle that tracks distance to wall
  c2 = sg.Point(nextCoord).buffer(minRadius) #circle to get enough area around obstacle
  intersection = c1.intersection(c2)

  endpoint = findBounds(intersection.exterior.coords, direction)#0 is below, 1 is right, 2 is above, 3 is left

  a1 = math.atan2(nextCoord[1]-currCoord[1], nextCoord[0]-currCoord[0]) % tau
  a2 = math.atan2(endpoint[1]-currCoord[1], endpoint[0]-currCoord[0]) % tau

  targetAngle = ( a1 + a2 )/2.0 #average angle of the 2
  targetPoint = (currCoord[0] + 1.1*minRadius * math.cos(targetAngle), currCoord[1] + 1.1*minRadius * math.sin(targetAngle)) #moves 1.5 lengths forward

  #adds endpoint, then targetpoint as node
  accumulatedDistance = getDist(currCoord[0], currCoord[1], endpoint[0], endpoint[1]) + currNode.getTravelledDist()
  node1 = Node(endpoint, accumulatedDistance, "direction", currNode)

  currCoord = node1.getCoord()
  accumulatedDistance = getDist(currCoord[0], currCoord[1], targetPoint[0], targetPoint[1]) + node1.getTravelledDist()
  priority = accumulatedDistance + getDist(targetPoint[0], targetPoint[1], goal[0], goal[1])
  nodeToAdd = Node((targetPoint, accumulatedDistance, "direction"), node1)
  queue.push(nodeToAdd, priority)
  ''' Perpendicular lines necesary?
  slope = (endPoint[1] - roomba.getY()) / (endPoint[0] - roomba.getX()) #y = m(x-x1) + y1 (roomba.getX(), roomba.getY())
  perpSlope = -1.0/slope #y = m(x-x1) + y1 (coord[0], coord[1])
  '''
  

walls = [sg.LineString([(500, 100), (500, -100)])]#, sg.LineString([(500,100), (700,100)])]
print replanASTAR((1000,0), walls)
