class Node:
    def __init__(self, info, parent):
        self.infoList = info #info is (coord, accumulatedDistance) to go around
        self.parentNode = parent

    def getCoord(self):
        return self.infoList[0]

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

  def display(self):
      for x in range(len(self.queue)):
          print self.queue[x][0], self.queue[x][1]

import shapely.geometry as sg
import math, turtle
def getDist(x1, y1, x2, y2):
  return math.sqrt( float((x1-x2)**2) + float((y1-y2)**2) )

def addNode(coord, currNode, goal, queue):
    endpoint = coord #should just be a tuple
    currCoord = currNode.getCoord()
    accumulatedDistance = getDist(currCoord[0], currCoord[1], endpoint[0], endpoint[1]) + currNode.getTravelledDist() #distance from current coord to wall endpoint + already covered distance from robot
    priority = accumulatedDistance + getDist(endpoint[0], endpoint[1], goal[0], goal[1]) #add heuristic to g-val

    nodeToAdd = Node((endpoint, accumulatedDistance, "direction"), currNode) #should only be accumulated value as currCost
    queue.push(nodeToAdd, priority)

def handleIntersection(currWall, currNode, walls, goal, openStack):
    print "WALL", currWall
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
                    addRoundNode(pointList[x], 3, currNode, walls, goal, openStack) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 xVal greater than point 2's
                    addRoundNode(pointList[x], 1, currNode, walls, goal, openStack)
            else: #second point
                if(dX < 0): #point 2 xVal greater than point 1's
                    addRoundNode(pointList[x], 1, currNode, walls, goal, openStack) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 xVal greater than point 2's
                    addRoundNode(pointList[x], 3, currNode, walls, goal, openStack)
        else: #means it's to the side of the robot
            if(x==0): #first point
                if(dY < 0): #point 2 yVal greater than point 1's
                    addRoundNode(pointList[x], 0, currNode, walls, goal, openStack) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 higher than point 2
                    addRoundNode(pointList[x], 2, currNode, walls, goal, openStack)
            else: #second point
                if(dY < 0): #point 2 yVal greater than point 1's
                    addRoundNode(pointList[x], 2, currNode, walls, goal, openStack) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 higher than point 2
                    addRoundNode(pointList[x], 0, currNode, walls, goal, openStack)

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


def checkAndAddIntersection(currLine, currNode, walls, goal, openStack): #whether point should be added if no intersection or not
    intersections = PriorityQueue()
    currCoord = currNode.getCoord()
    for i in range(len(walls)): #find closest line
      if(currLine.intersects(walls[i])): #if there's an intersection to a line
        intersect = currLine.intersection(walls[i])
        intersectCoord = (intersect.x, intersect.y)
        intersections.push(walls[i], getDist(intersect.x, intersect.y, currCoord[0], currCoord[1])) #the wall being intersected, and the distance to intersection point
    if(len(intersections) > 0):
        intersections.display()
        currWall = intersections.sortAndPop()
        handleIntersection(currWall, currNode, walls, goal, openStack)
        return True
    else:
        return False


def replanASTAR(goalCoord, walls): #walls is made up of LineStrings
  path = []
  openStack = PriorityQueue()
  listEmpty = False
  goal = goalCoord
  start = (0 ,0)#(roomba.getX(), roomba.getY())
  currNode = Node((start, 0), -1) #-1 means no parent
  #draw line to goal
  #if intersection:
  while(listEmpty == False and currNode.getCoord() != goal):
    currLine = sg.LineString([currNode.getCoord(), goal])
    print "ASTAR ITER"
    
    print currLine.coords[0], currLine.coords[1]
    #draw line from currNode to goal

    if not(checkAndAddIntersection(currLine, currNode, walls, goal, openStack)):
        endpoint = currLine.coords[1]
        addNode(endpoint, currNode, goal, openStack) #coord, currNode, currCoord, goal, queue



    if(len(openStack) == 0):
      listEmpty = True


    print "NODES"
    openStack.display()
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
        

robotSize = 343
gapTolerance = 50
tau = 6.28318

def addRoundNode(nextCoord, direction, currNode, walls, goal, queue): #endpoint, currNode, walls, goal, openStack
  global robotSize, gapTolerance

  currCoord = currNode.getCoord()
  minRadius = robotSize + gapTolerance  #convert distance to mm + robot radius
  distance = getDist(currCoord[0], currCoord[1], nextCoord[0], nextCoord[1])

  #create circles; radius = buffer 
  c1 = sg.Point(currCoord[0], currCoord[1]).buffer(distance) #circle that tracks distance to wall
  c2 = sg.Point(nextCoord).buffer(minRadius) #circle to get enough area around obstacle
  intersection = c1.intersection(c2)

  print "DIR:", direction
  endpoint = findBounds(intersection.exterior.coords, direction)#0 is below, 1 is right, 2 is above, 3 is left

  

  a1 = math.atan2(nextCoord[1]-currCoord[1], nextCoord[0]-currCoord[0]) % tau
  a2 = math.atan2(endpoint[1]-currCoord[1], endpoint[0]-currCoord[0]) % tau
  
  targetAngle = ( a1 + a2 )/2.0 #average angle of the 2
  print "ANGLES", a1, a2, targetAngle

  targetRadius = abs(distance/ math.cos(targetAngle)) #radii should always be full value
  targetPoint = (currCoord[0] + targetRadius * math.cos(targetAngle), currCoord[1] + targetRadius * math.sin(targetAngle)) #moves 1.5 lengths forward

  print "POINTS:", currCoord, nextCoord, targetPoint, endpoint

  #check if endpoint to targetpoint intersects walls
  currLine = sg.LineString([endpoint, targetPoint])
  if( checkAndAddIntersection(currLine, currNode, walls, goal, queue)):#find closest line
      return

  #adds endpoint, then targetpoint as node
  accumulatedDistance = getDist(currCoord[0], currCoord[1], endpoint[0], endpoint[1]) + currNode.getTravelledDist()
  node1 = Node((endpoint, accumulatedDistance), currNode)

  currCoord = node1.getCoord()
  accumulatedDistance = getDist(currCoord[0], currCoord[1], targetPoint[0], targetPoint[1]) + node1.getTravelledDist()
  priority = accumulatedDistance + getDist(targetPoint[0], targetPoint[1], goal[0], goal[1])
  nodeToAdd = Node((targetPoint, accumulatedDistance), node1)
  print "ROUNDER", endpoint, targetPoint
  queue.push(nodeToAdd, priority)
  ''' Perpendicular lines necesary?
  slope = (endPoint[1] - roomba.getY()) / (endPoint[0] - roomba.getX()) #y = m(x-x1) + y1 (roomba.getX(), roomba.getY())
  perpSlope = -1.0/slope #y = m(x-x1) + y1 (coord[0], coord[1])
  '''

window = turtle.Screen()
window.delay(0)
window.screensize(5000,5000)

linesDraw = turtle.Turtle()
linesDraw.shape('classic')
linesDraw.color('red')
linesDraw.pensize(5)
#linesDraw.resizemode('user')
#linesDraw.shapesize(.3, .3, .3)

#roombaDraw.hideturtle()
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
  
def drawLines(pen, p1, p2):
  pen.penup()
  pen.goto(p1[0], p1[1])
  pen.pendown()
  pen.goto(p2[0], p2[1])

def drawWalls(pen, lineStrings):
    for x in range(len(lineStrings)):
        p1 = lineStrings[x].coords[0]
        p2 = lineStrings[x].coords[1]
        drawLines(pen, p1, p2)

walls = [sg.LineString([(780.4262566887401,629.0517128), (330.2847168,-101.7195916345329)]),sg.LineString([(685.0876214110815,135.8979073), (-28.43541798,-394.2486279738732)])]
'''
sg.LineString([(708.7351778870609,-18.24999437), (706.7599488,41.91598961059081)]),
sg.LineString([(704.8365575090745,58.62301127), (702.4908525,74.22551466875379)]),
sg.LineString([(698.0368389192163,97.27350091), (686.1786567,141.1308045987683)]),
sg.LineString([(680.0496961242437,158.8930441), (673.0173235,176.8791711777591)]),
sg.LineString([(685.0876214110815,135.8979073), (-28.43541798,-394.2486279738732)]),
sg.LineString([(780.4262566887401,629.0517128), (330.2847168,-101.7195916345329)]),
sg.LineString([(1085.311432059344,722.8619484), (640.8703697,651.9597410354402)]),
sg.LineString([(742.5861337308215,627.537597), (1209.268585,869.4390239968177)]),
sg.LineString([(1080.959009404016,726.0327809), (604.9716242,665.0819817017612)]),
sg.LineString([(687.7847013785577,636.1954938), (1179.619441,835.3338721959278)]),
sg.LineString([(1056.577011964233,715.3591485), (1028.885037,697.1786976440148)]),
sg.LineString([(1022.856978263482,693.5790443), (564.6327956,683.1681475665322)]),
sg.LineString([(631.1861278751824,654.2453219), (1157.030016,814.5823350345458)]),
sg.LineString([(1003.164164279697,685.7151344), (543.6954055,695.2197413451252)]),
sg.LineString([(620.3251172338908,656.8962844), (1127.458959,783.5768485101289)]),
sg.LineString([(916.8427219460175,648.1788208), (529.9102173,700.7611353557267)]),
sg.LineString([(686.5381565609598,637.1761009), (1126.243612,780.5596100300291)]),
sg.LineString([(881.2383551881919,636.1489087), (483.0977024,731.7907033503338)]),
sg.LineString([(645.3800130002786,643.6121925), (1102.21452,754.9513619664385)]),
sg.LineString([(878.825149942268,631.1711581), (466.6572507,742.1640451097021)]),
]'''
   #, sg.LineString([(500,100), (700,100)])]
drawWalls(linesDraw, walls)
path =  replanASTAR((1000,0), walls)
print path
drawPath(pathDraw, path)
turtle.done()
