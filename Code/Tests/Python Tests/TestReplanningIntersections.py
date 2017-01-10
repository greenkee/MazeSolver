import shapely.geometry as sg
import math, turtle, Utils
from Utils import Node, PriorityQueue


def addNode(coord, currNode, goal, queue):
    endpoint = coord #should just be a tuple
    currCoord = currNode.getCoord()
    accumulatedDistance = Utils.getDist(currCoord[0], currCoord[1], endpoint[0], endpoint[1]) + currNode.getTravelledDist() #distance from current coord to wall endpoint + already covered distance from robot
    priority = accumulatedDistance + Utils.getDist(endpoint[0], endpoint[1], goal[0], goal[1]) #add heuristic to g-val

    nodeToAdd = Node((endpoint, accumulatedDistance, "direction"), currNode) #should only be accumulated value as currCost
    queue.push(nodeToAdd, priority)

def handleIntersection(currWall, currNode, walls, goal, openStack):
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
def checkIntersection(currLine, currNode, walls, goal, openStack):
    intersections = PriorityQueue()
    currCoord = currNode.getCoord()
    for i in range(len(walls)): #find closest line
      if(currLine.intersects(walls[i])): #if there's an intersection to a line
        return True
    return False

def checkAndAddIntersection(currLine, currNode, walls, goal, openStack): #whether point should be added if no intersection or not
    intersections = PriorityQueue()
    currCoord = currNode.getCoord()
    for i in range(len(walls)): #find closest line
      if(currLine.intersects(walls[i])): #if there's an intersection to a line
        intersect = currLine.intersection(walls[i])
        intersectCoord = (intersect.x, intersect.y)
        intersections.push(walls[i], Utils.getDist(intersect.x, intersect.y, currCoord[0], currCoord[1])) #the wall being intersected, and the distance to intersection point
    if(len(intersections) > 0):
        print "WALLS"
        intersections.display()
        currWall = intersections.sortAndPop()
        handleIntersection(currWall, currNode, walls, goal, openStack)
        return True
    else:
        return False


def replanASTAR(goalCoord, walls): #walls is made up of LineStrings
  path = []
  #visitedPoints = () #wall endpoints that have been visited
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
  if( checkIntersection(currLine, currNode, walls, goal, queue)):#find closest line
      print "CURR -> EP INTERSECT"
      #return

  #check if endpoint to targetpoint intersects walls
  currLine = sg.LineString([endpoint, targetPoint])
  if( checkIntersection(currLine, currNode, walls, goal, queue)):#find closest line
      print "EP -> TP INTERSECT"
      #return

  #adds endpoint, then targetpoint as node; goes from curr to endpoint to target
  accumulatedDistance = Utils.getDist(currCoord[0], currCoord[1], endpoint[0], endpoint[1]) + currNode.getTravelledDist()
  node1 = Node((endpoint, accumulatedDistance), currNode)

  currCoord = node1.getCoord()
  accumulatedDistance = Utils.getDist(currCoord[0], currCoord[1], targetPoint[0], targetPoint[1]) + node1.getTravelledDist()
  priority = accumulatedDistance + Utils.getDist(targetPoint[0], targetPoint[1], goal[0], goal[1])
  nodeToAdd = Node((targetPoint, accumulatedDistance), node1)
  print "ROUNDER", endpoint, targetPoint
  queue.push(nodeToAdd, priority)
  ''' Perpendicular lines necesary?
  slope = (endPoint[1] - roomba.getY()) / (endPoint[0] - roomba.getX()) #y = m(x-x1) + y1 (roomba.getX(), roomba.getY())
  perpSlope = -1.0/slope #y = m(x-x1) + y1 (coord[0], coord[1])
  '''

scalingFactor = .2
window = turtle.Screen()
window.delay(0)
#window.screensize(1000,1000)

linesDraw = turtle.Turtle()
linesDraw.shape('classic')
linesDraw.color('red')
linesDraw.pensize(4)
#linesDraw.resizemode('user')
#linesDraw.shapesize(.3, .3, .3)

linesDraw.hideturtle()

pathDraw = turtle.Turtle()
pathDraw.shape('classic')
pathDraw.color('blue')
pathDraw.pensize(4)
pathDraw.hideturtle()
#pathDraw.resizemode('user')
#pathDraw.shapesize(.3, .3, .3)

def drawPath(pen, path):
  pen.clear()
  for x in range(len(path)-1):#everything but last
    drawLines(pen, path[x], path[x+1])
  
def drawLines(pen, p1, p2):
  global scalingFactor
  pen.penup()
  pen.goto(p1[0]*scalingFactor, p1[1]*scalingFactor)
  pen.pendown()
  pen.goto(p2[0]*scalingFactor, p2[1]*scalingFactor)

def drawWalls(pen, lineStrings):
    for x in range(len(lineStrings)):
        p1 = lineStrings[x].coords[0]
        p2 = lineStrings[x].coords[1]
        drawLines(pen, p1, p2)

walls = [sg.LineString([(718.3932229866025,1.964469371047762),(713.5044337877139,-62.92895455347919)]),
sg.LineString([(710.9930128674491,-81.08479239518606),(429.7735580467337,438.21060975661163)]),
sg.LineString([(-216.59976146693091,-69.13424900939314),(-216.46189740568016,-70.01886965924373)]),
sg.LineString([(647.0589240156685,-561.5719148697983),(-97.32347027195976,-204.5469208265156)]),
sg.LineString([(187.43248187443498,-609.7111911919772),(620.1638524499243,224.08088711861433)]),
sg.LineString([(821.9291437233038,671.299508335862),(240.6157854464517,41.47026925795831)]),
sg.LineString([(789.1028643517297,246.50488890496257),(715.9485851933186,825.7106135863562)]),
sg.LineString([(819.7858929213944,673.9484361059096),(195.58921998980227,65.58314520932106)]), 
]
'''




sg.LineString([(803.0337673884239,644.2137995226702),(227.3237534916878,52.66983197568214)]),
sg.LineString([(813.5759779875109,618.3711698398201),(155.3030884038911,92.80882456682338)]),
sg.LineString([(646.9510423922327,152.71825247755788),(616.0606416521223,905.0496581926191)])
]

sg.LineString([(707.6174372224785,-22.54718613749703),(488.0139443550521,392.49620346516565)]),
sg.LineString([(694.8381335816646,91.89024079430625),(-15.483408113267785,-403.9452557037477)]),
sg.LineString([(635.0819697226011,566.3351818459878),(636.3007880302335,565.4795045830781)]),
sg.LineString([(727.8338825067642,699.7395650305227),(726.4344299347646,700.7336347601911)]),
sg.LineString([(954.638662480476,651.0311127290474),(474.0951429550849,-27.46962277953162)]),
sg.LineString([(211.88035108972846,-631.3027055299976),(590.1561914116027,217.66685949016068)]),
sg.LineString([(813.5948235035623,596.5173896232835),(723.2547955059898,777.7171595407701)]),
sg.LineString([(803.0337673884239,644.2137995226702),(227.3237534916878,52.66983197568214)]),
sg.LineString([(675.625418945423,165.3972228360024),(762.6530944008489,731.7253641860536)]),
sg.LineString([(813.5759779875109,618.3711698398201),(155.3030884038911,92.80882456682338)]),
sg.LineString([(646.9510423922327,152.71825247755788),(616.0606416521223,905.0496581926191)])
'''
   #, sg.LineString([(500,100), (700,100)])]
drawWalls(linesDraw, walls)
path =  replanASTAR((1000,0), walls)
print path
drawPath(pathDraw, path)
turtle.done()
