import shapely.geometry as sg
import math, turtle, Utils, time
from Utils import Node, PriorityQueue, Counter


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
        print "START:", currCoord, " END:", pointList[x], "ANGLE", pAngle
        if(((pAngle > tau/8.0) and (pAngle <= 3.0*tau/8.0)) or ((pAngle > 5.0*tau/8.0) and (pAngle <= 7.0*tau/8.0)) ): #upper/lower quadrant
            if(x==0): #first point
                if(dX < 0): #point 2 xVal greater than point 1's
                    addRoundNode(currWall, pointList[x], 3, currNode, walls, goal, openStack, addedWalls) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 xVal greater than point 2's
                    addRoundNode(currWall, pointList[x], 1, currNode, walls, goal, openStack, addedWalls)
            else: #second point
                if(dX < 0): #point 2 xVal greater than point 1's
                    addRoundNode(currWall, pointList[x], 1, currNode, walls, goal, openStack, addedWalls) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 xVal greater than point 2's
                    addRoundNode(currWall, pointList[x], 3, currNode, walls, goal, openStack, addedWalls)
        else: #means it's to the side of the robot
            if(x==0): #first point
                if(dY < 0): #point 2 yVal greater than point 1's
                    addRoundNode(currWall, pointList[x], 0, currNode, walls, goal, openStack, addedWalls) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 higher than point 2
                    addRoundNode(currWall, pointList[x], 2, currNode, walls, goal, openStack, addedWalls)
            else: #second point
                if(dY < 0): #point 2 yVal greater than point 1's
                    addRoundNode(currWall, pointList[x], 2, currNode, walls, goal, openStack, addedWalls) #0 is below, 1 is right, 2 is above, 3 is left
                else: #point 1 higher than point 2
                    addRoundNode(currWall, pointList[x], 0, currNode, walls, goal, openStack, addedWalls)

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


def replanASTAR(goalCoord, walls, startState): #walls is made up of LineStrings
  path = []
  visitedPoints = Counter() #wall endpoints that have been visited
  openStack = PriorityQueue()
  listEmpty = False
  goal = goalCoord
  start = (startState[0], startState[1])#(roomba.getX(), roomba.getY())
  visitCoord = start
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

        if not(checkAndAddIntersection(currLine, currNode, walls, goal, openStack, addedWalls)):
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
    
    drawPath(pathDraw, getPath(start, currNode))
    raw_input("Press Enter to continue...")
  
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

def getPath(start, currNode):
    path = []
    print "NODE", currNode.getCoord()
    while(currNode.getCoord() != start):
          path.append(currNode.getCoord())
          currNode = currNode.getParent()
    path.append(currNode.getCoord()) #finally, add the start node, but don't try to get parent
    path.reverse()

    return path
    
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

'''
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
'''       

robotSize = 343
gapTolerance = 50
tau = 6.28318

def addRoundNode(currWall, nextCoord, direction, currNode, walls, goal, queue, addedWalls): #endpoint, currNode, walls, goal, openStack
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
  endpoint = findBounds(currWall, currNode, intersection.exterior.coords, direction)#0 is below, 1 is right, 2 is above, 3 is left

  
  a1 = math.atan2(nextCoord[1]-currCoord[1], nextCoord[0]-currCoord[0]) % tau
  a2 = math.atan2(endpoint[1]-currCoord[1], endpoint[0]-currCoord[0]) % tau
  
  addAngle = Utils.findAngleDiff(a2, a1)/2.0 #angle between the 2; need to consider 0 = tau
  targetAngle = (a1 + addAngle) % tau
  print "ANGLES", a1, a2, addAngle, targetAngle

  #given avg angle of endPoint, currPoint; extend line - want robot to be in middle of gap, not end
  targetRadius = abs(distance/ math.cos(targetAngle)) #radii should always be positive value
  targetPoint = (currCoord[0] + targetRadius * math.cos(targetAngle), currCoord[1] + targetRadius * math.sin(targetAngle)) #moves 1.5 lengths forward

  print "POINTS:", currCoord, nextCoord, targetPoint, endpoint

  intersection = False

  #check if currPos to endpoint intersects walls
  currLine = sg.LineString([currCoord, endpoint])
  if( checkAndAddIntersection(currLine, currNode, walls, goal, queue, addedWalls)):#find closest line, add endpoints instead
      print "CURR -> EP INTERSECT"
      intersection = True

  #check if endpoint to targetpoint intersects walls
  currLine = sg.LineString([endpoint, targetPoint])
  if( checkAndAddIntersection(currLine, currNode, walls, goal, queue, addedWalls)):#find closest line
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

roombaDraw = turtle.Turtle()
roombaDraw.shape('classic')
roombaDraw.color('green')
roombaDraw.penup()
def drawState(pen, coord):
  global scalingFactor
  pen.goto(coord[0]*scalingFactor, coord[1]*scalingFactor)
  pen.setheading(math.degrees(coord[2]))
  pen.stamp()

def drawPath(pen, path):
  pen.clear()
  print "DRAW", len(path)
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

walls = [
sg.LineString([(699.90751388152,-20.560875306238994),(699.9409745396003,-16.459672622619177)]),
sg.LineString([(699.3735342805629,8.51452404058388),(695.6211762357502,49.001562085749256)]),
sg.LineString([(710.8474625371407,41.2631692952762),(698.0912510873818,109.20739030277562)]),
sg.LineString([(736.3379043084678,21.70520250331842),(721.1887339461709,114.66105961276838)]),
sg.LineString([(729.2462538619425,102.68933292738218),(-78.39993567574089,-321.5386075512735)]),
sg.LineString([(1010.1067635332907,335.07524165460256),(227.06937989459522,21.118893246374114)]),
sg.LineString([(1249.7699442781097,388.1463768041982),(1252.5392646862406,391.81237600336067)]),
sg.LineString([(1236.9912641447868,372.1349048900265),(459.6588009908153,452.395844129896)]),

sg.LineString([(-181.12617799455796,-471.6106910561819),(267.7813717944074,245.1546529011967)]),
sg.LineString([(-567.1797552978514,-305.311916230979),(283.5360338483871,-104.52321971099006)]),
]
'''



sg.LineString([(803.0337673884239,644.2137995226702),(227.3237534916878,52.66983197568214)]),
sg.LineString([(813.5759779875109,618.3711698398201),(155.3030884038911,92.80882456682338)]),
sg.LineString([(646.9510423922327,152.71825247755788),(616.0606416521223,905.0496581926191)])
]

sg.LineString([(559.3508879732534,-26.935883727158597),(559.5596705174853,-22.01716078481251)]),
sg.LineString([(559.8317469021916,2.608864660281366),(555.0907459616452,62.531896937816235)]),
sg.LineString([(565.8365462364318,54.99114781080167),(555.6824932540444,110.48814839843358)]),
sg.LineString([(550.5853383359067,129.51094373232203),(543.6565519089276,151.36322908330501)]),
sg.LineString([(581.5173720795083,48.852973143115534),(-248.1023647564648,-301.3625230054063)]),
sg.LineString([(575.9174676387429,575.9552427000352),(571.7426821547715,579.4053084732257)]),
sg.LineString([(301.0486031785689,685.3048799854596),(290.4651073098652,685.8019546754731)]),
sg.LineString([(603.2516332754055,551.5704204225394),(228.8459312282703,-253.7612785775721)]),
sg.LineString([(-739.775826214365,-308.6062468454671),(-661.8553309965479,-381.59136224685153)]),
sg.LineString([(-568.8397238261502,-435.4403787105302),(50.55466903080577,185.337418475601)]),
sg.LineString([(333.41640886247865,-28.29135362971775),(332.15022555760453,33.03887364353035)]),
sg.LineString([(330.44976548016786,49.43950374219537),(327.50308023670647,69.93409055293215)]),
sg.LineString([(324.5021034286235,108.69166052911149),(-474.7318387480559,-328.4438766273223)]),


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

startState = (-182 , -34, .738)
goalState = (1000,0,0)
drawWalls(linesDraw, walls)
drawState(roombaDraw, startState)
drawState(roombaDraw, goalState)
path =  replanASTAR((1000,0), walls, startState)
print "DONE", path
drawPath(pathDraw, path)
turtle.done()
