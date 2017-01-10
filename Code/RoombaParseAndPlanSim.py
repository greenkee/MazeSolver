import turtle, math, Utils, RoombaFunctASTAR, RoombaFunctParseData
import shapely.geometry as sg

#list of data points (xPos, yPos, angle, lidar reading)
dataSet = [
(155.49988008066043, -0.17323102875728605, 6.28259389394115, 31), (155.49988008066043, -0.17323102875728605, 0.1611264434580601, 34), (155.49988008066043, -0.17323102875728605, 0.3252813221543511, 35), (155.49988008066043, -0.17323102875728605, 0.4772192733744647, 396), (155.49988008066043, -0.17323102875728605, 0.6523640681208261, 282), (155.49988008066043, -0.17323102875728605, 0.8180546612260372, 294), (155.49988008066043, -0.17323102875728605, 0.9190229626727021, 257), (155.49988008066043, -0.17323102875728605, 1.1345082510254239, 171), (155.49988008066043, -0.17323102875728605, 1.306344841887375, 136), (155.49988008066043, -0.17323102875728605, 1.4898672169533718, 211), (155.49988008066043, -0.17323102875728605, 1.6822964296219083, 212), (155.49988008066043, -0.17323102875728605, 1.8698390975827064, 268), (155.49988008066043, -0.17323102875728605, 2.0575562984687035, 198), (155.49988008066043, -0.17323102875728605, 2.243004194039326, 189), (155.49988008066043, -0.17323102875728605, 2.4349060356839973, 485), (155.49988008066043, -0.17323102875728605, 2.619481643916405, 578), (155.49988008066043, -0.17323102875728605, 2.810165527332947, 312), (155.49988008066043, -0.17323102875728605, 3.0001516562465373, 368), (155.49988008066043, -0.17323102875728605, 3.110162096767017, 102), (155.49988008066043, -0.17323102875728605, 3.3055587463617897, 67), (155.49988008066043, -0.17323102875728605, 3.4122511751343776, 51), (155.49988008066043, -0.17323102875728605, 3.594557855339193, 37), (155.49988008066043, -0.17323102875728605, 3.7889129658001375, 18), (155.49988008066043, -0.17323102875728605, 3.9829141063890057, 12), (155.49988008066043, -0.17323102875728605, 4.178485288908977, 25), (155.49988008066043, -0.17323102875728605, 4.391335231023692, 16), (155.49988008066043, -0.17323102875728605, 4.603661197164965, 16), (155.49988008066043, -0.17323102875728605, 4.8252377855396515, 12), (155.49988008066043, -0.17323102875728605, 5.04001324824893, 20), (155.49988008066043, -0.17323102875728605, 5.256877447643234, 37), (155.49988008066043, -0.17323102875728605, 5.456462510155009, 42), (155.49988008066043, -0.17323102875728605, 5.646966202228983, 36), (155.49988008066043, -0.17323102875728605, 5.844806312776548, 34), (155.49988008066043, -0.17323102875728605, 6.044216842453061, 29), (155.49988008066043, -0.17323102875728605, 6.241533354225027, 32), (155.49988008066043, -0.17323102875728605, 0.1497308426084727, 32)              
]

scalingFactor = .1 #how much to scale drawing

#all in mm
samePointDist = 30 #dist between points for them to be considered the same (and be averaged); for overlap
sightThreshold = 1000 #max dist point can be until it is ignored for calculations (too far away)
angleTolerance = Utils.tau/8 #dist between angles for slope to be considered same
robotSize = 351 #size of robot
wallBuffer = robotSize/5 #space put around wall
robotWidthAndBuffer = robotSize/2 + wallBuffer #distance that path should be from wall
minDist = robotSize/1.5 #dist between points for them to be considered adjacent

#graphics

#canvas
window = turtle.Screen()
window.delay(0) #all points drawn instantly
window.bgcolor('white')

#used to draw walls
linesDraw = turtle.Turtle()
linesDraw.shape('classic')
linesDraw.color('red')
linesDraw.hideturtle()
linesDraw.penup()
linesDraw.pensize(3)

#used to draw data points
pointsDraw = turtle.Turtle()
pointsDraw.shape('classic')
pointsDraw.color('blue')
pointsDraw.hideturtle()
pointsDraw.penup()

#used to draw path
pathDraw = turtle.Turtle()
pathDraw.shape('classic')
pathDraw.color('green')
pathDraw.hideturtle()
pathDraw.penup()
pathDraw.pensize(3)


def drawLines(pen, p1, p2): #method to draw line from point to point
    global scalingFactor
    p1X = p1[0]
    p1Y = p1[1]
    p2X = p2[0]
    p2Y = p2[1]
    pen.penup()
    pen.goto(p1X*scalingFactor, p1Y*scalingFactor)
    pen.pendown()
    pen.goto(p2X*scalingFactor, p2Y*scalingFactor)
  
def drawPoint(pen, data): #stamps point at correct point
  global scalingFactor, sightThreshold
  pen.goto(data[0]*scalingFactor, data[1]*scalingFactor)
  '''
  if(data[3] < sightThreshold):
      pen.color('red')
  else:
      pen.color('green')
      '''
  pen.stamp()
  pen.goto(0,0)


currPos = (155.499880081, -0.173231028757)
for x in range(len(dataSet)):
    print RoombaFunctParseData.getCoord(dataSet[x])
    #print dataSet[x]
    drawPoint(pointsDraw, RoombaFunctParseData.getCoord(dataSet[x]))
    #raw_input()
pointsDraw.color('purple')
#walls = manageData(dataSet, currPos)
walls = []
result = RoombaFunctParseData.manageData(dataSet, walls, currPos, minDist, sightThreshold,
                                        angleTolerance, robotWidthAndBuffer, samePointDist)
walls = result
##newPoints = result[1]
##
##print "DRAW", len(newPoints)
##for x in range(len(newPoints)):
##    #print RoombaFunctParseData.getCoord(dataSet[x])
##    print newPoints[x]
##    drawPoint(pointsDraw, newPoints[x])
##    #raw_input()

for x in range(len(walls)):
    drawLines(linesDraw, walls[x].coords[0], walls[x].coords[1])
path = RoombaFunctASTAR.replanASTAR(currPos, (1000,0), walls, robotWidthAndBuffer)
for x in range(len(path)-1):
    drawLines(pathDraw, path[x], path[x+1])
turtle.done()
