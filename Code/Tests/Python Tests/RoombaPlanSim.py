import turtle, math, Utils, RoombaFunctASTAR, RoombaFunctParseData
import shapely.geometry as sg

walls = [
    sg.LineString([(627.4024277443739,-10.39005973), (516.8143707,379.0235000139201)]),
sg.LineString([(506.3844763191273,314.5377887), (-79.77497494,295.4270395603385)]),
sg.LineString([(539.1678120028023,-476.5771661), (511.9900126,-534.968751713549)]),
sg.LineString([(535.3925139514566,-478.9598338), (-32527.86523,-27675.1844373457)]),
sg.LineString([(568.5969927900495,211.8762903), (356.8887547,375.471671301236)]),
sg.LineString([(406.8549203269355,318.8841001), (-1767.754154,728.5217528077476)]),
sg.LineString([(650.8228779601759,-331.5073771), (401.7823315,173.5603382595359)]),
sg.LineString([(411.1849893116445,-653.4298874), (344.4693389,-702.5319807301519)]),
sg.LineString([(315.7764438759716,-676.0091977), (444.4905919,-198.5063451957612)]),
sg.LineString([(-215.3881277064804,-205.4618868), (-242.6271691,-153.9067003103313)]),
sg.LineString([(-215.4652312825066,-202.6039456), (-242.3818181,-265.7206449833323)]),
sg.LineString([(-18.97839958918979,-589.4777046), (18.13015281,-58.58976119022952)]),
sg.LineString([(675.1357165177261,-420.6509369), (-2148.838546,-5018.725225285434)]),
sg.LineString([(431.0774016250791,230.3246281), (371.9589053,281.0799762594969)]),
sg.LineString([(-214.7838712609164,-189.9500818), (-242.3028535,-263.3447618584457)]),
sg.LineString([(-37.22802671296134,-574.6987103), (-29.22224523,-618.1147779213379)]),
sg.LineString([(17.03133110284057,-611.9723295), (130.3891899,-145.0206438570512)]),
sg.LineString([(693.4615146302908,-380.4645474), (195.0623222,-397.8556971986337)]),
sg.LineString([(450.0906438115998,224.0937582), (-3867.653671,1707.582318626101)]),
sg.LineString([(669.8376915063737,-429.8278355), (393.9866478,13.33573789053531)]),
sg.LineString([(-26.29150859192097,-582.1039493), (-133.4580572,-522.5401279885006)]),
    ]

scalingFactor = .1

#all in mm

samePointDist = 30 #for points to be considered the same (and be averaged)
sightThreshold = 1000
angleTolerance = Utils.tau/8
robotSize = 351
wallBuffer = robotSize/3
robotWidthAndBuffer = robotSize/2 + wallBuffer
minDist = robotSize/1.5 #between points
#graphics  
window = turtle.Screen()
window.delay(0)
window.bgcolor('white')

linesDraw = turtle.Turtle()
linesDraw.shape('classic')
linesDraw.color('red')
linesDraw.hideturtle()
linesDraw.penup()
linesDraw.pensize(3)

pointsDraw = turtle.Turtle()
pointsDraw.shape('classic')
pointsDraw.color('blue')
pointsDraw.hideturtle()
pointsDraw.penup()

pathDraw = turtle.Turtle()
pathDraw.shape('classic')
pathDraw.color('green')
pathDraw.hideturtle()
pathDraw.penup()
pathDraw.pensize(3)


def drawLines(pen, p1, p2):
    global scalingFactor

    p1X = p1[0]
    p1Y = p1[1]
    p2X = p2[0]
    p2Y = p2[1]
    pen.penup()
    pen.goto(p1X*scalingFactor, p1Y*scalingFactor)
    pen.pendown()
    pen.goto(p2X*scalingFactor, p2Y*scalingFactor)
  
def drawPoint(pen, data):
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


currPos = (265, 38)

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
path = RoombaFunctASTAR.replanASTAR(currPos, (2000,0), walls, robotWidthAndBuffer)
for x in range(len(path)-1):
    drawLines(pathDraw, path[x], path[x+1])
turtle.done()
