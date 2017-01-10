import turtle, math, Utils
import shapely.geometry as sg

dataSet = [
(161.94828964680775, -3.875355719124985, 6.249594018924888, 29), (161.94828964680775, -3.875355719124985, 0.11640700879675744, 30), (161.94828964680775, -3.875355719124985, 0.2342889126555113, 30), (161.94828964680775, -3.875355719124985, 0.3678758075586331, 33), (161.94828964680775, -3.875355719124985, 0.5033825646389487, 37), (161.94828964680775, -3.875355719124985, 0.629992105735096, 32), (161.94828964680775, -3.875355719124985, 0.7682864354401779, 30), (161.94828964680775, -3.875355719124985, 0.910080341028676, 23), (161.94828964680775, -3.875355719124985, 1.0841588745780664, 17), (161.94828964680775, -3.875355719124985, 1.2441042043873052, 14), (161.94828964680775, -3.875355719124985, 1.3963651315455727, 173), (161.94828964680775, -3.875355719124985, 1.5510705101615976, 192), (161.94828964680775, -3.875355719124985, 1.7193894571794068, 186), (161.94828964680775, -3.875355719124985, 1.882472416441233, 182), (161.94828964680775, -3.875355719124985, 2.0364796635926883, 236), (161.94828964680775, -3.875355719124985, 2.131571224250689, 126), (161.94828964680775, -3.875355719124985, 2.289073093107135, 111), (161.94828964680775, -3.875355719124985, 2.457396003325947, 155), (161.94828964680775, -3.875355719124985, 2.6171628370089843, 176), (161.94828964680775, -3.875355719124985, 2.779373131644813, 503), (161.94828964680775, -3.875355719124985, 2.949610949862394, 472), (161.94828964680775, -3.875355719124985, 3.1133920408250173, 465), (161.94828964680775, -3.875355719124985, 3.275427802535647, 363), (161.94828964680775, -3.875355719124985, 3.4491572702346387, 102), (161.94828964680775, -3.875355719124985, 3.6223621481806094, 110), (161.94828964680775, -3.875355719124985, 3.7948778118052107, 74), (161.94828964680775, -3.875355719124985, 3.9738511938984193, 52), (161.94828964680775, -3.875355719124985, 4.149849589388775, 49), (161.94828964680775, -3.875355719124985, 4.322012223961974, 46), (161.94828964680775, -3.875355719124985, 4.484397051523002, 43), (161.94828964680775, -3.875355719124985, 4.654634869976811, 43), (161.94828964680775, -3.875355719124985, 4.833778821557988, 42), (161.94828964680775, -3.875355719124985, 5.0193765279343125, 45), (161.94828964680775, -3.875355719124985, 5.194855288086302, 50), (161.94828964680775, -3.875355719124985, 5.371028216738085, 53), (161.94828964680775, -3.875355719124985, 5.544407627845484, 40), (161.94828964680775, -3.875355719124985, 5.715520092407678, 34), (161.94828964680775, -3.875355719124985, 5.888030802090081, 29), (161.94828964680775, -3.875355719124985, 6.071534113364013, 27), (161.94828964680775, -3.875355719124985, 6.252070365145783, 28)
               
]

scalingFactor = .5
minDist = 20
sightThreshold = 100
angleTolerance = .2
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

def getCoord(dataPoint):
    pX = dataPoint[0] + dataPoint[3]*math.cos(dataPoint[2])
    pY = dataPoint[1] + dataPoint[3]*math.sin(dataPoint[2])
    return (pX, pY, dataPoint[3])

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

def pointsAdjacent(p1, p2):
    global minDist
    p1X = p1[0]
    p1Y = p1[1]
    p2X = p2[0]
    p2Y = p2[1]
    #print Utils.getDist(p1X, p1Y, p2X, p2Y)
    return (Utils.getDist(p1X, p1Y, p2X, p2Y) < minDist)

def slopeSimilar(p1, p2, currAngle):
    global angleTolerance
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
        
    return (abs( abs( math.atan2(dy, dx)) - currAngle) < angleTolerance)
    

def extendLine(left, right):
    if((left[0] - right[0]) != 0):
        slope = (left[1]-right[1]) / (left[0]-right[0]) #still need to handle undefined slope
        x1 = left[0]
        y1 = left[1]
        leftX = left[0] - 100 #100 to the left
        leftY = slope*(leftX - x1) + y1
        rightX = right[0] + 100
        rightY = slope*(rightX - x1) + y1
    else:
        leftX = left[0]
        rightX = right[0]
        if(left[1] > right[1]): #left is above right
            leftY = (left[1] + 100)
            rightY = (right[1] - 100)
        else: #right is above left
            leftY = (left[1] - 100)
            rightY = (right[1] + 100)
    return sg.LineString([(leftX, leftY), (rightX, rightY)])
    

def manageData(dataList, currPos):
    global scalingFactor, minDist, sightThreshold
    index = 0
    #print dataList
    while(len(dataList) > 0):
        print "ITER", len(dataList), (index+1)%len(dataList), (index-1)%len(dataList)
        length = 1
        startPoint = getCoord(dataList[index])#dataList.pop(index))
        currPoint = startPoint
        if(len(dataList) > 1):
            print "HELLO"
            #find right bound
            rightBound = currPoint
            rightInner = currPoint
            
            rightPoint = getCoord(dataList[(index+1)%len(dataList)])
            rDx = rightPoint[0] - currPoint[0]
            rDy = rightPoint[1] - currPoint[1]
            rightAngle = abs(math.atan2(rDy, rDx))
            
            leftPoint = getCoord(dataList[(index-1)%len(dataList)])
            lDx = leftPoint[0] - currPoint[0]
            lDy = leftPoint[1] - currPoint[1]
            leftAngle = abs(math.atan2(rDy, rDx))

            if( (rightAngle - leftAngle) <  angleTolerance):
                currAngle = (leftAngle + rightAngle)/2
            else:
                print "CONTINUE"
                continue
            distMet = True

            print "RIGHT"
            while(distMet and pointsAdjacent(currPoint, rightPoint) and slopeSimilar(currPoint, rightPoint, currAngle) and len(dataList) > 0):
                print "R:", length
                print currPoint, rightPoint
                currPoint = getCoord(dataList.pop((index+1)%len(dataList))) #remove the adjacent point (value set to current point
                print currPoint
                length += 1
                if(len(dataList) > 0):
                    rightPoint = getCoord(dataList[(index+1)%len(dataList)])
                rightInner = currPoint
                rightBound = rightPoint #since right point is not close enough
                if(rightBound[2] > sightThreshold): #over sightThreshold (min lidar reading for wall)
                    distMet = False
            #find left bound
            currPoint = startPoint
            leftBound = currPoint
            leftInner = currPoint

            print "LEFT"
            if(len(dataList) > 0):
                leftPoint = getCoord(dataList[(index-1)%len(dataList)])
                distMet = True
                while(distMet and pointsAdjacent(currPoint, leftPoint) and slopeSimilar(currPoint, rightPoint, currAngle) and len(dataList) > 0):
                    print "L:", length
                    print currPoint, leftPoint
                    currPoint = getCoord(dataList.pop((index-1)%len(dataList))) #gets rid of current point
                    length +=1
                    if(len(dataList) > 0):
                        leftPoint = getCoord(dataList[(index-1)%len(dataList)])
                    
                    leftInner = currPoint
                    leftBound = leftPoint
                    if(leftBound[2] > sightThreshold):#over sightThreshold (min lidar reading for wall)
                        distMet = False
                    print "DATA", currPoint, pointsAdjacent(currPoint, leftPoint), distMet
                    
            print "LENGTH:", length
            print leftBound, leftInner, rightBound, rightInner
                
            if(length > 3):
                drawLines(linesDraw, leftInner, rightInner)
                '''
                wallLine = extendLine(leftInner, rightInner)
                leftLine = extendLine(currPos, leftBound)
                rightLine = extendLine(currPos, rightBound)
                print "LINES:", wallLine, leftLine, rightLine #wall = orange, left=red, right=blue

                leftWall = wallLine.intersection(leftLine).coords[0]
                print wallLine, rightLine
                rightWall = wallLine.intersection(rightLine).coords[0]
                print "DRAW LINE", leftWall, rightWall
                drawLines(linesDraw, leftWall, rightWall)
                '''
            else:
                drawPoint(pointsDraw, currPoint)
        else:
            print "LAST POINT"
            drawPoint(pointsDraw, currPoint)
        if(len(dataList) > 0):
            dataList.pop(index) #get rid of point

for x in range(len(dataSet)):
    print getCoord(dataSet[x])
    drawPoint(pointsDraw, getCoord(dataSet[x]))
pointsDraw.color('green')
manageData(dataSet, (104.498233867, -0.525013256799))

turtle.done()
