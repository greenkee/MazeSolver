import turtle, math, Utils

dataSet = [
    (0,0,6.280642769,23),
(0,0,6.276333846,24),
(0,0,6.205004698,23),
(0,0,6.144846494,22),
(0,0,6.06653561,23),
(0,0,5.98386224,24),
(0,0,5.913929356,23),
(0,0,5.830035092,35),
(0,0,5.739856389,186),
(0,0,5.660748512,344),
(0,0,5.570560602,400),
(0,0,5.516336099,401),
(0,0,5.434884041,407),
(0,0,5.34994258,337),
(0,0,5.275124448,433),
(0,0,5.192452333,381),
(0,0,5.105241107,211),
(0,0,5.028858455,210),
(0,0,4.953689583,218),
(0,0,4.868923073,194),
(0,0,4.786423818,181),
(0,0,4.718412051,171),
(0,0,4.631026711,158),
(0,0,4.547656046,84),
(0,0,4.473010772,78),
(0,0,4.38719581,82),
(0,0,4.305046039,79),
(0,0,4.23563759,77),
(0,0,4.143888508,78),
(0,0,4.061040606,96),
(0,0,3.987614552,121),
(0,0,3.913319181,141),
(0,0,3.825932167,139),
(0,0,3.731390976,145),
(0,0,3.65657117,153),
(0,0,3.566741532,164),
(0,0,3.473769882,128),
(0,0,3.401743858,99),
(0,0,3.314019076,91),
(0,0,3.21075124,189),
(0,0,3.119701964,77),
(0,0,3.031093638,67),
(0,0,2.940218057,62),
(0,0,2.851435617,59),
(0,0,2.773472962,62),
(0,0,2.6825978,70),
(0,0,2.593814523,109),
(0,0,2.499447447,118),
(0,0,2.408221545,100),
(0,0,2.32729476,98),
(0,0,2.235719793,106),
(0,0,2.132452793,197),
(0,0,2.054666764,207),
(0,0,1.96029927,218),
(0,0,1.831026864,209),
(0,0,1.769646511,262),
(0,0,1.673534106,155),
(0,0,1.595921773,115),
(0,0,1.433530795,78),
(0,0,1.273761577,373),
(0,0,1.113992358,361),
(0,0,0.9440998121,301),
(0,0,0.7799672708,281),
(0,0,0.6067163345,283),
(0,0,0.5104783565,27),
(0,0,0.3805684263,25),
(0,0,0.3005557334,23),
(0,0,0.1489895627,21),
(0,0,6.265069962,19),
]

scalingFactor = .5
minDist = 30
threshold = 30
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

def drawLines(pen, p1, p2):
    global scalingFactor

    p1X = p1[0] + p1[3]*math.cos(p1[2])
    p1Y = p1[1] + p1[3]*math.sin(p1[2])
    p2X = p2[0] + p2[3]*math.cos(p2[2])
    p2Y = p2[1] + p2[3]*math.sin(p2[2])
    pen.penup()
    pen.goto(p1X*scalingFactor, p1Y*scalingFactor)
    pen.pendown()
    pen.goto(p2X*scalingFactor, p2Y*scalingFactor)
  
def drawPoint(pen, data):
  global scalingFactor, threshold
  pen.goto(data[0]*scalingFactor, data[1]*scalingFactor)
  pen.setheading(math.degrees(data[2]))
  pen.forward(data[3]*scalingFactor)
  '''
  if(data[3] < threshold):
      pen.color('red')
  else:
      pen.color('green')
      '''
  pen.stamp()

def pointsAdjacent(p1, p2):
    global minDist
    p1X = p1[0] + p1[3]*math.cos(p1[2])
    p1Y = p1[1] + p1[3]*math.sin(p1[2])
    p2X = p2[0] + p2[3]*math.cos(p2[2])
    p2Y = p2[1] + p2[3]*math.sin(p2[2])
    #print Utils.getDist(p1X, p1Y, p2X, p2Y)
    return (Utils.getDist(p1X, p1Y, p2X, p2Y) < minDist)

def manageData(dataList):
    global scalingFactor, minDist
    index = 0
    print dataList
    while(len(dataList) > 0):
        print "ITER", len(dataList), (index+1)%len(dataList)
        length = 1
        startPoint = dataList[index]
        #find right bound
        currPoint = startPoint
        rightBound = currPoint
        
        rightPoint = dataList[(index+1)%len(dataList)]
        while(pointsAdjacent(currPoint, rightPoint) and len(dataList) > 0):
            currPoint = dataList.pop((index+1)%len(dataList)) #remove the adjacent point (value set to current point
            length += 1
            if(len(dataList) > 0):
                rightPoint = dataList[(index+1)%len(dataList)]
            else:
                return
            rightBound = rightPoint #since right point is not close enough
        
        #find left bound
        currPoint = startPoint
        leftPoint = dataList[(index-1)%len(dataList)]
        leftBound = currPoint
        while(pointsAdjacent(currPoint, leftPoint)and len(dataList) > 0):
            currPoint = dataList.pop((index-1)%len(dataList)) #gets rid of current point
            length +=1
            if(len(dataList) > 0):
                leftPoint = dataList[(index-1)%len(dataList)]
            leftBound = leftPoint

        print "LENGTH:", length
        if(length > 1):
            drawLines(linesDraw, leftBound, rightBound)
        else:
            drawPoint(pointsDraw, currPoint)

        dataList.pop(index) #get rid of point

for x in range(len(dataSet)):
    drawPoint(pointsDraw, dataSet[x])

#manageData(dataSet)

turtle.done()
