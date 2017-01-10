import turtle, math, Utils
import shapely.geometry as sg

def getCoord(dataPoint):
    pX = dataPoint[0] + dataPoint[3]* 10 *math.cos(dataPoint[2]) #convert lidar to mm
    pY = dataPoint[1] + dataPoint[3]* 10 *math.sin(dataPoint[2]) #convert lidar to mm
    return (pX, pY, dataPoint[3])

def pointsAdjacent(p1, p2, minDist):
    p1X = p1[0]
    p1Y = p1[1]
    p2X = p2[0]
    p2Y = p2[1]
    #print Utils.getDist(p1X, p1Y, p2X, p2Y)
    return (Utils.getDist(p1X, p1Y, p2X, p2Y) < minDist)

def slopeSimilar(p1, p2, currAngle, angleTolerance):
    #finds curr angle
    checkAngle = getAngleBetween(p1,p2)
    print "DIFF:", checkAngle, Utils.findAngleDiff(checkAngle, currAngle)
    return (abs(Utils.findAngleDiff(checkAngle, currAngle))< angleTolerance)

def getAngleBetween(p1, p2): #gives from 0 to 2pi
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.atan2(dy, dx)%Utils.tau

def extendLine(left, right, robotWidthAndBuffer):
    
    if((left[0] - right[0]) != 0):
        slope = (right[1] - left[1]) / (right[0] - left[0])
        dX = robotWidthAndBuffer/ (math.sqrt( slope**2 + 1))
        dY = dX * slope
        rightHigherLeft = (right[1] > left[1])
        if(rightHigherLeft):
            if(slope > 0):
                rightX = right[0] + dX
                rightY = right[1] + dY
                leftX = left[0] - dX
                leftY = left[1] - dY
            else:
                rightX = right[0] - dX
                rightY = right[1] - dY
                leftX = left[0] + dX
                leftY = left[1] + dY
        else:#left higher right
            if(slope > 0):
                rightX = right[0] - dX
                rightY = right[1] - dY
                leftX = left[0] + dX
                leftY = left[1] + dY
            else:
                rightX = right[0] + dX
                rightY = right[1] - dY
                leftX = left[0] - dX
                leftY = left[1] + dY
        print "EXT:", robotWidthAndBuffer, " DX:", dX, " DY:", dY
    else:
        leftX = left[0]
        rightX = right[0]
        if(left[1] > right[1]): #left is above right
            leftY = (left[1] + robotWidthAndBuffer)
            rightY = (right[1] - robotWidthAndBuffer)
        else: #right is above left
            leftY = (left[1] - robotWidthAndBuffer)
            rightY = (right[1] + robotWidthAndBuffer)
    return sg.LineString([(leftX, leftY), (rightX, rightY)])
    
def convertData(dataList, samePointDist):
    newData = []
    for x in range(len(dataList)):
        newData.append(getCoord(dataList[x]))
    return newData

def manageData(dataList, walls, currPos, minDist, sightThreshold, angleTolerance, robotWidthAndBuffer, samePointDist):
    lookThreshold = 5 #will look at most 5 points back (5 point overlap)
    
    fixedData = convertData(dataList, samePointDist)
    print "DATA:", fixedData, len(fixedData)

    #adjoin doubled-over points
    x=0
    while(x < lookThreshold):
        currPoint = fixedData[x]
        j = 1
        while (j<=lookThreshold):
            nextPoint = fixedData[len(fixedData)-j]
            #not itself, but within tolerance
            if(Utils.getDist(currPoint[0], currPoint[1], nextPoint[0], nextPoint[1]) > 0 and
               Utils.getDist(currPoint[0], currPoint[1], nextPoint[0], nextPoint[1]) < samePointDist):
                print "Index", x, len(fixedData)
                newPoint = ( (currPoint[0] + nextPoint[0])/2.0 , (currPoint[1] + nextPoint[1])/2.0, (currPoint[2] + nextPoint[2])/2.0 )
                #remove both points
                p1 = fixedData.remove(currPoint)
                p2 = fixedData.remove(nextPoint)
                fixedData.insert(x, newPoint)
                print "POPPED:", p1, p2, newPoint, fixedData
                #raw_input()
            j+=1
        x+=1

    copyData = list(fixedData)
    print "DATA:", fixedData, len(fixedData)
    index = 0
    #print fixedData
    while(len(fixedData) > 0):
        print "ITER", len(fixedData), (index+1)%len(fixedData), (index-1)%len(fixedData)
        length = 1
        startPoint = fixedData[index]#fixedData.pop(index))
        currPoint = startPoint
        if(len(fixedData) > 1):
            print "HELLO"
            #find right bound

            #makes "absolute val" of slope
            '''
            if(rightAngle > Utils.tau/2):
                rightAngle -= Utils.tau/2
            if(leftAngle > Utils.tau/2):
                leftAngle -= Utils.tau/2
                '''
            

            #raw_input()
            '''
            if( Utils.findAngleDiff(rightAngle, leftAngle) <  angleTolerance):
                currAngle = Utils.findAverageAngle(leftAngle, rightAngle)
            else:
                print "CONTINUE"
                fixedData.pop(index)
                #raw_input()
                continue
            '''
            distMet = True

            print "RIGHT"

            rightBound = currPoint
            rightInner = currPoint
            
            rightPoint = fixedData[(index+1)%len(fixedData)]
            rDx = rightPoint[0] - currPoint[0]
            rDy = rightPoint[1] - currPoint[1]
            rightAngle = math.atan2(rDy, rDx)%Utils.tau

            angles = [rightAngle]
            
            currAngle = sum(angles)/len(angles)
            while(distMet and pointsAdjacent(currPoint, rightPoint, minDist) and
                  slopeSimilar(currPoint, rightPoint, currAngle, angleTolerance) and len(fixedData) > 0):
                
                angles.append(getAngleBetween(currPoint, rightPoint))
                currAngle = sum(angles)/len(angles)
                print "ANGLE:", getAngleBetween(currPoint, rightPoint), currAngle
               
                currPoint = fixedData.pop((index+1)%len(fixedData)) #remove the adjacent point (value set to current point
                length += 1
                if(len(fixedData) > 0):
                    rightPoint = fixedData[(index+1)%len(fixedData)]
                rightInner = currPoint
                rightBound = rightPoint #since right point is not close enough
                if(rightBound[2] > sightThreshold): #over sightThreshold (min lidar reading for wall)
                    distMet = False

            #find left bound
            currPoint = startPoint
            leftBound = currPoint
            leftInner = currPoint

            print "LEFT"
            currAngle = sum(angles)/len(angles)
            if(len(fixedData) > 0):
                leftPoint = fixedData[(index-1)%len(fixedData)]
                distMet = True
                while(distMet and pointsAdjacent(leftPoint, currPoint, minDist) and
                      slopeSimilar(leftPoint, currPoint, currAngle, angleTolerance) and len(fixedData) > 0):
                    angles.append(getAngleBetween(leftPoint, currPoint))
                    currAngle = sum(angles)/len(angles)
                    print "ANGLE:", getAngleBetween(leftPoint, currPoint), currAngle
                    currPoint = fixedData.pop((index-1)%len(fixedData)) #gets rid of current point
                    length +=1
                    if(len(fixedData) > 0):
                        leftPoint = fixedData[(index-1)%len(fixedData)]
                    
                    leftInner = currPoint
                    leftBound = leftPoint
                    if(leftBound[2] > sightThreshold):#over sightThreshold (min lidar reading for wall)
                        distMet = False
                    #print "DATA", currPoint, pointsAdjacent(currPoint, leftPoint, minDist), distMet
                    
            print "LENGTH:", length
            print leftBound, leftInner, rightBound, rightInner
                
            if(length > 3):
                '''
                wallLine = extendLine(leftInner, rightInner)
                leftLine = extendLine(currPos, leftBound)
                rightLine = extendLine(currPos, rightBound)
                print "LINES:", wallLine, leftLine, rightLine #wall = orange, left=red, right=blue

                leftWall = wallLine.intersection(leftLine).coords[0]
                rightWall = wallLine.intersection(rightLine).coords[0]
                print "DRAW LINE", leftWall, rightWall
                newWall = sg.LineString([(leftWall[0], leftWall[1]), (rightWall[0], rightWall[1])]) 
                '''
                newWall = extendLine(leftInner, rightInner, robotWidthAndBuffer)
                #newWall = sg.LineString([leftInner, rightInner])
                walls.append(newWall)
            #else:
                #drawPoint(pointsDraw, currPoint)
        else:
            print "LAST POINT"
            #drawPoint(pointsDraw, currPoint)
        if(len(fixedData) > 0):
            fixedData.pop(index) #get rid of point
    print "DATA WALLS", walls
    return walls

