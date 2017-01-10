import serial, time, math
from Arduino import Arduino
from Roomba import Roomba

def reachedPoint(x1, x2, y1, y2):
  tolerance = 20;
  return (  sqrt( float(sq(x1-x2)) + float(sq(y1-y2)) ) < tolerance)


pidIndex = 0

accum_error = 0
diff_error = 0
cte = 0
prev_cte = 0
tau_p = 3.0
tau_i = 0
tau_d = 0.0
path = [ (0,0), (500, 0), (500, 500), (0, 500), (0,1000), (1000, 1000) ]

def PID(elapsed): #time elapsed in seconds
    global cte, prev_cte, accum_error, diff_error, pidIndex
    prev_cte = cte
    
    xDist = roomba.getX() - path[pidIndex][0]
    yDist = roomba.getY() - path[pidIndex][1] #error for y

    xLength = path[pidIndex+1][0] - path[pidIndex][0] #horiz length of current path
    yLength = path[pidIndex+1][1] - path[pidIndex][1] #vert length of current path

    u = float( xDist * xLength + yDist*yLength) / float(xLength*xLength + yLength*yLength) #percent of path traveled - something to do with vector projection

    #cte = (float)yDist
    cte = (float( yDist * xLength - xDist*yLength) / float(xLength*xLength + yLength*yLength))
    + ( robot.getAngle() - math.atan2( (path[pidIndex+1][1] - roomba.getY()) ,
                                       (path[pidIndex+1][0] - roomba.getX())) )


    if(pidIndex == (len(path)-2)):
        if(reachedPoint(robot.getX(), path[pidIndex+1][0], robot.getY(), path[pidIndex+1][1])): 
            pidIndex += 1
            print ("NEXT POINT")

    elif(u > 1):
        pidIndex += 1
        print ("REACHED POINT")
        print path[pidIndex][0]
        print path[pidIndex][1]
  
    diff_error = float(cte - prev_cte)/elapsed # maybe multiply 3000 as constant
    accum_error += cte*elapsed

    
    steeringRad = -tau_p * cte - tau_d * diff_error - tau_i * accum_error
    #print cte, diff_error, steeringRad
    if(steeringRad != 0):
        steeringRad = (2.0/steeringRad/10.0)#smaller radius means sharper turn - 20 is just a guess should get values from .01 to .5 maybe?
    if(steeringRad > 1):
        steeringRad = 1
    elif(steeringRad < 1):
        steeringRad = -1
    #print steeringRad
    roomba.drive(.3, steeringRad)



global t1, t2
angle = 0.0

initAngle = 0 
start = time.time()
t1 = start
kDrift = kDrift = -0.007


tau = 6.28318
while (True):
    time.sleep(1) #in seconds
    t2 = time.time() - t1
    t1 = time.time()
    
    reading = 0
    elapsed = time.time() - start
    angle = float(reading) - initAngle + elapsed*kDrift
    radAngle = (math.radians(angle)) % tau #keeps angle within [0, 2pi]

    PID(t2)
