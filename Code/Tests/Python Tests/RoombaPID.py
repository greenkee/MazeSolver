import serial, time, math
from Arduino import Arduino
from Roomba import Roomba

def reachedPoint(x1, x2, y1, y2):
  tolerance = 100;
  return (  math.sqrt( float((x1-x2)**2) + float((y1-y2)**2) ) < tolerance)


pidIndex = 0

accum_error = 0
diff_error = 0
cte = 0
prev_cte = 0


tau = 6.28318

def PID(path, params, elapsed): #time elapsed in seconds
    global cte, prev_cte, accum_error, diff_error, pidIndex
    prev_cte = cte
    
    xDist = roomba.getX() - path[pidIndex][0]
    yDist = roomba.getY() - path[pidIndex][1] #error for y

    xLength = path[pidIndex+1][0] - path[pidIndex%len(path)][0] #horiz length of current path
    yLength = path[pidIndex+1][1] - path[pidIndex%len(path)][1] #vert length of current path

    u = float( xDist * xLength + yDist*yLength) / float(xLength*xLength + yLength*yLength) #percent of path traveled - something to do with vector projection

    #cte = (float)yDist
    goalAngle = math.atan2( (path[pidIndex+1][1] - roomba.getY()),(path[pidIndex+1][0] - roomba.getX())) % tau
    angleDiff = (goalAngle - roomba.getAngle()) #direction robot needs to turn
    if(abs(angleDiff) > tau/2):
      if(angleDiff < 0):
        angleDiff += tau
      else:
        angleDiff -= tau
      #angleDiff = (angleDiff- tau)#%tau
    
    #cte = (float( yDist * xLength - xDist*yLength) / float(xLength*xLength + yLength*yLength)) -  angleDiff 
    cte = -angleDiff
    if(reachedPoint(roomba.getX(), path[pidIndex+1][0], roomba.getY(), path[pidIndex+1][1])):
        pidIndex  += 1#+= 1 
        print path[pidIndex%len(path)][0]
        print path[pidIndex%len(path)][1]
        print "REACHED POINT"

    '''
    if(pidIndex == (len(path)-2)):
        #print cte, goalAngle, roomba.getAngle(), angleDiff
        if(reachedPoint(roomba.getX(), path[pidIndex+1][0], roomba.getY(), path[pidIndex+1][1])):
            print path[pidIndex%len(path)][0]
            print path[pidIndex%len(path)][1]
            print "REACHED POINT
            pidIndex  = -1#+= 1 

    elif(u > 1):
        pidIndex += 1
        print ("REACHED POINT")
        print path[pidIndex%len(path)][0]
        print path[pidIndex%len(path)][1]
        '''
  
    diff_error = float(cte - prev_cte)/elapsed # maybe multiply 3000 as constant
    accum_error += cte*elapsed

    
    steeringRad = -params[0] * cte - params[1] * diff_error - params[2] * accum_error
    print "ADIFF", math.degrees(angleDiff), goalAngle, roomba.getAngle()
    print "PID", cte, diff_error, steeringRad
    if(steeringRad != 0):
        steeringRad = (2.0/steeringRad/10.0)#smaller radius means sharper turn - 20 is just a guess should get values from .01 to .5 maybe?
    if(steeringRad > 1):
        steeringRad = 1
    elif(steeringRad < -1):
        steeringRad = -1
    #print steeringRad
    roomba.drive(.3, steeringRad)

#PID Vars
tau_p = 3.0
tau_i = 0
tau_d = 0.0

path = [ (0,0), (1000, 0), (1000, 1000), (0, 1000), (0, 2000), (1000,2000) ]

#path = [ (0,0), (1000, 0), (1000, 1000), (0, 1000), (0,0)]
params = [tau_p, tau_i, tau_d]

#Serial Ports
roombaSer = serial.Serial(port = 'COM3', baudrate = 115200, bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 1)
arduinoSer = serial.Serial(port = 'COM4', baudrate = 115200, bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 1)

    
roomba = Roomba(0, 0, 0, roombaSer)
arduino = Arduino(arduinoSer)
roomba.start()

t1= 0
t2 = 0
angle = 0.0

initAngle = float(arduino.readAngle())
start = time.time()
t1 = start
kDrift = kDrift = -0.007


while (pidIndex < (len(path)-1)):
    try:
        time.sleep(.01) #in seconds
        t2 = time.time() - t1
        t1 = time.time()
        
        reading = arduino.readAngle()
        elapsed = time.time() - start
        angle = float(reading) - initAngle + elapsed*kDrift
        radAngle = (math.radians(angle)) % tau #keeps angle within [0, 2pi]
        roomba.update(t2, radAngle, 0, True)
        print "VALS", roomba.getX(), roomba.getY(), roomba.getAngle()
        PID(path, params, t2)
        
    except (KeyboardInterrupt, SystemExit):
        print "VALS", roomba.getX(), roomba.getY(), roomba.getAngle()
        roomba.end()
        arduino.end()
        raise Exception("STOPPED")
        break
roomba.end()
arduino.end()
    
