import serial, time
from Arduino import Arduino
arduinoSer = serial.Serial(port = 'COM4', baudrate = 115200, bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 1)
arduino = Arduino(arduinoSer)
time.sleep(.1)
angle = 0.0
global t1, t2
t1 = time.time()


initAngle = float(arduino.readString())
start = time.time()

kDrift = 0
while(True):
    try:
        print "ITER"
        t2 = time.time() - t1
        
        reading = arduino.readString()
        print reading
        reading = float(reading)

        t1 = time.time()

        elapsed = time.time() - start

        #angle = reading - initAngle + elapsed*kDrift
        time.sleep(.1)
        print reading
    except (KeyboardInterrupt, SystemExit):
        raise Exception("STOPPED")
        arduino.end()
        break
    
