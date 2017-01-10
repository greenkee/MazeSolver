import serial, time

ser = serial.Serial(port = 'COM3', baudrate = 115200, bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)

a = '140 0 7 71 16 69 16 67 16 69 16 71 16 71 16 71 16 141 0'
b = '140 0 1 62 32'
play = '141 0'

def writeString(s):
    arr = s.split(" ")
    print arr
    for x in range(len(arr)):
        print int(arr[x])
        ser.write(chr(int(arr[x])))
#ser.open()
print ser.isOpen()

ser.write(chr(128))
ser.write(chr(131))
time.sleep(.1)

#writeString(b)
writeString(a)

'''
ser.write(chr(140))
ser.write(chr(0))
ser.write(chr(1))
ser.write(chr(62))
ser.write(chr(32))
'''


'''
ser.write(chr(140))
ser.write(chr(0))
ser.write(chr(1))
ser.write(chr(62))
ser.write(chr(32))
ser.write(chr(141))
ser.write(chr(0))

ser.write(bytearray(b'140 0 140 0 7 71 16 69 16 67 16 69 16 71 16 71 16 71 16'))
ser.write(bytearray(b'141 0'))
\ser.write(bytearray(b'128'))
'''
ser.close()
