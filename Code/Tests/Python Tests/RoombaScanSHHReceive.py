import turtle
import RoombaFunctASTAR
import RoombaFunctParseData
import Utils

import paramiko


def printWalls(walls):
  for x in range(len(walls)):
    print "WALL",walls[x].coords[0], walls[x].coords[1]

def drawPath(pen, path):
  pen.clear()
  for x in range(len(path)-1):#everything but last
    drawLines(pen, path[x], path[x+1])

def drawRobot(pen, roomba):
  global scalingFactor
  pen.goto(roomba.getX()*scalingFactor, roomba.getY()*scalingFactor)
  pen.setheading(math.degrees(roomba.getAngle()))
  pen.stamp()

def drawLines(pen, p1, p2):
  global scalingFactor
  pen.penup()
  pen.goto(p1[0]*scalingFactor, p1[1]*scalingFactor)
  pen.pendown()
  pen.goto(p2[0]*scalingFactor, p2[1]*scalingFactor)

def endProgram():
  printWalls(walls)
  turtle.done() 

#graphics  
window = turtle.Screen()
window.delay(0)
window.bgcolor('white')

roombaDraw = turtle.Turtle()
roombaDraw.shape('classic')
roombaDraw.color('green')
#roombaDraw.resizemode('user')
#roombaDraw.shapesize(.3, .3, .1)


linesDraw = turtle.Turtle()
linesDraw.shape('classic')
linesDraw.color('red')
linesDraw.pensize(5)
#linesDraw.resizemode('user')
#linesDraw.shapesize(.3, .3, .3)

#roombaDraw.hideturtle()
roombaDraw.penup()
linesDraw.hideturtle()

pathDraw = turtle.Turtle()
pathDraw.shape('classic')
pathDraw.color('blue')
pathDraw.pensize(5)
pathDraw.hideturtle()

scalingFactor = .1

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect( '192.168.1.20', username = 'pi', password = 'mnrlo8qt' )

stdin, stdout, stderr  = ssh.exec_command('cd Justin; python RoombaScanSSHSend.py')

path = []
walls = []

def manageData(message):
    #get first char; c=
    if(c == 'w'):#wall
        #parse wall; get p1 and p2
        drawLines(linesDraw, p1, p2)
    elif(c== 'r'):#robot pos
        #parse string to position
        drawRobot(roombaDraw, pos)
    elif(c == 'p'):
        pathDraw.clear()

while (True):
    try:
        for line in iter(lambda: stdout.readline(2048), ""):
            manageData((line, end=""))

            #every time received, add to list of walls, print new one
            #print new planned path
            #every time robot received, print position
    except (KeyboardInterrupt, SystemExit):
        print "VALS", roomba.getX(), roomba.getY(), roomba.getAngle(), roomba.getLidar()
        endProgram()
        raise Exception("STOPPED")
        break
endProgram()
    
