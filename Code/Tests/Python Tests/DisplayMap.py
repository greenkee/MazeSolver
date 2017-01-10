import turtle

robotPath = [ (0,0, 90), (50,50, 0), (100,50, 300) ]
walls = [ ((10,10), (20,20)), ((20,30), (40,30)) ]

window = turtle.Screen()
window.delay(0)

window.bgcolor('white')
roomba = turtle.Turtle()
roomba.shape('classic')
roomba.color('green')
#roomba.resizemode('user')
roomba.shapesize(5, 5, .1)


lines = turtle.Turtle()
lines.shape('classic')
lines.color('red')
lines.resizemode('user')
#lines.shapesize(.3, .3, .3)

roomba.hideturtle()
roomba.penup()
for x in range(len(robotPath)):
    coord = robotPath[x]
    roomba.goto(coord[0], coord[1])
    roomba.setheading(coord[2])
    roomba.stamp()
#roomba.showturtle()

lines.hideturtle()
for x in range(len(walls)):
    lines.penup()
    p1 = walls[x][0]
    p2 = walls[x][1]
    lines.goto(p1[0], p1[1])
    lines.pendown()
    lines.goto(p2[0], p2[1])
#lines.showturtle()
