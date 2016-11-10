# simple demo of Python turtle graphics.  Details and docs at:
#    https://docs.python.org/2/library/turtle.html

from turtle import *
import math

# set up mode (initially at origin, (0,0) - center of screen pointing to right along X axis)
# speed(0 is fastest), drawing color is red

mode("standard")
color('red')
speed(0)

# now draw a red box 100 x 100 centered at origin (center of screen)
penup()
setposition(0,0)
pendown()
forward(100)
left(90)
forward(100)
left(90)
forward(100)
left(90)
forward(100)

#turn left to again be in horizontal X axis
left(90)
penup()


#Now draw a blue 100x100 box centered at lower right of screen and scaled by a factor of 5

scale=5
offsetx= -scale/2*100
offsety= -scale/2*100

color('blue')

penup()
setposition(offsetx,offsety)
pendown()
forward(scale*100)
left(90)
forward(scale*100)
left(90)
forward(scale*100)
left(90)
forward(scale*100)

# now draw a filled blue circle at (50,75) scaled appropriately)
penup()
fillcolor('blue')
setposition(offsetx +scale*50,offsety + scale*75)
pendown()
color('blue')
fill(True)
circle(3)  
penup()
fill(False)

# now draw a green filled circle at (15,33) scaled appropriately

penup()
fillcolor('green')
setposition(offsetx +scale*15,offsety + scale*33)
pendown()
color('green')
fill(True)
circle(3)  
penup()
fill(False)
shape('arrow')
seth(math.pi * 180 / math.pi)

#move back to origin and exit when graphics screen clicked
penup()
setposition(0,0)  
exitonclick()