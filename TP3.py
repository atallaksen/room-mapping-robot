import math
from tkinter import *

#Libraries
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

#set GPIO Pins
GPIO_TRIGGER = 8
GPIO_ECHO = 10

Motor1A = 16
Motor1B = 18
Motor1E = 22

Motor2A = 11
Motor2B = 13
Motor2E = 15

Servo = 12

clkB = 36
dtB = 38
clkA = 37
dtA = 35

##-------From Online--------
#https://github.com/modmypi/Rotary-Encoder/blob/master/rotary_encoder.py
GPIO.setup(clkA, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dtA, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(clkB, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dtB, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
##-------End From Online--------

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

GPIO.setup(Motor1A,GPIO.OUT)
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(Motor1E,GPIO.OUT)

GPIO.setup(Motor2A,GPIO.OUT)
GPIO.setup(Motor2B,GPIO.OUT)
GPIO.setup(Motor2E,GPIO.OUT)

GPIO.setup(Servo,GPIO.OUT)

#set motor speeds
GPIO.output(Motor1E,GPIO.LOW)
GPIO.output(Motor2E,GPIO.LOW)
    
pwm1 = GPIO.PWM(Motor1A, 1000) #frequency @ 1000
pwm2 = GPIO.PWM(Motor1B, 1000) #frequency @ 1000

pwm3 = GPIO.PWM(Motor2A, 1000) #frequency @ 1000
pwm4 = GPIO.PWM(Motor2B, 1000) #frequency @ 1000

servo = GPIO.PWM(Servo, 100) #frequency @ 100

#Below functions control basic
def goForward(inches):
    GPIO.output(Motor1E,GPIO.HIGH)
    GPIO.output(Motor2E,GPIO.HIGH)
    pwm1.start(60)
    pwm2.start(0)
    pwm3.start(0)
    pwm4.start(60)
    time.sleep(inches/11)
    GPIO.output(Motor1E,GPIO.LOW)
    GPIO.output(Motor2E,GPIO.LOW)

def goBackward(inches):
    GPIO.output(Motor1E,GPIO.HIGH)
    GPIO.output(Motor2E,GPIO.HIGH)
    pwm1.start(0)
    pwm2.start(60)
    pwm3.start(60)
    pwm4.start(0)
    time.sleep(inches/11)
    GPIO.output(Motor1E,GPIO.LOW)
    GPIO.output(Motor2E,GPIO.LOW)

def turnLeft():
    GPIO.output(Motor1E,GPIO.HIGH)
    GPIO.output(Motor2E,GPIO.HIGH)
    pwm1.start(0)
    pwm2.start(60)
    pwm3.start(0)
    pwm4.start(60)
    time.sleep(.8)
    GPIO.output(Motor1E,GPIO.LOW)
    GPIO.output(Motor2E,GPIO.LOW)

def turnRight():
    GPIO.output(Motor1E,GPIO.HIGH)
    GPIO.output(Motor2E,GPIO.HIGH)
    pwm1.start(60)
    pwm2.start(0)
    pwm3.start(60)
    pwm4.start(0)
    time.sleep(.8)
    GPIO.output(Motor1E,GPIO.LOW)
    GPIO.output(Motor2E,GPIO.LOW)

def distance(x0,y0,x1,y1):
    return ( (x1 - x0)**2 + (y1 - y0)**2 )**0.5

def make2DList(rows, cols, val):
    a = []
    for row in range(rows): a += [[val]*cols]
    return a

##---------From onlinee--------------
#http://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/
def sonarDistance():
    #set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    #set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    #save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    #save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    #time difference between start and arrival
        TimeElapsed = StopTime - StartTime
    #multiply with the sonic speed (34300 cm/s)
    #and divide by 2, because there and back
    distance = (TimeElapsed * 34300)/2

    return distance
##-------End from online--------------

#Used to draw the arcs representing the locus of possible points that the
#sonar sensor could have detected
def drawSemiCircle(canvas, data, rad, cx, cy, angle):
    
    angle = math.pi/180*angle
    ultraSonicAngle = math.pi/180*data.ultraSonicAngle

    #Represent corners of rectangle that surround the arc
    x0 = cx + rad*math.cos(angle - ultraSonicAngle/2) 
    y0 = cy + rad*math.sin(angle - ultraSonicAngle/2)
    x1 = ( cx + rad*math.cos(angle + ultraSonicAngle/2) +
                         (rad-rad*math.cos(ultraSonicAngle/2))*math.cos(angle))
    y1 = ( cy + rad*math.sin(angle + ultraSonicAngle/2) +
                        (rad-rad*math.cos(ultraSonicAngle/2))*math.sin(angle) )
                        
    canvas.create_arc(cx-rad,cy-rad,cx+rad, cy+rad,
                        start = (angle - ultraSonicAngle/2)*180/math.pi,
                        extent = ultraSonicAngle*180/math.pi, 
                        style=ARC, outline = 'light gray')
                        
def drawArcs(canvas, data):
    #draw arcs representing signal received from sonar sensor
    #light gray used because these represent all possible points
    for i in range(len(data.semiCircles)):
        for j in range(len(data.semiCircles[i])):
            drawSemiCircle(canvas, data, data.semiCircles[i][j][0], 
                            data.semiCircles[i][j][1], 
                            data.semiCircles[i][j][2], 
                            data.angle[i])
                        
def drawGrid(canvas, data):
    for x in range(0, data.width, 20):
        canvas.create_line(x, 0, x, data.height, fill = 'light blue')
    for y in range(0, data.height, 20):
        canvas.create_line(0, y, data.width, y, fill = 'light blue')
    canvas.create_text(10, 20, text='Scale: 20 cm per grid', anchor=NW,
                       fill="blue", font="Arial 18 bold")  
                       
def drawRobotPath(canvas, data):
    #draw the robot path in red
    for i in range(len(data.robotPath)):
        canvas.create_line(data.robotPath[i][0], data.robotPath[i][1],
                           data.robotPath[i][2], data.robotPath[i][3],
                           fill = 'red')
                        
def drawBoundaryLines(canvas, data):
    #draw tangent lines connecting arcs
    #black used because they much more likely represent a boundary/wall       
    for i in range(len(data.connectingLines)):
        for j in range(len(data.connectingLines[i])-1):
            #very small imaginary numbers were appearing
            #therefore, I specify the real parts
            canvas.create_line(data.connectingLines[i][j][0].real, 
                            data.height - data.connectingLines[i][j][1].real, 
                            data.connectingLines[i][j][2].real, 
                            data.height - data.connectingLines[i][j][3].real)

#Used to plot the path of the robot
def addPathLine(data):
    cx = data.semiCircles[0][-2][1]
    cy = data.semiCircles[0][-2][2]
    cx_PlusOne = data.semiCircles[0][-1][1]
    cy_PlusOne = data.semiCircles[0][-1][2]
    data.robotPath.append((cx,cy,cx_PlusOne,cy_PlusOne))
    
    
#The following equations were taken the paper "Echolocation-From range to 
#outline segments" by Philip John McKerrow and were adapted for use in this
#program
def addLine(data):
    
    for i in range(len(data.semiCircles)):
        
        r_t = data.semiCircles[i][-2][0]
        r_tPlusOne = data.semiCircles[i][-1][0]
        
        d = distance(data.semiCircles[i][-1][1], data.semiCircles[i][-1][2],
                        data.semiCircles[i][-2][1], data.semiCircles[i][-2][2])
                        
        alpha = data.angle[i]*math.pi/180
        beta = data.ultraSonicAngle/2*math.pi/180
        
        low = min(-d*math.cos(alpha+beta), -d*math.cos(alpha-beta))
        high = max(-d*math.cos(alpha+beta), -d*math.cos(alpha-beta))

        sgn = abs(data.angle[i])/data.angle[i]
        
        if ( (r_tPlusOne - r_t > low) and (r_tPlusOne - r_t < high)
            and (r_t**2) > 1):
            
            e = d*r_tPlusOne/(r_tPlusOne - r_t)
            m = sgn*(((e**2)/(r_t**2)-1)**(1/2))
            
            x_t = -e/(1+m**2) + data.semiCircles[i][-2][1]
            y_t = -e*m/(1+m**2) + data.semiCircles[i][-2][2]
            x_tPlusOne = (d*m**2-e)/(1+m**2) + data.semiCircles[i][-2][1]
            y_tPlusOne = (-d*m-e*m)/(1+m**2) + data.semiCircles[i][-2][2]
            
            data.connectingLines[i].append((x_t, y_t, x_tPlusOne, y_tPlusOne))
        else:
            #placeholder value
            data.connectingLines[i].append((0,0,0,0))

servo.start(12) #Initialize servo position before continuing

####################################
# Animation
####################################

def init(data):
    data.rad = 0
    data.cx = 0 
    data.cy = data.height/2 #begin plotting in center of screen
    data.angle = [90, 60, 30, -30, -60, -90] #degrees
    data.servoFrequency = [21, 17.17, 14.33, 8.67, 5.83, 3.5]
    data.ultraSonicAngle = 20 #degrees
    data.semiCircles = make2DList(len(data.angle),1,(data.rad,data.cx,data.cy))
    data.connectingLines = [[]]*len(data.angle)
    data.robotPath = []
    data.paused = False

def mousePressed(event, data):
    # use event.x and event.y
    pass

def keyPressed(event, data):
    # use event.char and event.keysym
    pass

def timerFired(data):
    
    #Pause timerFired to allow the robot to gather information
    data.paused = True 

    #Check if obstacles in front
    servo.ChangeDutyCycle(12)
    time.sleep(1) #Give a second for the servo to move
    d = sonarDistance()
    print("Distance in front is", d)

    if d<15:
        #Infinite loop when the robot cannot go any further
        while True:
            print("Done!!!") 

    #If no obstacles in front, move forward
    time.sleep(1)
    goForward(11)
    print("Moving forward")
    data.cx += 21 #manually measured distance is actually ~21 cm per move
    time.sleep(2) #Give a second for the robot to move

    #defined for the sake of clarity
    cx = data.cx
    cy = data.cy

    #sweep the room using the servo motor and sonar sensor
    for i in range(len(data.angle)):
        servo.ChangeDutyCycle(data.servoFrequency[i])
        time.sleep(1) #Give a second for the servo to move
        rad = sonarDistance()
        data.semiCircles[i].append((rad,cx,cy))
    
    #create tangent lines and robot path lines based on new data
    for i in range(len(data.semiCircles)):
        if len(data.semiCircles[i]) > 1:
            addLine(data)
            addPathLine(data)

    #resume Tkinter
    data.paused = False

def redrawAll(canvas, data):

    drawGrid(canvas, data)

    drawArcs(canvas, data)

    drawBoundaryLines(canvas, data)

    drawRobotPath(canvas, data)
    
    

            
####################################
# Taken from the course website
####################################

def run(width=300, height=300):
    def redrawAllWrapper(canvas, data):
        canvas.delete(ALL)
        canvas.create_rectangle(0, 0, data.width, data.height,
                                fill='white', width=0)
        redrawAll(canvas, data)
        canvas.update()    

    def mousePressedWrapper(event, canvas, data):
        mousePressed(event, data)
        redrawAllWrapper(canvas, data)

    def keyPressedWrapper(event, canvas, data):
        keyPressed(event, data)
        redrawAllWrapper(canvas, data)

    def timerFiredWrapper(canvas, data):
        timerFired(data)
        redrawAllWrapper(canvas, data)
        # pause, then call timerFired again
        canvas.after(data.timerDelay, timerFiredWrapper, canvas, data)
    # Set up data and call init
    class Struct(object): pass
    data = Struct()
    data.width = width
    data.height = height
    data.timerDelay = 1000 # milliseconds
    init(data)
    # create the root and the canvas
    root = Tk()
    canvas = Canvas(root, width=data.width, height=data.height)
    canvas.pack()
    # set up events
    root.bind("<Button-1>", lambda event:
                            mousePressedWrapper(event, canvas, data))
    root.bind("<Key>", lambda event:
                            keyPressedWrapper(event, canvas, data))
    timerFiredWrapper(canvas, data)
    # and launch the app
    root.mainloop()  # blocks until window is closed
    print("bye!")

run(400, 600)
