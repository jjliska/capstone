# AME 485/486 - Capstone
# Team: Reflection
# Members: Joshua Liska - Programming/Engineering/Fusion360 Modeling/Building
#          Ivan Mendoza - Programming/Building
#          Jack Carroll - Sound Designer/Unity Programming
#          Albert Bang - Visual Artist/Unity Programming/3D Modeling
#
# References: Emotion Model: https://drive.google.com/file/d/1192YC8mYKaCbCoACP8hTfr9PCMC2iN30/view?usp=sharing from https:/github.com/jaydeepthik/
#             Face Detection: https://realpython.com/face-detection-in-python-using-a-webcam/
#             Tkinter and general pyhton syntax
#             Two-way communication between Python 3 and Unity (C#) - Y. T. Elashry @ https://github.com/Siliconifier/Python-Unity-Socket-Communication.git
#
# Use Case: A user will step infront of the camera and the camera will then being to track that user.
# Every (delayPeriod) the emotion of the user will be taken and passed to a unity program that will also use facial reference data
# to calculate where the user is and look at them. This facial data will also be used to drive several servos to a corrected position
# and follow their movements accordingly.
#
#
# Layout of code:
# Packages
# Variables
# Computation
# Visual UI (unneccessary code aside from debugging)
# Webcam handler and webcam
# Exit Handlers
#


# Packages
import UdpComms as U
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from tkinter import *
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import cv2
from PIL import Image, ImageTk
from datetime import datetime
import time
from threading import Thread
import serial
import tensorflow as tf
from tensorflow import keras
import os

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

#Serial Communication
#---------------------------------------------------------------------------------------------------------
try:
  ser = serial.Serial(
    port='COM3',
    timeout=100
  )
  ser.flush()
except:
  print("Serial port is not connected")

sock = U.UdpComms(udpIP="127.0.0.1", portTX=8000, portRX=8001, enableRX=True, suppressWarnings=True)

#COMPUTATION
#---------------------------------------------------------------------------------------------------------
#These variables will he changed by the serial information
# x is changed exclusively by the size of the box
# y is changed by the location of the box vertically
# z is changed by the placement of the box horizontally
x = 0
y = 40
zRotation = 90

#CONSTANTS <<LEAVE UNTOUCHED>>
#Base is technically irrelevant but it looks better to have a sized base for the graph
arm1 = 32.5
arm2 = 32.5
hand = 12.0
base = 5.0
screenSize = 17.0

#All masses assume the entirety of the mass is at the end
#of the point allowing for a larger "buffer" and less torque on the system
#Units: kg (rough rounded up estimates)
arm1Mass = .5+.09
arm2Mass = .5+.09
handMass = 1
totMass = arm1Mass + arm2Mass + handMass

#Gear ratio
gear1 = 45
gear2 = 30
gearRatio = gear1/gear2

#Units: kg/cm
a1Torque = 60*gearRatio
a2Torque = 25*gearRatio
a3Torque = 25*gearRatio

#MaxAcceptableAngles
a1MaxAngle = [0,180]
#if x is negative the angle must remain [45,90] else [45,180]
a2MaxAnglePos = [45,180]
a2MaxAngleNeg = [45,90]
a3MaxAngle = [0,180]
a4MaxAngle = [0,270]

facePos = [0,0,0,0]
facialEmotion = ""

#Variable for if a face isnt detected for a period of time then
noFaceDetected = 10
noFaceTime = 10

#Takes off 15% to ensure the system is not overloading
def acceptableRange(input):
  return input*0.6

#GLOBAL INSTANCING OF MATHEMATICS VARIABLES
#-------------------------------------------------------------------
#This is used once so it doesn't need to be edited or called back to q

mLengthA1 = a1Torque/(arm1Mass+arm2Mass+handMass)
mLengthA2 = a2Torque/(arm2Mass+handMass)
mLengthA3 = a3Torque/(handMass)

maxXHigh = mLengthA1-base
maxXLow = 0.0

if ((mLengthA1+base)*(-1)) < (arm1*-1):
  maxXLow = arm1*-1
  accXLow = maxXLow
else:
  maxXLow = (mLengthA1+base)*(-1)
  accXLow = acceptableRange(maxXLow)

accXHigh = acceptableRange(maxXHigh)

maxYLowXLow = (screenSize/2.0)+base+2.0
maxYLowXHigh = (screenSize/2.0)+2.0

#End of startup variables

#MATH EQUATIONS FOR HANDLING LATER
#------------------------------------------------------------------------------
def findYHigh(x,radius):
  return np.sqrt(radius**2-x**2)

def maxYCircle(ox,oy,x,y,radius):
  return np.sqrt((x-ox)**2+(y-oy)**2)

#Calculates variables neccessary for the function during startup
def lawOfCosines(a,b,c):
  return np.arccos((a**2+b**2-c**2) / (2.0*a*b))

def distance(x1,y1,x2,y2):
  return np.sqrt((x1-x2)**2+((y1-y2)**2))

def distanceMidArm(angle):
  opp = np.cos(angle) * arm1
  adj = np.sin(angle) * arm1
  return opp, adj

def angles(x,y):
  dist = distance(x,y,0,0)
  D1 = np.arctan2(y,x)
  D2 = lawOfCosines(dist,arm1,arm2)
  A1 = D1 + D2
  A2 = lawOfCosines(arm1,arm2,dist)
  return A1, A2

def rotateX(ox,x,angle):
  return ox+np.cos(angle)*(x-ox)

def rotateZ(ox,x,angle):
  return np.sin(angle)*(x-ox)

#Establisihing global variables prior to changing them
#a1 is between base and arm1
#a2 is between arm1 and arm2
a1, a2 = angles(x,y)
mx, my = distanceMidArm(a1)
dist = distance(mx,my,x+hand,y)
#a3 is between arm2 and hand
if my <= y:
  a3 = lawOfCosines(arm2,hand,dist)
else:
  a3 = np.radians(360)-lawOfCosines(arm2,hand,dist)

#a4 is z rotation
a4 = np.radians(zRotation)

xPlot = [0,0,mx,x,x+hand]
yPlot = [0,base,my+base,y+base,y+base]

xPlot3D = [rotateX(0,x,np.radians(a4)) for x in xPlot]
zPlot3D = [rotateZ(0,x,np.radians(a4)) for x in xPlot]

def angleChecker(array,angle):
  if angle >= np.radians(array[0]) and angle <= np.radians(array[1]):
    return True
  else:
    return False

def angleHandler(angle1,angle2,angle3,x):
  if angleChecker(a1MaxAngle,angle1) and angleChecker(a3MaxAngle,angle3):
    if x >= 0:
      if angleChecker(a2MaxAnglePos,angle2):
        return True
      else:
        return False
    else:
      if angleChecker(a2MaxAngleNeg,angle2):
        return True
      else:
        return False
  else:
    return False

def positionHandlerX(x):
  if x >= accXLow and x <= accXHigh:
    return True
  else:
    return False

def positionHandlerY(x,y):
  if x >= accXLow and x <= accXHigh:
    if x >= 0:
      if y >= maxYLowXHigh and y <= findYHigh(x,arm1+arm2+base):
        return True
      else:
        return False
    else:
      if y >= maxYLowXLow and y <= findYHigh(x,arm1+arm2+base):
        return True
      else:
        return False
  else:
    return False

def updateVariables(tempX,tempY,tempZRot):
  global x,y
  global a1,a2,a3
  global mx,my
  global dist
  global xPlot,yPlot
  #a1 is between base and arm1
  #a2 is between arm1 and arm2
  a1temp, a2temp = angles(x,tempY)
  mxtemp, mytemp = distanceMidArm(a1temp)
  disttemp = distance(mxtemp,mytemp,x+hand,tempY)
  #a3 is between arm2 and hand
  a3temp = lawOfCosines(arm2,hand,disttemp)

  #a4 is z rotation
  a4temp = np.radians(tempZRot)

  yPlottemp = [0,base,mytemp+base,tempY+base,tempY+base]

  if angleHandler(a1temp,a2temp,a3temp,x):
    if positionHandlerY(x,tempY):
      y = tempY
      a1,a2,a3 = a1temp,a2temp,a3temp
      mx,my = mxtemp,mytemp
      dist = disttemp
      yPlot = yPlottemp

  #a1 is between base and arm1
  #a2 is between arm1 and arm2
  a1temp, a2temp = angles(tempX,y)
  mxtemp, mytemp = distanceMidArm(a1temp)
  disttemp = distance(mxtemp,mytemp,tempX+hand,y)
  #a3 is between arm2 and hand
  a3temp = lawOfCosines(arm2,hand,disttemp)

  xPlottemp = [0,0,mxtemp,tempX,tempX+hand]

  if angleHandler(a1temp,a2temp,a3temp,tempX):
    if positionHandlerX(tempX):
      x = tempX
      a1,a2,a3 = a1temp,a2temp,a3temp
      mx,my = mxtemp,mytemp
      dist = disttemp
      xPlot= xPlottemp

  if angleChecker(a4MaxAngle,a4temp):
    global zRotation
    zRotation = tempZRot
    global a4
    a4 = a4temp
    global xPlot3D
    xPlot3D = [rotateX(0,x,a4) for x in xPlot]
    global zPlot3D
    zPlot3D = [rotateZ(0,x,a4) for x in xPlot]
  toSerial()
  toSocket()

def toSocket():
  centerFaceX, centerFaceY = getCenterBox()
  socketString = str(centerFaceX)+","+str(centerFaceY)+","+str(np.degrees(a4))+","+facialEmotion
  sock.SendData(socketString)

def toSerial():
  try:
    if not ser.is_open:
      ser.open()
    elif ser.is_open:
      ser.flushInput()
      ser.flushOutput()
      serialString = str(np.degrees(a1))+","+str(np.degrees(a2))+","+str(np.degrees(a3))+","+str(np.degrees(a4))+"\n"
      ser.write(bytes(serialString,'utf-8'))
    else:
      print("Serial port not open")
  except:
    pass

#CAMERA POSITION HANDLEING
#-----------------------------------------------------------------------------------------------------------
#Get the center of the camera
# x is "zoom"
# y is "tilt"
# z is "pan"
cameraRes = [1280,720]
centerScreen=[cameraRes[0]/2,cameraRes[1]/2]

#Max velocity allowed in any direction
#Later this will be passed as a percetage so the system does not exceeded this
#X,Y,Z

def getCenterBox():
  return (facePos[0]+(facePos[2]/2)-centerScreen[0]),(centerScreen[1]-facePos[1]-(facePos[3]/2))

screenOffset = 1.0/6.0
boundsRight = centerScreen[0]*screenOffset
boundsLeft = centerScreen[0]*screenOffset*(-1)

boundsTop = centerScreen[1]*screenOffset
boundsBottom = centerScreen[1]*screenOffset*(-1)

# Arbitrary values that seem to give a good distance between the arm and the
# person so that there is no way the arm will malfunction and hit the person
boundsFar = 100
boundsNear = 140

def facePosHandler():
  # x axis of camera is z axis of robot
  # y axis of cameria is y axis of robot
  # size of facial bounding box is the x axis of robot

  if noFaceDetected < noFaceTime:
    z1C, y1C = getCenterBox()
    x1C = facePos[2]
    tempX,tempY,tempRotZ = 0,0,0
    # Z rotation
    if z1C >= boundsRight:
      tempRotZ = velocityHandler(2,-1)
    elif z1C <= boundsLeft:
      tempRotZ = velocityHandler(2,1)
    else:
      tempRotZ = velocityHandler(2,0)

    # Y movement
    if y1C >= boundsTop:
      tempY = velocityHandler(1,1)
    elif y1C <= boundsBottom:
      tempY = velocityHandler(1,-1)
    else:
      tempY = velocityHandler(1,0)

    # X movement
    if x1C >= boundsNear:
      tempX = velocityHandler(0,-1)
    elif x1C <= boundsFar:
      tempX = velocityHandler(0,1)
    else:
      tempX = velocityHandler(0,0)
  else:
    tempX = velocityHandler(0,0)
    tempY = velocityHandler(1,0)
    tempRotZ = velocityHandler(2,0)

  if not tempX == 0 or not tempY == 0 or not tempRotZ == 0:
    updateVariables(x+tempX,y+tempY,zRotation+tempRotZ)

# 10,.5,2.0 looks amazing its almost where i want it in terms of smoothness
# 10,.49,2.0 is that good good
smoothingValue = 10
maxVelocity = 0.49
rotationAdjustment = 2.0
velocity = [0.0,0.0,0.0]

# Handles acceleration and velocity to create a smooth start and stop
def velocityHandler(velNum,direction):
  global velocity
  if velNum == 2:
    accelVal = maxVelocity/smoothingValue
    maxVelocityAdjusted = rotationAdjustment*maxVelocity
  else:
    accelVal = maxVelocity/smoothingValue
    maxVelocityAdjusted = maxVelocity
  # No face detected or the face is in the bounds deaccelerate
  if noFaceDetected > noFaceTime or direction == 0:
    if velocity[velNum] >= -1*accelVal and velocity[velNum] <= accelVal:
      velocity[velNum] = 0
    elif velocity[velNum] > 0:
      velocity[velNum] -= accelVal
    elif velocity[velNum] < 0:
      velocity[velNum] += accelVal
  # Increases acceleration to create and easing effect on the servos so they do not immediately start jumping to large degree of angle changes
  else:
    if direction == -1 and velocity[velNum] > -1*maxVelocityAdjusted:
      velocity[velNum] -= accelVal
    elif direction == 1 and velocity[velNum] < maxVelocityAdjusted:
      velocity[velNum] += accelVal
  return velocity[velNum]

# UN-NEEDED FUNCTIONS THESE ARE SIMPLY FOR UI ELEMENTS AND DEBUGGING THE DATA
# -------------------------------------------------------------------------------------------------
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# -------------------------------------------------------------------------------------------------

#GUI ELEMENTS
#------------------------------------------------------------------------------------------------------------
#Instancing the gui frame
root = Tk()
root.title("Capstone")
root.geometry("1920x1080")

#Text frame for the label
container = Frame(root,bg="darkgrey")
container.pack(padx=12,pady=12)

textFrame = Frame(container)
textFrame.pack(side=LEFT)

label1 = Label(textFrame, text="Temp", justify=LEFT,bg="darkgrey",width=30,height=100)
label1.pack(fill=BOTH)

graphFrame = Frame(container)
graphFrame.pack(side=LEFT)

fig = plt.figure(figsize=(8,16),dpi=100)
fig.set_facecolor("darkgrey")
ax = fig.add_subplot(2,1,1)
ax.set_facecolor("darkgrey")
line1, = ax.plot([0,0,0,0,0],[0,0,0,0,0],'r-',markersize=4,marker='o',color='black',linestyle='solid',linewidth=2,markerfacecolor='red')
ax.set_xlim([-1*((arm1*2)+hand),(arm1*2)+hand])
ax.set_ylim([0,(arm1*2)+base])

ax2 = fig.add_subplot(2,1,2,projection='3d')
ax2.set_facecolor("darkgrey")
line3D = ax2.plot3D(xPlot3D,zPlot3D,yPlot,markersize=1,color='black',linewidth=2)
ax2.set_xlim([-1*((arm1*2)+hand),(arm1*2)+hand])
ax2.set_ylim([-1*((arm1*2)+hand),(arm1*2)+hand])
ax2.set_zlim([0,(arm1*2)+base])

graph = FigureCanvasTkAgg(fig, graphFrame)
graph.get_tk_widget().pack()

webcamFrame = Label(container,text="Temp",width=cameraRes[0],height=cameraRes[1])
webcamFrame.pack(side=TOP,padx=12,pady=12)
textFrame2 = Frame(container)
textFrame2.pack(side=LEFT)
label2 = Label(textFrame2,text="Temp",justify=LEFT,bg="darkgrey")
label2.pack(side=TOP)

def getTime():
  now = datetime.now()
  return now.strftime("%H:%M:%S")

def updateText():
  tempStringData = "----------------    "+getTime()+"    ----------------\n"

  #Text data part 1
  #All of this is unneccessary and simply for visual graphs and data outputs to verify the program is properly working
  tempStringData += "\n  Point of arm: ("+"{:.2f}".format(mx)+", "+"{:.2f}".format(my)+")"

  tempStringData += "\n\n  Verifying distance:\n    Arm 1: "+"{:.2f}".format(distance(0,0,mx,my))+"\n    Arm 2: "+"{:.2f}".format(distance(mx,my,x,y))

  tempStringData += "\n\n  Unfiltered Angles:\n    Angle 1: "+"{:.5f}".format(np.degrees(a1))+"\n    Angle 2: "+"{:.5f}".format(np.degrees(a2))+"\n    Angle 3: "+"{:.5f}".format(np.degrees(a3))+"\n    Angle 4: "+"{:.5f}".format(np.degrees(a4))

  tempStringData += "\n\n  Max length armatures:\n    @ a1: "+"{:.2f}".format(mLengthA1)+"\n    @ a2: "+"{:.2f}".format(mLengthA2)+"\n    @ a3: "+"{:.2f}".format(mLengthA3)

  tempStringData += "\n\n  Max range of X: ["+"{:.2f}".format(maxXLow)+", "+"{:.2f}".format(maxXHigh)+"]"
  tempStringData += "\n  Max acc range of X: ["+"{:.2f}".format(accXLow)+", "+"{:.2f}".format(accXHigh)+"]"

  tempStringData += "\n\n  Max range of Y:"
  tempStringData += "\n    neg X: @ "+"{:.2f}".format(accXLow)+" ["+"{:.2f}".format(maxYLowXLow)+", "+"{:.2f}".format(findYHigh(accXLow,arm1+arm2+base))+"]"
  tempStringData += "\n    pos X: @ "+"{:.2f}".format(accXHigh)+" ["+"{:.2f}".format(maxYLowXHigh)+", "+"{:.2f}".format(findYHigh(accXHigh,arm1+arm2+base))+"]\n"
  tempStringData += "\n  Graph Plots:"
  tempStringData += "\n    x: "+str(["{:.2f}".format(x) for x in xPlot3D]).replace("'", "")
  tempStringData += "\n    y: "+str(["{:.2f}".format(y) for y in yPlot]).replace("'", "")
  tempStringData += "\n    z: "+str(["{:.2f}".format(z) for z in zPlot3D]).replace("'", "")
  label1.configure(text=tempStringData)

  #Secod text data
  faceData = "[x1= "+str(facePos[0])+", y2= "+str(facePos[1])+", w= "+str(facePos[2])+", h= "+str(facePos[3])+"]"
  faceData += "\n[x1= "+str(getCenterBox())+"]"
  faceData += "\n[vX= "+str(velocity[0])+", vY= "+str(velocity[1])+", vZ= "+str(velocity[2])+"]"
  faceData += "\nEmotion Detected: "+facialEmotion
  faceData += "\n"+str(noFaceDetected)
  label2.configure(text=faceData)
  label1.after(200,updateText)

def updateGraphs():
  line1.set_xdata(xPlot)
  line1.set_ydata(yPlot)
  fig.canvas.draw()
  fig.canvas.flush_events()

  ax2.clear()
  ax2.plot3D(xPlot3D,zPlot3D,yPlot,markersize=1,color='black',linewidth=2)
  ax2.set_xlim([-1*((arm1*2)+hand),(arm1*2)+hand])
  ax2.set_ylim([-1*((arm1*2)+hand),(arm1*2)+hand])
  ax2.set_zlim([0,(arm1*2)+base])

button = Button(graphFrame, text="Update", command=updateGraphs)
button.place(x=360,y=70)

# -------------------------------------------------------------------------------------------------
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# -------------------------------------------------------------------------------------------------
# END OF UNNEEDED FUNCTIONS


# This flipflop function increases the frame rate of the webcam by limiting the amount of time it calculates the expression the user is showing
# this also gives time to the expression in whatever 3D program we'll be using the time to change expressions
flipflopContainer = Label(container)
getMood = True

def flipflop():
  global getMood
  if not getMood:
    getMood = True
  flipflopContainer.after(5000, flipflop)

#Face cam portion of the GUI
#Getting n-nearest data from the xml file
cascPath = "haarcascade_frontalface_default.xml"
modelPath = "model_35_91_61.h5"
emotion =  ['Anger', 'Disgust', 'Fear', 'Happy', 'Sad', 'Surprise', 'Neutral']
model = keras.models.load_model(modelPath)
faceCascade = cv2.CascadeClassifier(cascPath)

#grabbing the camera
try:
  cam = cv2.VideoCapture(0)
  cam.set(cv2.CAP_PROP_FRAME_WIDTH,cameraRes[0])
  cam.set(cv2.CAP_PROP_FRAME_HEIGHT,cameraRes[1])
except:
  print("Camera was not able to open at start")

# webcamStream tries to run as fast as possible
# hard limit of 10ms so it keeps it under 100fps
# it often runs slower than this anyways
def webcamStream():
  if not cam.isOpened():
    print('Unable to load camera.')
    pass
  try:
    # Capture frame-by-frame
    ret, frame = cam.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
      gray,
      scaleFactor=1.2,
      minNeighbors=10,
      minSize=(30, 30)
    )

    global getMood
    global facePos
    global facialEmotion

    #Error is thrown in camera at faces if it is a null object

    # Draw a rectangle around the faces
    for (x, y, w, h) in faces:
      # Every second get the emotion written on the face via a 48*48 "facial structure" which pulls bounding data and refers to a trained model
      if getMood:
        face_component = gray[y:y+h, x:x+w]
        fc = cv2.resize(face_component, (48, 48))
        inp = np.reshape(fc,(1,48,48,1)).astype(np.float32)
        inp = inp/255.
        prediction = model.predict(inp)
        em = emotion[np.argmax(prediction)]
        facialEmotion = em
        getMood = False

      # Draw the standard rectangle and store facial data in the facePos array
      cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
      if not (x,y,w,h) == (0,0,0,0):
        facePos = (x,y,w,h)
      break

    img = Image.fromarray(frame)
    imgtk = ImageTk.PhotoImage(image=img)
    webcamFrame.imgtk = imgtk
    webcamFrame.configure(image=imgtk)
  except:
    print("Frame dropped @ "+getTime())

  # No Face Detected velocity handler
  # Essentially if you can nolonger see a face the velocity trends towards 0 else it accelerates at maxVelocity*noFaceTime
  try:
    global noFaceDetected
    if not faces:
      if noFaceDetected < noFaceTime:
        noFaceDetected+=1
    else:
      noFaceDetected=0
  #Exception is thrown @ if not faces: perfect time to set noFaceDetected to 0
  except:
    noFaceDetected=0

  facePosHandler()

  webcamFrame.after(10, webcamStream)

#Calling the GUI elements
updateText()
updateGraphs()
flipflop()
#Getting the webcam
try:
  webcamStream()
except:
  Print("Something Went Wrong!")

root.mainloop()
