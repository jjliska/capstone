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

#COMPUTATION
#---------------------------------------------------------------------------------------------------------
#These variables will he changed by the serial information
# x is changed exclusively by the size of the box
# y is changed by the location of the box vertically
# z is changed by the placement of the box horizontally
x = 10
y = 20
zRotation = 0

degreesPerStep = .18
numberOfStepsTotal = 1000

#CONSTANTS <<LEAVE UNTOUCHED>>
arm1 = 20.0
arm2 = 20.0
hand = 4.0
base = 4.0
screenSize = 30.0

#All masses assume the entirety of the mass is at the end
#of the point allowing for a larger "buffer" and less torque on the system
#Units: kg
arm1Mass = .25+.06
arm2Mass = .25+.06
handMass = 1
totMass = arm1Mass + arm2Mass + handMass

#Gear ratio
gear1 = 30
gear2 = 20
gearRatio = gear1/gear2

#Units: kg/cm
a1Torque = 35*gearRatio
a2Torque = 25*gearRatio
a3Torque = 20*gearRatio

#MaxAcceptableAngles
a1MaxAngle = [0,180]
#if x is negative the angle must remain [90,180]
a2MaxAnglePos = [45,180]
a2MaxAngleNeg = [45,90]
a3MaxAngle = [90,270]
a4MaxAngle = [0,360]

#User Defined Maximum Degree of freedom box
x1Max, y1Max = (0,0)
x2Max, y2Max = (0,0)

facePos = [0,0,0,0]

#Variable for if a face isnt detected for a period of time then 
noFaceDetected = 0
noFaceTime = 10

#Takes off 15% to ensure the system is not overloading
def acceptableRange(input):
  return input*0.85

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

maxYLowXLow = (screenSize/2.0)+base
maxYLowXHigh = base

#End of startup variables

#MATH EQUATIONS FOR HANDLING LATER
#------------------------------------------------------------------------------
def findYHigh(x,radius):
  return np.sqrt(radius**2-x**2)

def maxYCircle(ox,oy,x,y,radius):
  return np.sqrt((x-ox)**2+(y-oy)**2)

#Calculates variables neccessary for the function during startup

def angleChecker(array,angle):
  if angle >= array[0] and angle <= array[1]:
    return True
  else:
    return False

def angleHandler(angle1,angle2,angle3,angle4,x):
  if angleChecker(a1MaxAngle,angle1) and angleChecker(a3MaxAngle,angle3) and angleChecker(a4MaxAngle,angle4) :
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

def userInputCheck():
  if positionHandler(x1Max,y1Max) and positionHanlder(x2Max,y2Max):
    return True
  else:
    return False

def positionHandler(x,y):
  if x >= accXLow and x <= accXHigh:
    if x >= base:
      if y >= maxYHighXHigh and y <= findYHigh(accXHigh,arm1+arm2+base):
        return True
      else:
        return False
    else:
      if y >= maxYLowXLow and y <= findYHigh(accXLow,arm1+arm2+base):
        return True
      else:
        return False
  else:
    return False

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

#This filter may need to be changed to accomodate the microsteps we use to smooth the actuation of the servos/steppers
def filterAngles(a1,a2,a3,a4):
  a1Filt = int(np.trunc(a1))
  a2Filt = int(np.trunc(a2))
  rem = (a1-a1Filt)+(a2-a2Filt)
  a3Filt = round(a3+rem)
  a4Filt = round(a4)
  return a1Filt, a2Filt, a3Filt, a4Filt

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

a1Filt, a2Filt, a3Filt, a4Filt = filterAngles(np.degrees(a1),np.degrees(a2),np.degrees(a3),np.degrees(a4))

xPlot3D = [rotateX(0,x,np.radians(a4Filt)) for x in xPlot]
zPlot3D = [rotateZ(0,x,np.radians(a4Filt)) for x in xPlot]

def updateVariables(x,y,zRot):
  if positionHandler(x,y):
    #a1 is between base and arm1
    #a2 is between arm1 and arm2
    a1temp, a2temp = angles(x,y)
    mxtemp, mytemp = distanceMidArm(a1temp)
    disttemp = distance(mxtemp,mytemp,x+hand,y)
    #a3 is between arm2 and hand
    a3temp = lawOfCosines(arm2,hand,disttemp)
    #a4 is z rotation
    a4temp = np.radians(zRot)

    xPlottemp = [0,0,mxtemp,x,x+hand]
    yPlottemp = [0,base,mytemp+base,y+base,y+base]
    if angleHandler(a1temp,a2temp,a3temp,a4temp,x):
      global a1,a2,a3,a4
      a1,a2,a3,a4 = a1temp,a2temp,a3temp,a4temp
      global mx,my
      mx,my = mxtemp,mytemp
      global dist
      dist = disttemp
      global xPlot,yPlot
      xPlot,yPlot = xPlottemp,yPlottemp
      global a1Filt,a2Filt,a3Filt,a4Filt
      a1Filt, a2Filt, a3Filt, a4Filt = filterAngles(np.degrees(a1),np.degrees(a2),np.degrees(a3),np.degrees(a4))
      global xPlot3D
      xPlot3D = [rotateX(0,x,a4) for x in xPlot]
      global zPlot3D
      zPlot3D = [rotateZ(0,x,a4) for x in xPlot]
  else:
    print("Cannot move ("+x+","+y+") is outside the bounds.")

#CAMERA POSITION HANDLEING
#-----------------------------------------------------------------------------------------------------------
#Get the center of the camera
# x is "zoom"
# y is "tilt"
# z is "pan"
cameraRes = [640,480]
centerScreen=[cameraRes[0]-(cameraRes[0]/2),cameraRes[1]-(cameraRes[1]/2)]

#Max velocity allowed in any direction
#Later this will be passed as a percetage so the system does not exceeded this
maxVelocity = 1.0
acceleration = 0.1
maxVelReached = [False,False,False]
#X,Y,Z
systemVelocity = [0.0,0.0,0.0]

def getCenterBox():
  return (facePos[0]+(facePos[2]/2)-(cameraRes[0]/2)),(facePos[1]+(facePos[3]/2)-(cameraRes[1]/2))

boundsRight = cameraRes[0]*2/5
boundsLeft = cameraRes[0]*3/5
boundsTop = cameraRes[1]*2/5
boundsBottom = cameraRes[1]*3/5

def handleCamMovement():
  if not facePos == (0,0,0,0):
    facePosHandler()
    
def facePosHandler():
  # x axis of camera is z axis of robot
  # y axis of cameria is y axis of robot
  z1C, y1C = getCenterBox()
  if z1C <= boundsRight:
    accelerateMovement(2,True,-1)
  elif z1C >= boundsLeft:
    accelerateMovement(2,True,1)
  elif z1C <= innerBoundsRight:
    accelerateMovement(2,False,-1)
  elif z1C >= innerBoundsLeft:
    accelerateMovement(2,False,1)
  else:
    pass

#Variable = variable to be changed
#Bounded indicates if the object is bounded
#posNeg = positive or negative
def accelerateMovement(variable,bounded,posNeg):
  global systemVelocity,maxVelReached
  accel = acceleration*posNeg
  maxVel = maxVelBounded(variable,posNeg)
  if noFaceDetected >= noFaceTime:
    if bounded:
      if abs(systemVelocity[variable]) <= maxVelocity:
        systemVelocity[variable] += acceleration*posNeg
      else:
        systemVelocity[variable] = maxVelocity*posNeg
        maxVelReached[variable] = True
    else:
      if systemVelocity[variable] >= maxVel:
        systemVelocity[variable] = maxVel
      else:
        systemVelocity[variable] += acceleration*posNeg
        if systemVelocity[variable] >= maxVel:
          systemVelocity[variable] = maxVel
  else:
    if not systemVelocity[variable] == 0:
      systemVelocity[variable] += acceleration*posNeg*-1
    

def maxVelBounded(variable,posNeg):
  cx,cy = centerScreen
  x1,y1 = innerBoundsRight, innerBoundsLeft
  x2,y2 = getCenterBox()
  if variable == 2:
    distCen = abs(x2-cx)
    dist = x1-cx
    maxVel = (abs(abs(dist)-distCen)/abs(boundsRight-innerBoundsRight))*maxVelocity*posNeg
    print (abs(dist),distCen,maxVel)
  print(maxVel)
  return maxVel

  
    
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

webcamFrame = Label(container,text="Temp",width=cameraRes[0],height=cameraRes[1])
webcamFrame.pack(side=TOP,padx=12,pady=12)
textFrame2 = Frame(container)
textFrame2.pack(side=LEFT)
label2 = Label(textFrame2,text="Temp",justify=LEFT,bg="darkgrey")
label2.pack(side=TOP)

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

def getTime():
  now = datetime.now()
  return now.strftime("%H:%M:%S")
  
def updateText():
  tempStringData = "----------------    "+getTime()+"    ----------------\n"
  
  #Text data part 1
  #All of this is unneccessary and simply for visual graphs and data outputs to verify the program is properly working
  tempStringData += "\n  Point of arm: ("+"{:.2f}".format(mx)+", "+"{:.2f}".format(my)+")"

  tempStringData += "\n\n  Verifying distance:\n    Arm 1: "+"{:.2f}".format(distance(0,0,mx,my))+"\n    Arm 2: "+"{:.2f}".format(distance(mx,my,x,y))

  tempStringData += "\n\n  Unfiltered Angles:\n    Angle 1: "+"{:.2f}".format(np.degrees(a1))+"\n    Angle 2: "+"{:.2f}".format(np.degrees(a2))+"\n    Angle 3: "+"{:.2f}".format(np.degrees(a3))+"\n    Angle 4: "+"{:.2f}".format(np.degrees(a4))

  tempStringData += "\n\n  Filtered Angles:\n    Angle 1: "+"{:.2f}".format(a1Filt)+"\n    Angle 2: "+"{:.2f}".format(a2Filt)+"\n    Angle 3: "+ "{:.2f}".format(a3Filt)+"\n    Angle 4: "+"{:.2f}".format(a4Filt)

  #tempStringData += "\n\n  Max length armatures:\n    @ a1: "+"{:.2f}".format(mLengthA1)+"\n    @ a2: "+"{:.2f}".format(mLengthA2)+"\n    @ a3: "+"{:.2f}".format(mLengthA3)

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
  faceData += "\n[vX= "+str(systemVelocity[0])+", vY= "+str(systemVelocity[1])+", vZ= "+str(systemVelocity[2])+"]"
  faceData += "\n"+str(noFaceDetected)
  label2.configure(text=faceData)
  label1.after(100,updateText)

def updateGraphs():
  thread = Thread(target = threaded_graph, args = (10,))
  thread.start()
  thread.join()
  graphFrame.after(50, updateGraphs)
  
#This has a "memory" missuse, however the alternative animation doesn't seem to want to co-operate so it will work at decent frames
#I can also thread the camera if the latency is too big of an issue  
def threaded_graph(args):
  plt.close()
  #Line Graph
  line1.set_xdata(xPlot)
  line1.set_ydata(yPlot)
  
  #3d Graph
  ax2.plot3D(xPlot3D,zPlot3D,yPlot, color='black',linewidth=2)

#Face cam portion of the GUI
#Getting n-nearest data from the xml file
cascPath = "haarcascade_frontalface_default.xml"
faceCascade = cv2.CascadeClassifier(cascPath)
#grabbing the camera
try:
  cam = cv2.VideoCapture(0)
except:
  print("Camera was not able to open at start")

#webcamStream tries to run as fast as possible
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
    
    global facePos
    
    #Error is thrown in camera at faces if it is a null object
    try:
      global noFaceDetected
      if not faces:
        if noFaceDetected < noFaceTime:
          noFaceDetected+=1
      else:
        noFaceDetected=0
    except:
    #Exception is thrown @ if not faces: perfect time to set noFaceDetected to 0
      noFaceDetected=0
      
    # Draw a rectangle around the faces
    for (x, y, w, h) in faces:
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
    
  handleCamMovement()
    
  webcamFrame.after(10, webcamStream)

#Pulling info from this to the second script
def pullData():
  return 

#Calling the GUI elements
updateText()
updateGraphs()
#Getting the webcam
webcamStream()
root.mainloop()
