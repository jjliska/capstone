import numpy as np
from mpl_toolkits.mplot3d import Axes3D 
import matplotlib.pyplot as plt 
from tkinter import *
from matplotlib.figure import Figure 
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import cv2
from PIL import Image, ImageTk
from datetime import datetime
from threading import Thread

#COMPUTATION
#---------------------------------------------------------------------------------------------------------
#These variables will he changed by the serial information
# x is changed exclusively by the size of the box
# y is changed by the location of the box vertically
# z is changed by the placement of the box horizontally
x = 10
y = 20
zRotation = 90

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

errorStatement = ""

#Takes off 15% to ensure the system is not overloading
def acceptableRange(input):
  return input*0.85

#Startup variables that need calculations
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
    a3temp = lawOfCosines(arm2,hand,dist)
    #a4 is z rotation
    a4temp = np.radians(zRot)

    xPlottemp = [0,0,mx,x,x+hand]
    yPlottemp = [0,base,my+base,y+base,y+base]
    if angleHandler(a1temp,a2temp,a3temp,a4temp,x):
      a1,a2,a3,a4 = a1temp,a2temp,a3temp,a4temp
      mx,my = mxtemp,mytemp
      dist = disttemp
      xPlot,yPlot = xPlottemp,yPlottemp
      a1Filt, a2Filt, a3Filt, a4Filt = filterAngles(np.degrees(a1),np.degrees(a2),np.degrees(a3),np.degrees(a4))
      xPlot3D = [rotateX(0,x,a4) for x in xPlot]
      zPlot3D = [rotateZ(0,x,a4) for x in xPlot]
  else:
    print("Cannot move ("+x+","+y+") is outside the bounds.")
    
#GUI ELEMENTS
#------------------------------------------------------------------------------------------------------------
#Instancing the gui frame
root = Tk()
root.title("Capstone")
root.geometry("1920x1080")

#Text frame for the label
container = Frame(root)
container.pack()

textFrame = Frame(container)
textFrame.pack(side=LEFT)

label1 = Label(textFrame, text="Temp", justify=LEFT,bg="white",width=30,height=100)
label1.pack(fill=BOTH)

graphFrame = Frame(container)
graphFrame.pack(side=LEFT)

webcamFrame = Label(container,width=600,height=400)
webcamFrame.pack(side=TOP,padx=20,pady=20)

fig = plt.figure(figsize=(4,12),dpi=100)
ax = fig.add_subplot(2,1,1)
line1, = ax.plot([0,0,0,0,0],[0,0,0,0,0],'r-',markersize=4,marker='o',color='black',linestyle='solid',linewidth=2,markerfacecolor='red')
ax.set_xlim([-1*((arm1*2)+hand),(arm1*2)+hand])
ax.set_ylim([0,(arm1*2)+base])

ax2 = fig.add_subplot(2,1,2,projection='3d')
line3D = ax2.plot3D(xPlot3D,zPlot3D,yPlot, markersize=2,color='black',linewidth=2)
ax2.set_xlim([-1*((arm1*2)+hand),(arm1*2)+hand])
ax2.set_ylim([-1*((arm1*2)+hand),(arm1*2)+hand])
ax2.set_zlim([0,(arm1*2)+base])

graph = FigureCanvasTkAgg(fig, graphFrame)
graph.get_tk_widget().pack()

def getTime():
  now = datetime.now()
  return now.strftime("%H:%M:%S")
  
def updateText():
  now = datetime.now()
  tempStringData = "  "+getTime()+"\n---------------------------------"
  #All of this is unneccessary and simply for visual graphs and data outputs to verify the program is properly working
  tempStringData += "\n  Point of arm: ("+"{:.2f}".format(mx)+","+"{:.2f}".format(my)+")"

  tempStringData += "\n\n  Verifying distance:\n    Arm 1: "+"{:.2f}".format(distance(0,0,mx,my))+"\n    Arm 2: "+"{:.2f}".format(distance(mx,my,x,y))

  tempStringData += "\n\n  Unfiltered Angles:\n    Angle 1: "+"{:.2f}".format(np.degrees(a1))+"\n    Angle 2: "+"{:.2f}".format(np.degrees(a2))+"\n    Angle 3: "+"{:.2f}".format(np.degrees(a3))+"\n    Angle 4: "+"{:.2f}".format(np.degrees(a4))

  tempStringData += "\n\n  Filtered Angles:\n    Angle 1: "+"{:.2f}".format(a1Filt)+"\n    Angle 2: "+"{:.2f}".format(a2Filt)+"\n    Angle 3: "+ "{:.2f}".format(a3Filt)+"\n    Angle 4: "+"{:.2f}".format(a4Filt)

  tempStringData += "\n\n  Maximum length given torque:\n    @ a1: "+"{:.2f}".format(mLengthA1)+"\n    @ a2: "+"{:.2f}".format(mLengthA2)+"\n    @ a3: "+"{:.2f}".format(mLengthA3)

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
  print(facePos)
  label1.after(50,updateText)

def updateGraphs():
  thread = Thread(target = threaded_graph, args = (10,))
  thread.start()
  thread.join()
  graphFrame.after(50, updateGraphs)
  
#This has a "memory" missuse it overuses, however the alternative animation doesn't seem to want to co-operate so it will work at decent frames
#I can also thread the camera if the latency is too big an issue  
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
except ValueError:
  errorStatement.append("\nCamera was not able to open")

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

    # Draw a rectangle around the faces
    for (x, y, w, h) in faces:
      cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
      global facePos
      facePos = (x,y,w,h)
      break
        
    img = Image.fromarray(frame)
    imgtk = ImageTk.PhotoImage(image=img)
    webcamFrame.imgtk = imgtk
    webcamFrame.configure(image=imgtk)
  except ValueError:
    errorStatement.append("\nFrame dropped @ "+getTime())
  webcamFrame.after(10, webcamStream)

#Calling the GUI elements
updateText()
updateGraphs()
#Getting the webcam
webcamStream()
root.mainloop()
