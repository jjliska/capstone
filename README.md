# AME 486 - Capstone - Reflection  

#### Table of Contents  
&ensp;[Group Members](#Group-Members)  
&ensp;[Project Description](#Project-Description)  
&ensp;[Capstone Video](#Capstone-Video)  
&ensp;[Successes](#Success)  
&ensp;[Improvements](#Improvements)  
&ensp;[Demo Videos](#Demo-Videos)  
&ensp;[Explanations](#Explanations)  
&ensp;[Links](#Links)  
&ensp;[References](#References)  

## Project Description  

## Group Members  
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>  
&ensp;Albert Bang - Unity/Animator  
&ensp;Ivan Mendoza - Programming/Building  
&ensp;Jack Carroll - Sound Designer  
&ensp;[Joshua Liska](https://www.linkedin.com/in/joshua-liska-34a4b77b/) - Programming/Design/Engineering

## Capstone Video  
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>  
&ensp;Not currently uploaded.

## Successes  
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>  

## Improvements  
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>  

## Demo Videos
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>  
### Unity Demo
[![IMAGE ALT TEXT HERE](https://yt-embed.herokuapp.com/embed?v=sWtO3qcnU5k)](https://www.youtube.com/watch?v=sWtO3qcnU5k)
### Tracking Demo
[![IMAGE ALT TEXT HERE](https://yt-embed.herokuapp.com/embed?v=6vG7myi-orQ)](https://www.youtube.com/watch?v=6vG7myi-orQ)
### Smoothing Demo
[![IMAGE ALT TEXT HERE](https://yt-embed.herokuapp.com/embed?v=c3GQ3jPTU7w)](https://www.youtube.com/watch?v=c3GQ3jPTU7w)
### Alpha Demo
[![IMAGE ALT TEXT HERE](https://yt-embed.herokuapp.com/embed?v=l52GL87oeng)](https://www.youtube.com/watch?v=l52GL87oeng)

## Explanations
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>

#### Quick Links for Explanations
&ensp;[End Effector](#End-Effector)  
&ensp;[ML Facial Data to Movement](#ML-Facial-Data-to-Movement)  
&ensp;[Smoothing Algorithm](#Smoothing-Algorithm)  
&ensp;[Hardware](#Hardware)  
&ensp;[Other Explanations](#Other)  

### End Effector
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Explanations/EndEffector.png)

&ensp;Inverse kinematics works via the premise that you know the base is 0,0 and the end effector is at some given point (x,y). Every shape can be then broken into triangles which have a given total angle. You can then determine this angle using the law of tangents and law of cosines. These allow you to determine the angle given arm lengths and positinal data. For example our arm is four degrees of freedom, three in the X and Y plane and one in the Z plane. We use two of the degrees of freedom to extend and retract the arm and a third to stabalize the "hand" and LCD attatched to it. The fourth degree of freedom is used for Z rotation which is a simple trigonometry sin and cos function to calculate where the point is in a 3d space.

<details><summary>Python Script</summary>
<p>
  
```python
def lawOfCosines(a,b,c):
  return np.arccos((a**2+b**2-c**2) / (2.0*a*b))

def distance(x1,y1,x2,y2):
  return np.sqrt((x1-x2)**2+((y1-y2)**2))

def distanceMidArm(angle):
  opp = np.cos(angle) * arm1
  adj = np.sin(angle) * arm1
  return opp, adj

...

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
  
...
  
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
```

&ensp;[From capstoneV11.py](https://github.com/jjliska/capstone/blob/main/capstoneV11_UNDEBUGGED.py)

</p>
</details>

### ML Facial Data to Movement
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Explanations/FacialTracking.png)  

&ensp;The machine learning algorithm detects the face and then sends that data to an algorithm that determines where in the cameras lense the person is. It then determines where it should move to and how quickly given previous movement and the distance from the center of the camera. This then allows the camera to attempt to position itself over the center of the users face. This allows the program to better recognize tthe users emotional state as there is no distortion in the image or partial faces.  
&ensp;Vertical movement in the camera translates to vertical of the arm.  
&ensp;Horizontal movement in the cameras plane translates to rotational movement of the base, a4.  
&ensp;The bounding size of the facial data translates to depth of the arm.  

<details><summary>Python Script</summary>
<p>

```python
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
```

&ensp;[From capstoneV11.py](https://github.com/jjliska/capstone/blob/main/capstoneV11_UNDEBUGGED.py)

</p>
</details>

### Smoothing Algorithm
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Explanations/SmoothingAlgorythms.png)  

&ensp;We use two seperate smoothing algorithm to try and smooth the movement of the arm. The first is run on the python script which uses an acceleration equation to gently accelerate to a top velocity and then once it reaches the bounding box, or facial positioning data is nolonger available, the velocity gently lowers back to zero. This allows the program to create a gentle start and stop for the end effector in which the first several steps will be slow and gently bring the arm up to full speed and then gently lower it back to speed to attempt to lessen damping and "bouncing" on the system. This allows us to achieve much faster speeds without sacrificing the percision of the facial tracking algorithm. Although this is a trade off, if it is too smooth it will not be fast enough, and if it is too fast it will not be smooth enough and begin overshooting and having to compensate.    

<details><summary>Python Script</summary>
<p>

```python
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
```

&ensp;[From capstoneV11.py](https://github.com/jjliska/capstone/blob/main/capstoneV11_UNDEBUGGED.py)

</p>
</details>

&ensp;The second smoothing algorithm is important for several reasons. One is that the python program is not running over ever microsecond to gently smooth the servos angular position to the desired position. Thus we need to feed data for a microcontroller and smooth on that microcontroller. Servos do not have a set speed so instead we can use writeMicroseconds() to get much finer angular translation. We can then take inputs from the python every ~10 milliseconds and smooth it over that given amount of time, recalculation where the angle needs to be at to gently smooth the arm between a given input(a[num]) and the current angle(cura[num]). Typically a servo would instead move to the desired position it was fed(a[num]) as fast as it could while ours attempts to smooth it linerally to closer fit the acceleration model in the python.


<details><summary>C/C++ Script</summary>
<p>

```c
const int delayTime = 100;

//Average time (in ms) between serial.write() on python script
const int timeForMessages = 10000;

//Rate of which the machine is smoothed so i runs every "milisecond" it could run faster
//to create even smoother lines but since its run on parsecs and we only have a degree of
//accuracy of about 1.2 +- some deflection it should be fine 
const float smoothingRate = float(delayTime)/float(timeForMessages);

...

float moveToAngle(float input, float currentAngle, Servo servoName){
  float retFloat;
  float difference = abs(input-currentAngle);
  if(currentAngle > input){
    if((currentAngle - input) > (difference*smoothingRate)){
      retFloat = currentAngle-(difference*smoothingRate);
      writeToServo(servoName,retFloat);
      return retFloat;
    }
    else{
      writeToServo(servoName,input);
      return input;
    }
  }
  else if(currentAngle < input){
    if((currentAngle - input) < (-1*(difference*smoothingRate))){
      retFloat = currentAngle+(difference*smoothingRate);
      writeToServo(servoName,retFloat);
      return retFloat;
    }
    else{
      writeToServo(servoName,input);
      return input;
    }
  }
}
```

&ensp;[From ServoController.ino](https://github.com/jjliska/capstone/blob/main/ServoController/ServoController.ino)

</p>
</details>

### Hardware
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Explanations/Hardware.png)  

&ensp;We use a python script running on a laptop that draws information from a generic USB webcamera. The USB webcam passes visual information into a machine learning script that then creates a generic rectangle over the given persons face given the size and position of the face. This information is then stored and used by the script to effect the end effector. We do this to stabilize the LCD so that the interaction on the display is more stable for the user. The python script wworks by creating a vector model from given initial information along with the facial bounding box. This information includes length, mass, torque, and angular hard limits of the system. The system then determines what the maximum amount of force it can put on its joints before its motors will begin to slip. This information is then used in an inverse kinematic model ([stated in the End Effector](#End-Effector) portion of the explenation) of the robotic arm in whcih the end effector is moved inside of a 3D model to determine what the angles between the arms are. The information is then passed from this python script to a Teensy 3.5 in order to utilized pwm and additional vector smoothing.

### Other
#### Angular Memory in the Event of a Crash

<details><summary>C/C++ Script</summary>
<p>

```c
if(millis()-eepromTimer >= eepromDelay){
  writeStringToEEPROM(0, eepString);
}

String readStringFromEEPROM(int addrOffset){
  int newStrLen = EEPROM.read(addrOffset);
  char data[newStrLen + 1];
  for (int i = 0; i < newStrLen; i++){
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\0';
  return String(data);
}

void writeStringToEEPROM(int addrOffset, const String &strToWrite){
  byte len = strToWrite.length();
  EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++){
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
}

```

&ensp;[From ServoController.ino](https://github.com/jjliska/capstone/blob/main/ServoController/ServoController.ino)

</p>
</details>

&ensp;We attempt to stop the robot from seriously damaging itself in the event that the script throws an error. This saves the angular information every several seconds so that, if the program is restarted it will then attempt to rehome to a set location from the stored angular data.


## Links
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>  
&ensp;- A collab with some of the basic inverse kinematic models we used is avialable here: [Google Colab](https://colab.research.google.com/drive/112x_Fhu4YKPZFFK7a1mo7FeFkNSDPJKg?usp=sharing)  
&ensp;- A link to all of the 3D printed parts we designed and printed: [3D Prints](https://github.com/jjliska/capstone/tree/main/3D-Prints)  
&ensp;- The Fusion 360 Model created: [Fusion 360](https://a360.co/3mX6sYx) &ensp;&ensp; *Note it is a large file so it will take a while to render on a browser.*

## References
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>
#### &ensp;[Ender 3 Cable Chain](https://www.thingiverse.com/thing:2920060)
#### &ensp;[Emotion Model](https://drive.google.com/file/d/1192YC8mYKaCbCoACP8hTfr9PCMC2iN30/view?usp=sharing) from [jaydeepthik](https://github.com/jaydeepthik)
#### &ensp;[Face Detection](https://realpython.com/face-detection-in-python-using-a-webcam/)
#### &ensp;[Two-way communication between Python 3 and Unity (C#) - Y. T. Elashry](https://github.com/Siliconifier/Python-Unity-Socket-Communication.git)
#### &ensp;[Writing to EEPROM](https://roboticsbackend.com/arduino-write-string-in-eeprom/)
