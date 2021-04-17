# AME 486 - Capstone - Reflection

##### Table of Contents  
&ensp;[Group Members](#Group-Members)  
&ensp;[Project Description](#Project-Description)  
&ensp;[Capstone Video](#Capstone-Video)  
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

## Demo Videos
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>  
### Unity Demo
[![alt text](https://img.youtube.com/vi/sWtO3qcnU5k/0.jpg)](https://www.youtube.com/watch?v=sWtO3qcnU5k)
### Tracking Demo
[![alt text](https://img.youtube.com/vi/6vG7myi-orQ/0.jpg)](https://www.youtube.com/watch?v=6vG7myi-orQ)
### Smoothing Demo
[![alt text](https://img.youtube.com/vi/c3GQ3jPTU7w/0.jpg)](https://www.youtube.com/watch?v=c3GQ3jPTU7w)
### Alpha Demo
[![alt text](https://img.youtube.com/vi/l52GL87oeng/0.jpg)](https://www.youtube.com/watch?v=l52GL87oeng)

## Explanations
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>

##### Quick Links for Explanations
&ensp;[End Effector](#End-Effector)  
&ensp;[ML Facial Data to Movement](#ML-Facial-Data-to-Movement)  
&ensp;[Smoothing Algorithm](#Smoothing-Algorithm)  
&ensp;[Hardware](#Hardware)  
&ensp;[Other Explanations](#Other)  

### End Effector
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Explanations/EndEffector.png)

&ensp;Inverse kinematics works via the premise that you know the base is 0,0 and the end effector is at some given point (x,y). Every shape can be then broken into triangles which have a given total angle. You can then determine this angle using the law of tangents and law of cosines. These allow you to determine the angle given arm lengths and positinal data. For example our arm is four degrees of freedom, three in the X and Y plane and one in the Z plane. We use two of the degrees of freedom to extend and retract the arm and a third to stabalize the "hand" and LCD attatched to it. The fourth degree of freedom is used for Z rotation which is a simple sin and cos to determine where the point is in a 3d space. We do this via: 
```python
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

### ML Facial Data to Movement
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Explanations/FacialTracking.png)  

&ensp;The machine learning algorithm detects the face and then sends that data to an algorithm that determines where in the cameras lense the person is. It then determines where it should move to and how quickly given previous movement and the distance from the center of the camera. This then allows the camera to attempt to position itself over the center of the users face. This allows the program to better recognize tthe users emotional state as there is no distortion in the image or partial faces.  
&ensp;Vertical movement in the camera translates to vertical of the arm.  
&ensp;Horizontal movement in the cameras plane translates to rotational movement of the base, a4.  
&ensp;The bounding size of the facial data translates to depth of the arm.  

### Smoothing Algorithm
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Explanations/SmoothingAlgorythms.png)  

&ensp;We use two seperate smoothing algorithm to try and smooth the movement of the arm. The first is run on the python script which uses an acceleration equation to gently accelerate to a top velocity and then once it reaches the bounding box, or facial positioning data is nolonger available, the velocity gently lowers back to zero. This allows the program to create a gentle start and stop for the end effector in which the first several steps will be slow and gently bring the arm up to full speed and then gently lower it back to speed to attempt to lessen damping and "bouncing" on the system. This allows us to achieve much faster speeds without sacrificing the percision of the facial tracking algorithm. Although this is a trade off, if it is too smooth it will not be fast enough, and if it is too fast it will not be smooth enough and begin overshooting and having to compensate.    
  
&ensp;The second smoothing algorithm is important for several reasons. One is that the python program is not running over ever microsecond to gently smooth the servos angular position to the desired position. Thus we need to feed data for a microcontroller and smooth on that microcontroller. Servos do not have a set speed so instead we can use writeMicroseconds() to get much finer angular translation. We can then take inputs from the python every ~10 milliseconds and smooth it over that given amount of time, recalculation where the angle needs to be at to gently smooth the arm between a given input(a[num]) and the current angle(cura[num]). Typically a servo would instead move to the desired position it was fed(a[num]) as fast as it could while ours attempts to smooth it linerally to closer fit the acceleration model in the python.

### Hardware
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Explanations/Hardware.png)  

&ensp;We use a python script running on a laptop that draws information from a generic USB webcamera. The USB webcam passes visual information into a machine learning script that then creates a generic rectangle over the given persons face given the size and position of the face. This information is then stored and used by the script to effect the end effector. We do this to stabilize the LCD so that the interaction on the display is more stable for the user. The python script wworks by creating a vector model from given initial information along with the facial bounding box. This information includes length, mass, torque, and angular hard limits of the system. The system then determines what the maximum amount of force it can put on its joints before its motors will begin to slip. This information is then used in an inverse kinematic model ([stated in the End Effector](#End-Effector) portion of the explenation) of the robotic arm in whcih the end effector is moved inside of a 3D model to determine what the angles between the arms are. The information is then passed from this python script to a Teensy 3.5 in order to utilized pwm and additional vector smoothing.

### Other


## Links
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>  

## References:
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>
#### &ensp;[Ender 3 Cable Chain](https://www.thingiverse.com/thing:2920060)
#### &ensp;[Emotion Model](https://drive.google.com/file/d/1192YC8mYKaCbCoACP8hTfr9PCMC2iN30/view?usp=sharing) from [jaydeepthik](https://github.com/jaydeepthik)
#### &ensp;[Face Detection](https://realpython.com/face-detection-in-python-using-a-webcam/)
#### &ensp;[Two-way communication between Python 3 and Unity (C#) - Y. T. Elashry](https://github.com/Siliconifier/Python-Unity-Socket-Communication.git)
