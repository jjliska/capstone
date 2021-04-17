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

### End Effector
![alt text](https://github.com/jjliska/capstone/blob/main/Media/EndEffector.png)

    &ensp;Inverse kinematics works via the premise that you know the base is 0,0 and the end effector is at some given point (x,y). If you push the x,y to a specific axis you can then move the arms by invoking arctan and arccos, which get a theta given arm length or a given point will determine the angle. Since the end effector is at the knuckle between the second and third arm there is no need to invoke inverse kinematics again, as the third arm needs to be parallel to the floor to give the viewer the most viewable experience.
### ML Facial Data to Movement
![alt text](https://github.com/jjliska/capstone/blob/main/Media/FacialTracking.png)  
          
    &ensp;The machine learning algorithm detects the face and then sends that data to a check that determines where in the cameras vision the face was detected. This data is then enterpolated into a system that determines how much the and how quickly the arm end effector of the arm needs to move to try and center the face in its grid.
### Smoothing Algorithm
![alt text](https://github.com/jjliska/capstone/blob/main/Media/SmoothingAlgorythms.png)  
       
    &ensp;We use two seperate smoothing algorithm to try and smooth the movement of the arm. The first is run on the python script which uses an acceleration equation to gently accelerate to a top velocity and then once it reaches the bounding box, or facial positioning data is nolonger available, the velocity gently lowers back to zero.
### Hardware
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Hardware.png)  
    
    &ensp;We use a python script running on a laptop that draws information from a generic USB webcamera. The USB webcam passes visual information into a machine learning script that then creates a generic rectangle over the given persons face given the size and position of the face. This information is then stored and used by the script to effect the end effector. We do this to stabilize the LCD so that the interaction on the display is more stable for the user. The python script wworks by creating a vector model from given initial information along with the facial bounding box. This information includes length, mass, torque, and angular hard limits of the system. The system then determines what the maximum amount of force it can put on its joints before its motors will begin to slip. This information is then used in an inverse kinematic model of the robotic arm in whcih the end effector is moved inside of a 3D model to determine what the angles between the arms are. This angular information is then passed to the robots motors to replicate this 3D model. The information is passed from this python script to a Teensy 3.5 in order to utilized pwm and additional vector smoothing.

## Links
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>  

## References:
&ensp;<sup>[Back to Top](#AME-486---Capstone---Reflection)</sup>
#### &ensp;[Ender 3 Cable Chain](https://www.thingiverse.com/thing:2920060)
#### &ensp;[Emotion Model](https://drive.google.com/file/d/1192YC8mYKaCbCoACP8hTfr9PCMC2iN30/view?usp=sharing) from [jaydeepthik](https://github.com/jaydeepthik)
#### &ensp;[Face Detection](https://realpython.com/face-detection-in-python-using-a-webcam/)
#### &ensp;[Two-way communication between Python 3 and Unity (C#) - Y. T. Elashry](https://github.com/Siliconifier/Python-Unity-Socket-Communication.git)
