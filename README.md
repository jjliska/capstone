# AME 486 - Capstone - Reflection

##### Table of Contents  
&ensp;[Group Members](#Group-Members)  
&ensp;[Project Description](#Project-Description)  
&ensp;[Capstone Video](#Capstone-Video)
&ensp;[Demo Videos](#Demo-Videos)
&ensp;[Coding Explained](#Coding-Explained)
&ensp;[Fusion 360 Models](#Fusion-360-Models)  

## Project Description

## Group Members
&ensp;- Albert Bang - Unity/Animator  
&ensp;- Ivan Mendoza - Programming/Building  
&ensp;- Jack Carroll - Sound Designer  
&ensp;- [Joshua Liska](https://www.linkedin.com/in/joshua-liska-34a4b77b/) - Programming/Design/Engineering

## Capstone Video

## Demo Videos
### Unity Demo
[![alt text](https://img.youtube.com/vi/sWtO3qcnU5k/0.jpg)](https://www.youtube.com/watch?v=sWtO3qcnU5k)
### Tracking Demo
[![alt text](https://img.youtube.com/vi/6vG7myi-orQ/0.jpg)](https://www.youtube.com/watch?v=6vG7myi-orQ)
### Smoothing Demo
[![alt text](https://img.youtube.com/vi/c3GQ3jPTU7w/0.jpg)](https://www.youtube.com/watch?v=c3GQ3jPTU7w)
### Alpha Demo
[![alt text](https://img.youtube.com/vi/l52GL87oeng/0.jpg)](https://www.youtube.com/watch?v=l52GL87oeng)

## Coding Explained
##### Quick Links to Coding Explained
&ensp;[Hardware](#Hardware)  
&ensp;[ML Facial Data to Movement](#ML-Facial-Data-to-Movement)  
&ensp;[Capstone Video](#Smoothing-Algorithm)


### Hardware
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Hardware.png)  
### ML Facial Data to Movement
![alt text](https://github.com/jjliska/capstone/blob/main/Media/FacialTracking.png)  
### Smoothing Algorithm
![alt text](https://github.com/jjliska/capstone/blob/main/Media/SmoothingAlgorythms.png)  
&ensp;We use two seperate smoothing algorithm to try and smooth the movement of the arm. The first is run on the python script which uses an acceleration equation to gently accelerate to a top velocity and then once it reaches the bounding box, or facial positioning data is nolonger available, the velocity gently lowers back to zero.

## Fusion 360 Models
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Reflections2v63.png)
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Reflections2v63_1.png)
![alt text](https://github.com/jjliska/capstone/blob/main/Media/Reflections2v63_2.png)

## References:
#### [Ender 3 Cable Chain](https://www.thingiverse.com/thing:2920060)
#### [Emotion Model](https://drive.google.com/file/d/1192YC8mYKaCbCoACP8hTfr9PCMC2iN30/view?usp=sharing) from [jaydeepthik](https://github.com/jaydeepthik)
#### [Face Detection](https://realpython.com/face-detection-in-python-using-a-webcam/)
#### [Two-way communication between Python 3 and Unity (C#) - Y. T. Elashry](https://github.com/Siliconifier/Python-Unity-Socket-Communication.git)
