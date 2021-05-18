# Claw-Rover-K3
Project of a programmed and simulated robot that drives autonomously and collects garbage.
<p align="center"><img src="/Design/Images/Robot.png" width="300"/></p>

# Table of Contents
   * [What is this?](#What-is-this)
   * [Description](#Description)<!-- [Amazing contributions](#Amazing-contributions)-->
   * [Hardware Scheme](#Hardware-Scheme)   
   * [3D pieces](#3D-pieces)
   * [Software Architecture](#Software-Architecture)
   * [Modules](#modules)
      <!--* [Brisca AI](#Brisca-AI)
      * [Card Detection with computer vision](#Card-Detection-with-computer-vision)
      * [Inverse kinematics algorithm visualizer](#Inverse-kinematics-algorithm-visualizer)
      * [Voice Recognition](#voice-recognition)
      * [Videogame Simulation](#videogame-simulation)-->
   * [Video](#video)
   * [Authors](#authors)



# What is this?
We are a group of four Computer Engineering students. More specifically, we specialize in the branch of Computation. 
This project integrates knowledge that we have acquired throughout the degree, mixing disciplines such as Robotics, Computer Vision, Deep Learning, Artificial Intelligence, Mathematics...
The result of this work is in this repository, in the form of a self-driving robot which follows the road, while detecting any trash along the way. It collects all the trash it detects, effectively acting as an autonomous garbage truck.

The Claw Rover K3 (CRK3) project is based on two very clear motivations. First, we wanted to create a robot committed to the environment, capable of making the world a slightly cleaner planet. Secondly, we were very fascinated by autonomous driving systems, such as those of Tesla, and we wanted to take advantage of this project to study them in depth, to implement one satisfactorily and to be able to play and experiment with it.
From the union of these two great motivations, and from the ideas and the effort of the four members of the group, Claw Rover K3 was born 

# Description
In this project you will find the code, simulations and 3D designs of the parts of a robot developed from scratch, Claw-Rover-K3, which is able to drive autonomously and collect garbage previously detected using computer vision techniques. This robot consists, schematically, of an anthropomorphic arm located on a platform with four wheels (front-wheel drive). It is implemented with Coppelia simulator and, in case you want to carry it out in real life, the instructions dedicated to the movement of the simulation robot should be substituted by communication instructions from a Raspberry Pi board to an Arduino board. This is a necessary step in order to correctly interact with the hardware components of the robot, such as the wheels, the arm or the sensors.

Basically, the robot is able to, through a camera sensor, detect the road and adjust its speed, direction and position based on a PID controller. In addition, through the use of a neural network, you have the option of enabling traffic sign recognition, which is used to influence the robot's driving and influence its trajectory. While it is self-driving, it analyses each of the processed frames in order to find garbage and collect it. When it detects trash, the robot reduces its speed, stops to a halt, and using the same camera that is used to drive, it detects the exact position of the object, using inverse kinematics equations. Afterwards, it will use its claw to pick up the object and deposit it in the trash container.

<!--
# Amazing contributions


The three most important contributions in which our robot stands up are:<img src="https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/imgs/braç.png" align="right" width="150" alt="header pic"/>
- Entertainment for seniors:  it is designed to entertain the elderly, for whom robots are a whole new thing.
- Classic Game Automation, the brisca: we give life to a classic and mythical game like the brisca combining it with technology and having it be even more enjoyable.
- Voice recognition with human interaction: Designed to bring the user closer to the robot and allows them to communicate with it.
-->

# Hardware Scheme
This is the Hardware Scheme we planned for this project, within the 100€ budget. 

<p align="center"><img src="Design/Images/Hardware Scheme.png" width="400"/></p>

<a href="https://github.com/guiuomsfont/Claw-Rover-K3/blob/main/Design/Hardware%20Components.txt">Here</a> is the list of hardware components with purchase links.


# 3D Pieces
<!--
In order to recreate the physical model of the robot, we had to design some of its parts as models to print with a 3D printer the university gave us access to. This are the models needed.  

<img src="https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/imgs/cam.png" width="100" align="center"/>
<img src="https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/imgs/xeringa.png" width="250" align="center"/>
<img src="https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/imgs/girador.png" width="200" align="center"/>
<img src="https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/imgs/deck.png" width="250" align="center"/>
<img src="https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/imgs/pila.png" width="200" align="center"/>




Files are avaliable under [stl](https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/stl/). Of course they could be replaced by any other model or created with other methods rather than 3D printing.
-->


# Software Architecture

<!--
In order to develop the idea we had, we must divide the software architecture in different modules. First to make them work separately and then be able to put them all together as one whole project. The modules are:
* Computer Vision module: card recognition
* Brisca AI 1vs1
* Inverse Kinematics
* Voice Recognition
* Controller: communication of all the above modules.

Initially all of them should have worked together, but after the project's objectives changed we decided to do different simulations in order to reproduce the functionality we were aiming for. These are:
* Computer Vision module: card recognition (as an independent simulation)
* Inverse Kinematics Simulation: not only doing the math but also visualizing it.
* Fully functional 3D game: This simulation involves 3D models, animation and game development to have a fully inmersive experience and getting the closest image to what the project was going to look like.

![2](https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/imgs/DiagramaModuls.png)


## Modules

### Brisca AI
Requirements for the algorithm: python 3

The first module is the AI that drives the game flow, it's the one who decides which card to choose from those on _C.A.R.L.E.S._' hand, based on what a human player could see, and more. It is explained in more detail in the [report](https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/reports/RLP_SPRINT_5%20-%20Final%20Report.pdf)), both it's python version for the actual raspberry pi-driven robot and the rework we made for the Unity videogame.

### Card Detection with computer vision
We made a program based on computer vision able to detect the number and suit of a card with any rotation and different backgrounds and illuminations. Here we show a part of the process: 

Requirements: Python 3, and its libraries numpy, cv2, imutils, math and scipy.

![2](https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/gif/modulVisio.gif)


### Inverse kinematics algorithm + visualizer

This algorithm built from scratch is based on the geometric inverse kinematics method for calculating arm degrees from coordinates, and the smooth movement between two points is calculated using a continuous rectiliniar trajectory.

The visualizer takes an imput of an x,y,z position inside the workspace and shows an animation of _C.A.R.L.E.S._' arm doing the designated trajectory. As an example, this is the animation it'd play as a celebration when winning the game:

![2](https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/gif/ik.gif)

Requirements for the algorithm: Python 3 and the numpy and maths libraries. For visualization, matplotlib and scipy are also needed.


### Voice recognition

This module is based on google's speech recognition API, and it's used to analize the human opponent's orders, and guess which of the possible actions the user is requesting. This is sent to the main controller, which will send the information to the AI module if necessary.

Requirements: Python 3 and its libraries google-cloud-speech, google-auth-oauthlib, sounddevice and soundfile.
-->

# Video
<!--
Short video showing all the functionalities of the project.

[![2](https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/gif/funcionalities.gif)](https://www.youtube.com/watch?v=alATNutyEoA&feature=youtu.be)

Click the animated gif for the full version!

-->
# Authors

- [GUIU OMS FONT](https://github.com/guiuomsfont) - 1525686

- [QUIM CAMPRUBÍ CASAS](https://github.com/quimcamprubi) - 1528104

- [JOSEP BRAVO BRAVO](https://github.com/LeBrav) - 1526453

- [ARNAU JOSEP ALCÓN ACEDO](https://github.com//1529603) - 1529603
