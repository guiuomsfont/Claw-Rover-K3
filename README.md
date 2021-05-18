# Claw-Rover-K3
Project of a programmed and simulated robot that drives autonomously and collects garbage.

<p align="center"><img src="/Design/Images/Robot.png" width="300"/></p>

   * [What is this?](#What-is-this)
   * [Description](#Description)
   <!-- [Amazing contributions](#Amazing-contributions)-->
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

<!-- We are 3rd year Computer Science students and this is a robotics project for our subject on Robòtica, Llenguatge i Planificació - Robotics Language and Planning.


We have focused deeply on polishing the software part of the project, but due to the circumstances (this project was interrupted by the 2020 Coronavirus outbreak), we haven't been able to implement it on hardware. Instead, we have prepared a full-on videogame to act as a simulation for what this project can become. And it is prepared for anyone to take it and move our sofware modules to a phisical robot.

This is where you come in! -->


# Description
In this project you will find the code, simulations and 3D designs of the parts of a robot developed from scratch, which is able to drive autonomously and collect garbage previously detected using computer vision techniques. This robot consists, schematically, of an anthropomorphic arm located on a platform with four wheels (front-wheel drive). It is implemented with Coppelia simulator and, in case you want to carry it out in real life, the instructions dedicated to the movement of the simulation robot should be substituted by communication instructions from a Raspberry Pi board to an Arduino board. This is a necessary step in order to correctly interact with the hardware components of the robot, such as the wheels, the arm or the sensors.

<!--
_C.A.R.L.E.S_ is able to:<img src="https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/imgs/cartas.png" align="right" width="300" alt="header pic"/>
- Recognize cards with the computer vision module (number and suit of the card).
- Play a brisca game with an AI algorithm that will do its best to outsmart the opponent.
- Control the actions to be performed (start and end game, steal a card, knowing it's C.A.R.L.E.S.' turn, etc.) by voice commands that the human will say during the game.
- Calculate the angles of rotation of the arm motors in order to move the manipulator (including the one that controls the pressure of the suction cup) from one point to another, according to the positions of everything else on the board.
- Unification of all the modules in a single workflow, which is what the physical robot would have had.

-->

# Amazing contributions

<!--
The three most important contributions in which our robot stands up are:<img src="https://github.com/OriolMoreno/C.A.R.L.E.S/blob/master/imgs/braç.png" align="right" width="150" alt="header pic"/>
- Entertainment for seniors:  it is designed to entertain the elderly, for whom robots are a whole new thing.
- Classic Game Automation, the brisca: we give life to a classic and mythical game like the brisca combining it with technology and having it be even more enjoyable.
- Voice recognition with human interaction: Designed to bring the user closer to the robot and allows them to communicate with it.
-->

# Hardware Scheme
This is the Hardware Scheme we planned for this project, within the 100€ budget. 

<p align="center"><img src="Design/Images/Hardware-Scheme.png" width="300"/></p>

Here is the list of hardware components with purchase links:
https://github.com/guiuomsfont/Claw-Rover-K3/Design/Hardware_Components.txt


# 3D pieces
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

-->

## Modules
<!--
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

- [ARNAU JOSEP ALCÓN ACEDO](https://github.com/arnaujosepalcon) - 1529603
