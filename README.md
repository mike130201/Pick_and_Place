<a name="top"></a>
# Pick and Place
<p align="center">
  <img src="Images/Open.jpg" alt="Imagen Open" style="width:30%;"> 
</p>


<p align="center">This is an implementation of a pick and place solution for the OpenManipulatorX.

## Project Overview
The Open MANIPULATOR-X robot based on ROS is one of the most commonly used robotic arms for training in the industry. In this project, a Pick and Place operation that will be performed using this robot arm. The goal is to create a program that instructs the robotic arm to pick up a series of objects and place them at a specific point in the workspace. Prior to this, a thorough analysis of the arm will be conducted to obtain its direct and inverse kinematics, and will be done using Matlab software in collaboration with the Robotics Toolbox plug-in developed by Peter Corke. For the Open MANIPULATOR-X will be used the teleoperation funtions to create a new project that execute the Pick and Place.
  
  
  
## Content List
- [Pick and Place](#pick-and-place)  
- [Project Overview](#project-overview)  
- [Requirements](#requirements)
- [Robot kinematics](#robot-kinematics)
  - [Denavit-Hartenberg parameters](#denavit-hartenberg-parameters)
  - [Obtaining and Validation of the Forward Kinematics and Inverse Kinematics using Matlab](#obtaining-and-validation-of-the-forward-kinematics-and-inverse-kinematics-using-matlab)
- [Initial Setup](#initial-setup)
  - [Installation](#installation)
  - [Open CR](#open-cr)
  - [Simulation](#simulation)
- [Add New Things](#add-new-things)
- [Important Links](#important-links)
- [Contact](#contact)
## Requirements

To run this project you need the following components:

- Ubuntu 20.04.
- ROS Noetic.
- Matlab.
- OpenManipulatorX.
- Open CR.
- Gazebo (Optional)

# Robot kinematics

### Denavit-Hartenberg parameters.

In order to obtain the kinematic analysis of the Robot, it is required to obtain the Denavit-Hartenberg parameters, this by means of the fifteen steps.
In order to obtain the kinematic analysis of the robot, it is required to obtain the Denavit-Hartenberg parameters, this by means of the fifteen steps, nevertheless, the most important thing is to consider which is the position 0 of the robot, in the following image we can see how the OPENMANIPULATOR X is the position 0.

<p align="center">
  <img src="Images/Open.png" alt="Imagen Open">
</p>

As this is the expected configuration, we started to elaborate each of the steps to obtain the parameters.
parameters.

1. Number the links: "0" will be called the "ground", or fixed base where the robot is anchored. "1" the first
first mobile link, etc. 0-1-2-3 Links.

<p align="center">
  <img src="Images/Paso1.png" alt="Imagen Open">
</p>

2.Number the joints: "1" will be the first degree of freedom, and "n" the last. 1-4 Articulations

3. Locate the axis of each joint: For pairs of revolution, it will be the axis of rotation. For prismatic, it will be the axis along which the link moves.

<p align="center">
  <img src="Images/EjeDeCadaArticulacion.png" alt="Imagen Open">
</p>

4. Z axes: We start placing the XYZ systems. We place the Zi-1 on the axes of the i joints, with i = 1, . . . ., n. That is, Z0 goes on the axis of the 1st joint, Z1 goes on the axis of the 2nd degree of freedom, etc.

<p align="center">
  <img src="Images/EjesZ.png" alt="Imagen Open">
</p>

5. Coordinate system 0: The origin point is located at any point along Z0. The orientation of X0 and Y0 can be arbitrary, provided of course that XYZ is a dextrorotating system.
6. Rest of systems: For the rest of the systems i = 1, . . . ., N - 1, place the origin point at the intersection of Zi with the normal common to Zi and Zi+1. intersection of Zi with the normal common to Zi and Zi+1. In case the two Z axes intersect, place it at that point of intersection. In case they are parallel, place it in some point of the i + 1 joint.

<p align="center">
  <img src="Images/RestoDelSistema.png" alt="Imagen Open">
</p>

7. X-axis Each Xi goes in the direction of the normal common to Zi-1 and Zi, in the direction from Zi-1 to Zi. to Zi.
8. Y axes: Once the Z and X axes are located, the Y axes have their directions determined by the constraint of forming a dextrorotating XYZ. constraint of forming a dextrorotating XYZ.
9. Robot end system: The n-th XYZ system is placed at the robot end (tool), with its Z axis (tool), with its Z-axis parallel to Zn-1 and X and Y in any valid direction.

<p align="center">
  <img src="Images/Ejes_y_SistemaExterno.png" alt="Imagen Open">
</p>

10. Angles θ: Each θi is the angle from Xi-1 to Xi revolving around Zi.

Θ1=-180° & 180°
Θ2=-180° & 180°
Θ3=-180° & 180°
Θ4=-180° & 180°

11. Distances d: Each di is the distance from the XY system Zi-1 to the intersection of the common normals of Zi-1 towards Zi , along Zi-1 . common normals from Zi-1 to Zi , along Zi-1.

d1 = 0.77 mm
d2 =0.128 mm
d3 =0mm
d4 =0mm

12. Distances a: Each ai is the length of such common normal.

a1=0mm
a2=0.024mm
a3=0.124mm
a4=0.126mm

13. Angles ' α: Angle to rotate ' Zi-1 to reach Zi, rotating around Xi.

𝛼1=pi/2
𝛼2=0
𝛼3=0
𝛼4=0

14. Total transformation: The total transformation matrix relating the robot base to its tool is the chaining (multiplication) of all these matrices: T =0 A1 ∗ 1 A2 ∗ .... n-1An

<p align="center">
  <img src="Images/DH.png" alt="Imagen Open">
</p>


### Obtaining and validation of the Forward Kinematics and Inverse Kinematics using Matlab.

Made the DH parameters, it is possible to obtain the direct and inverse kinematics of the robot, this will be done using the Matlab software, in conjunction with Peter Corke's "Robotics Toolbox" plugin, by obtaining and validating both kinematics we can obtain the kinematic analysis of the robot.

In general, the procedure consists of generating the robot in Matlab, giving the specifications of the measurements and types of the joints, as well as the rotation that they have, that is, substituting the values in the DH matrix, which remains in the following way:

<p align="center">
  <img src="Images/DHvalores.png" alt="Imagen Open">
</p>

(IMAGEN DE DH CON VALORES)

Using the ".teach" command, the graphical interface of the robot is printed, which is as follows:

(IMAGEN DL ROBOT EN MATLAB)

The direct kinematics is obtained by calculating through the analysis of each of the joints, to later obtain the analysis of the entire robot by multiplying the results of all the joints, the procedure for each joint is as follows:

(IMAGE OF THE GENERAL PROC OF THE CD)

Therefore, it is enough to substitute the values ​​in each of the operations, said values ​​are the same as those of the DH table, this applies to all the joints of our robot, with which it only remains to multiply the analyzes of the different joints. To know that the calculation is correct, the values of the matrix obtained are compared with the values ​​of the simulation, as observed below:

(Image of the robot and the CD)

Since the values are the same, the calculation is correct.

To obtain the inverse kinematics, a complex analysis must be carried out for each joint, in which the procedure differs depending on the characteristics of each one, so using the Robotics Toolbox plugin, the ".ikine" command can be used, which performs the calculation in the position that we assign, the result is shown below:

(IMAGEN DE LA CI CON EL ROBOT)

With both kinematics validated, we proceed to use Ros and Gazebo.


[Back to Top](#top)
## Initial Setup

### Installation
Is important to say that is recomended to install Ubuntu 20.04 in the computer not in a Virtual Box.
The OpenManipulator is configurated to work in ROS Noetic in the mentioned Ubuntu version, once the Ubuntu is installed is recomender to install ROS from the Wiki:
http://wiki.ros.org/noetic/Installation/Ubuntu


However the installation also can be donde with this comand that is the fast installation:
```ROS
$ sudo apt update
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
$ chmod 755 ./install_ros_noetic.sh
$ bash ./install_ros_noetic.sh
```

Install dependent pacakges
```ROS
$ source ~/.bashrc
$ sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo* ros-noetic-moveit* ros-noetic-industrial-core
$ sudo apt install ros-noetic-dynamixel-sdk ros-noetic-dynamixel-workbench*
$ sudo apt install ros-noetic-robotis-manipulator
```
Download and build OpenMANIPULATOR-X packages
```ROS
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
$ cd ~/catkin_ws && catkin_make
```
This project is an addition for the OpenManipulator_teleop, a cpp programan was created in the src and all the dependencies as the lunch and the header where created, also the Cmake file was modified to add the pick and place funtion, so to get this project once all the OpenMANIPULATOR-X packages are installed, a new code can be created with the name and location of the files that are added in this repository, it can be added by VS code or directly in the text editor but save it with the corresponding name
### Open CR
Once all the OpenMANIPULATOR-X packages are installed and ROS the hardware provided by the lab for the manipulation of the Robot is the Open CR 
<p align="center">
  <img src="Images/opencr.png" alt="Open CR" style="width:20%;"> 
</p>
In order to connect the computer to the Open Manipulator X is important to download Arduino, the version downloaded by the date of this project is the Arduino IDE 1.8.19 and the version is for Linux.


https://www.arduino.cc/en/software


Once Arduino is downloaded is time to configure the Open CR to the port of the computer, the Open has a guide where in case of the Open CR is not flashed the new user can configured as well

https://emanual.robotis.com/docs/en/parts/controller/opencr10/#arduino-ide

First the ports are configured:

USB Port Settings
```ROS
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
$ sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```
Compiler Settings
```ROS
$ sudo apt-get install libncurses5-dev:i386
```
Now once Arduino is downloaded extract the file and install it.

```ROS
$ cd ~/downloads/arduino-1.8.19
$ ./install.sh
```
Set the file path of installed Arduino IDE as an absolute path named PATH in the bashrc file.
```ROS
$ gedit ~/.bashrc
$ export PATH=$PATH:$HOME/tools/arduino-1.8.19
$ source ~/.bashrc
```
Porting to Arduino IDE(Linux)
Install the OpenCR package via Boards Manager
Click Tools → Board → Boards Manager.
<p align="center">
  <img src="Images/Openboard.png" alt="Board configuration" style="width:30%;"> 
</p>
<p align="center">
  <img src="Images/Openboard2.png" alt="Port Configuration " style="width:30%;"> 
</p>
However, if is not clear enough there is the link for the Open CR in the links given above

[Back to Top](#top)
### Simulation
Once all the previous steps are done the implementation can be done.

For initialized the Open is command is necessary to give torque to the motors
```ROS
$ roslaunch open_manipulator_controller open_manipulator_controller.launch usb_port:=/dev/ttyACM0 baud_rate:=1000000
```
To run the project you need to write the following command
```ROS
$ roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch
```
Finally the depending of the choice the robot wil start moving, in the video is shown the Palletizing and the Depalletizing:


<p align="center">
  <a href="https://www.youtube.com/watch?v=AY5m8ooS1Zg"><img src="https://img.youtube.com/vi/AY5m8ooS1Zg/0.jpg" alt="Video de pick and place"></a>
</p>


## Add New Things

If you want to contribute to this project, please follow these steps:

1. set the positions of the things that are going to be pick and place.
2. With the matlab codes calculate the inverse kinematics to obtain the positions of the ariticulations.
3. That values can be simulated with the GUI Program. (Do not forget to connect to the Open before)
```ROS
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
```
4. Verify the positions and save it in the new cpp code .
5. Is recomended to simulate first in Gazebo to avoid problems with the real Open.

Launch Gazebo:
```ROS
$ roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
```
Connect the Open to Gazebo
```ROS
$ roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
```
Launch your program
```ROS
$ roslaunch open_manipulator_teleop open_manipulator_teleop_Pick_and_Place.launch
```
if the simulation is correct now you can do it with the OpenManipulatorX

## Important Links
This proyect is based on the Keyboard cpp program, however new functions are called and created, if you want to see the main differences with the original project this link can be check:

https://github.com/ROBOTIS-GIT/open_manipulator

For the OpenManipulator e-Manual you can Check this link:

https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/

For the Open CR:

https://emanual.robotis.com/docs/en/parts/controller/opencr10/

Also, other source where more information can be found is the the following book:

http://wiki.ros.org/Books/ROS_Robot_Programming_English
## Contact 
Authors:
José Miguel Zúñiga Juárez - jose.zunigajz@udlap.mx


Project Link: https://github.com/mike130201/Pick_and_Place

# Enjoy, entertain yourself, and improve the program!
