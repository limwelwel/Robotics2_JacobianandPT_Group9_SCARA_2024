<p align="center">
  <img src=https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/44a768492060b21e114aa6e205f7cb09aa34ecfa/bsu%20header.png alt=Bsu style="height: 200px;">
  <hr>
<h3 align="center">COLLEGE OF ENGINEERING</h3>
<h3 align="center">BACHELOR OF SCIENCE IN MECHATRONICS ENGINEERING</h3>
<h3 align="center">ROBOTICS 2: Final Project</h3>
<h1 align="center"> Jacobian Matrix and Path & Trajectory Planning </h1> 
<br>

## I. Abstract of the Project
<p align="justify"> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;This project offers an in-depth analysis of a <b>RRP</b> variant <b>SCARA</b> Manipulator, <b><i>Selective Compliance Articulated Robot for Assembly</i></b>. The <b>Jacobian Matrix</b> calculation will be generated with the help of the program code and the <b>Path and Trajectory Planning</b> will be covered. Furthermore, the <b>GUI Calculator</b> gives assistance in terms of checking if the calculations of various parts of the SCARA manipulator are correct. Thus, it will inspect the content that will be inputted to determine if there are no errors being exhibited. All the fundamental concepts that have been mentioned were supported through simulation in Python, whereas it can be utilized to operate a wide range of services and programs..</p>

## II. Introduction of the Project
<p align="center">
<img src="https://github.com/limwelwel/DRAFT/blob/886150416b7f3f652f653c6c94ded8c8a9af1dfc/scara.png"  height="300px"/> <img src="https://github.com/limwelwel/DRAFT/blob/886150416b7f3f652f653c6c94ded8c8a9af1dfc/scara.gif" height="300px"/>

<p align="justify"> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In 1978, Professor Hiroshi Makino's group at Yamanashi University in Japan created the first <b>SCARA</b> <b><i>(Selective Compliance Articulated Robot for Assembly)</i></b> manipulator for use in industry. The initial purpose of SCARA robots was to meet the demand for quick and accurate assembly activities in the manufacturing sector. For a variety of assembly processes, including pick-and-place, assembly, and packaging applications, its mobility is helpful. SCARA robots, which were initially introduced to industrial assembly lines in 1981, continue to provide the finest results for high-speed assembly. The desire to combine the advantages of Cartesian and Articulated robots, allowing for increased reach, flexibility, and precision in assembly tasks, had an impact on the development of SCARA manipulators.
  <p align="justify"> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;As technology undergoes rapid advancement, complicated tasks such as control systems are accomplished with a highly automated control system. Due to increased use of industrial robot arms, an evolution to that topic began trying to imitate human movements in a detailed mode. The mechanical design of the robot arm is based on a robot manipulator with similar functions to a human arm. The links of such a manipulator are connected by joints allowing rotational motion and the links of the manipulator are considered to form a kinematic chain. The business end of the kinematic chain of the manipulator is called the end effector or end-of-arm-tooling and it is analogous to the human hand. To validate the right positioning of the robotic arm, inverse kinematics calculations are carried out.

## III. Jacobian Matrix of SCARA Mechanical Manipulaltor
<p align="center"><img src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/997852d02716c1a5c075a7d8b32b30ed78079f10/Final/Jacobi.png"  height="150px"/>
<h4 align="center"> <b>Carl Gustav Jacob Jacobi</b> </h4>
<p align="justify"> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;The <b><i>Jacobian Matrix</i></b> is a mathematical matrix that captures the relationship between the partial derivatives of a function and its variables. In the context of robot programming, this function is typically the forward kinematics, which establishes the correspondence between the robot's joint angles and the position as well as the orientation of its end-effector. By employing the Jacobian matrix, we can understand how changes in the joint velocity affect the velocity of the end-effector. This matrix finds application in diverse areas of robot programming, including motion planning, control, and optimization, enabling efficient and effective operations.

<p align="center">
<img src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/44a768492060b21e114aa6e205f7cb09aa34ecfa/Laboratory%202/1.png">
<p align="center">
<img src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/44a768492060b21e114aa6e205f7cb09aa34ecfa/Laboratory%202/2.png">
<p align="center">
<img src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/44a768492060b21e114aa6e205f7cb09aa34ecfa/Laboratory%202/3.png">
<p align="center">
<img src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/44a768492060b21e114aa6e205f7cb09aa34ecfa/Laboratory%202/4.png">
<h3 align="center"> <b><i>Figure 1. SCARA Manipulator Jacobian Matrix Calculation</i></b> </h3>
  
<p align="center">
  <a href="https://drive.google.com/file/d/1op9exDoMc77kwkfGMwfVgLHAAUpMgtI7/view?usp=sharing"><img alt="Task 1" title="Task 1" src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/d6691994aff4549a985d125d7ee75fd6425c1af2/Final/Jacobian%20MAtrix.png"height="400px"/></a>
<h3 align="center"> <b><i>Video 1. Solving the Jacobian Matrix of the Standard SCARA Manipulator</i></b> </h3> 

<p align="center">
<img src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/44a768492060b21e114aa6e205f7cb09aa34ecfa/Laboratory%202/5.png">
<h3 align="center"> <b><i>Figure 2. SCARA Manipulator Singularity Calculation</i></b> </h3>
  
<p align="center">
  <a href="https://drive.google.com/file/d/1eeaC-MJpd6UdjeQKlfFawMI1_1w4Df4t/view?usp=sharing"><img alt="Task 1" title="Task 1" src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/d6691994aff4549a985d125d7ee75fd6425c1af2/Final/Singularity.png"height="400px"/></a>
<h3 align="center"> <b><i>Video 2. Solving the Singularity of the Standard SCARA Manipulator</i></b> </h3> 

## IV. Differential Equation of SCARA Mechanical Manipulaltor
<p align="justify"> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; The <b><i>differential method</i></b>, when applied in the context of the Jacobian matrix, leverages the Jacobian to analyze and approximate the behavior of multivariable functions. This technique is particularly valuable for obtaining linear approximations of nonlinear functions, solving systems of equations, and gaining insights into the local behavior of transformations. By examining the Jacobian matrix, which consists of first-order partial derivatives, one can understand how small changes in input variables influence the output. This is essential in fields such as optimization, numerical analysis, and dynamical systems, where precise local behavior can inform broader system understanding and solutions.
<p align="center">
<img src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/3417b4b28f2a69fba5b2c2c6919cba4ac703baf9/Final/d.png">
<p align="center">
<img src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/3417b4b28f2a69fba5b2c2c6919cba4ac703baf9/Final/e.png">
<h3 align="center"> <b><i>Figure 3. SCARA Manipulator Differential Equation Calculation</i></b> </h3>
  
## V. Path and Trajectory Planning of SCARA Mechanical Manipulaltor
<p align="justify"> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <b><i>Path Planning</i></b> is a way for mechanical manipulators to find the shortest and most obstacle-free path from the start to its goal state. The path that is used can be a set of states (known as position and/or orientation) or even waypoints. Moreover, path planning is required to have a map of the mechanical manipulator's environment along with its start and goal states as input. This map could be represented with the use of grid maps, state spaces, or even topological roadmaps.
<p align="justify"> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<b><i>Trajectory Planning</i></b>, on the other hand, consists of finding a time series of successive joint angles, which allows a mechanical manipulator to move from a starting configuration toward a goal configuration, which allows the manipulator to achieve a certain task. In addition, it is used in generating a time schedule for how to follow a path given constraints such as position, velocity, and acceleration.

<p align="center">
<img src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/da654a873ed5d446df10b1326480d5db8b6f3762/Final/pick%20and%20place.png" height="400px"/>
<h3 align="center"> <b><i>Figure 4. SCARA Manipulator Pick and Place</i></b> </h3>
  
<p align="center">
<img src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/da654a873ed5d446df10b1326480d5db8b6f3762/Final/Welding.png" height="400px"/>
<h3 align="center"> <b><i>Figure 5. SCARA Manipulator Welding</i></b> </h3>

<p align="center">
<img src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/da654a873ed5d446df10b1326480d5db8b6f3762/Final/GUI.png"  height="500px"/> 
<h3 align="center"> <b><i>Figure 6. GUI with velocity calculator and path and trajectory of the Standard SCARA Manipulator</i></b> </h3>

<p align="center">
  <a href="https://drive.google.com/file/d/156aP5h_4mMLDn2SP1jIzFICRTXFqht-V/view?usp=sharing"><img alt="Task 1" title="Task 1" src="https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/da654a873ed5d446df10b1326480d5db8b6f3762/Final/PT.png"height="400px"/></a>
<h3 align="center"> <b><i>Video 3. Path and Trajectory and GUI of the Standard SCARA Manipulator</i></b> </h3> 

## VI. References
<p align="justify"> 
The Robot Hall of Fame - Powered by Carnegie Mellon University. (n.d.). http://www.robothalloffame.org/inductees/06inductees/scara.html#:~:text=The%20limited%20motion%20of%20the,ratio%20regarding%20high%20speed%20assembly.
  
TheFamousPeople. (n.d.). Carl Gustav Jacob Jacobi. Carl Gustav Jacob Jacobi Biography. https://www.thefamouspeople.com/profiles/carl-gustav-jacob-jacobi-6029.php

<h3 align="center">G9 SCARA ENGINEERS</h3>

<h4 align="center">Ada, Lemuel J. - PP <i>(Project Programmer)</i></h4>
<h4 align="center">Cometa,Steven Shaine M. - PL <i>(Project Leader)</i></h4>
<h4 align="center">Eje, - PQ <i>(Project Quality Engineer)</i></h4>
<h4 align="center">Umali, Felicity D. - PS <i>(Project Supervisor)</i></h4>

<hr>
<p align="center">
  <img src=https://github.com/limwelwel/ROBOTICS-2-PICTURES-AND-GIF/blob/44a768492060b21e114aa6e205f7cb09aa34ecfa/bsu%20footer.png alt=Bsu style="height: 200px;">
