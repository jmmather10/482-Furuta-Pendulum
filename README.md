# 482-Furuta-Pendulum

  # Table of Contents  
1) [Introduction](#headers)  
2) [Objectives](#headers)  
3) [Derivation](#headers) 
4) [Matlab Code](#headers) 
5) [Model Simulation](#headers) 
6) [Results](#headers) 

  # Introduction
  The Furtua Pedulum otherwise known as the rotary inverted pendulum is a basic system to help exemplify a non-linear system that is have a rotary arm atteched to a sevro motor that rotates about the z-axis in the horizontal plane. Using centripital force the system is able to balance the pendulum arm that is attached to the end of the rotary arm in the upright position. Through utilizing this model the team will derive a state space representation of the model, apply this state space matricies in Matlab, output this system to Simulink, and connect the final result to Coppelia Sim in order to verify the result using a simulated model.    

  # Objectives
* Linearize the non-linear equations of motion.
* Derive the State Space representation of the model.
* Develope a state-feedback control system that will balance the pendulum in the erect position using Pole Placement.
* Simulate the closed loop system using Simulink.
* Set up a swing up control that is energy based.
* Output the system to Coppelia Sim to test out the code in a simulated model.

 # Derivation
![Equations of motion](https://github.com/jmmather10/482-Furuta-Pendulum/blob/main/Pendulum_Images/Derivations__Page_1.png?raw=true "Derivations Pg 1")

![State Space representation](https://github.com/jmmather10/482-Furuta-Pendulum/blob/main/Pendulum_Images/Derivations__Page_2.png?raw=true "Derivations Pg 2")
  
![A and B matrices](https://github.com/jmmather10/482-Furuta-Pendulum/blob/main/Pendulum_Images/Derivations__Page_3.png?raw=true "Derivations Pg 3")
   
  # Matlab Code
<br>Below are some of the parameter constants of the system.<br/>
><br>% Element Output parameters:<br/>
><br>% g:             Gravitational constant                         (m/s^2)<br/>
><br>% Mx:            Element Mass with T-fitting                    (kg)<br/>
><br>% Lx:            Full Length of the element (w/ T-fitting)      (m)<br/>
><br>% lx:            Distance from pivot to centre Of gravity       (m)<br/>
><br>% Jx:            moment of inertia                              (kg.m^2)<br/>
><br>% Bx:            Viscous damping coefficient as seen at the element axis (N.m.s/rad)<br/> 
><br><br/> 
  # Model Simulation
 
 <br>![CoppeliaSim Model](https://github.com/jmmather10/482-Furuta-Pendulum/blob/main/Pendulum_Images/Sad_Pendulum.jpg?raw=true "CoppeliaSim Model") 
 Pendulum model in CoppeliaSim, running using the Newton engine.<br/>
 <br>![State Space Coefficients](https://github.com/jmmather10/482-Furuta-Pendulum/blob/main/Pendulum_Images/MatLab_SS_Rep.jpg?raw=true "State Space Coefficients")
 State space representation with K-coefficients.<br/>
 <br>![Connection Test](https://github.com/jmmather10/482-Furuta-Pendulum/blob/main/Pendulum_Images/Test_Connect.jpg?raw=true "Connection Test")
 Test connection MatLab mouse position output.<br/> 
 <br>![CoppeliaSim Error](https://github.com/jmmather10/482-Furuta-Pendulum/blob/main/Pendulum_Images/Connection_Error.jpg?raw=true "CoppeliaSim Error")  
 Error while trying to connect MatLab model to CoppeliaSim.<br/> 
  
  # Results
The governing non-linear equations of the system were derived and linearized sucessfully by using small angle approixmation. The A matrix is found by solving the linearized governing equations of motion for the accerations of the arm and the pendulum, theata (arm) and apha (pendulum) double-dot respectively. These accerations are in terms of the torque applied to the motor as an input. The state space representation can be seen in the images as the A, B, C, and D matrices.
