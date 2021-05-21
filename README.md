# 482-Furuta-Pendulum

  # Table of Contents  
1) [Introduction](#headers)  
2) [Objectives](#headers)  
3) [Derivation](#headers) 
4) [Matlab Code](#headers) 
5) [Model Simulation](#headers) 
6) [Results](#headers) 

  # Introduction
  
The Furtua Pedulum otherwise known as the rotary inverted pendulum is a basic system to help exemplify a non-linear system that is have a rotary arm atteched to a sevro motor
that rotates about the z-axis in the horizontal plane. Using centripital force the system is able to balance the pendulum arm that is attached to the end of the rotary arm in
the upright position. Through utilizing this model the team will derive a state space representation of the model, apply this state space matricies in Matlab, output this
system to Simulink, and connect the final result to Coppelia Sim in order to verify the result using a simulated model.    

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
>
<br>![Pendelum Balance](https://github.com/jmmather10/482-Furuta-Pendulum/blob/21ca6dc59d7f82e6f4237d747ff048897aeb2aaa/Pendulum_Images/Balance.JPG?raw=true "Rotopen Balance")
>Simulink Pendelum Balance
>>
<br>![Zero Pole Map](https://github.com/jmmather10/482-Furuta-Pendulum/blob/7aa464eb652acbaf4eb3da7ee2e73d9f28ff8ecd/Pendulum_Images/Zero%20Pole.jpg "Zero Pole Map")
>Poles of the system from Vm
>>
<br>![Zero Pole Map](https://github.com/jmmather10/482-Furuta-Pendulum/blob/7aa464eb652acbaf4eb3da7ee2e73d9f28ff8ecd/Pendulum_Images/Zero%20Pole%20Map.jpg?raw=true "Zero Pole Map")
>Zero Pole Map
>>
<br>![Compensated Step Response](https://github.com/jmmather10/482-Furuta-Pendulum/blob/7aa464eb652acbaf4eb3da7ee2e73d9f28ff8ecd/Pendulum_Images/Compensated%20Step%20Response.jpg?raw=true "Compensated Step Response")
>Compensated Step Response
>>
Developing the Matlab code based on the equations of motion,state space functions, and A & B martices derived by hand took some finesse and were able to be introduced using a matrix format, this made it possible to integrate the code into the provided simulink files. Additionally, the code was quite extensive but integration of the provided software and combination of information found in opensource online files the Matlab code worked to output the appropriate deliverables.
Utilization of the Simulink system introduced a big learning curve for the project, without any prior experience the team struggled with using the application. Once some of the code was being introduced into simulink to create outputs required for Coppelia Sim a warning/error for the swingup simulink file occured because of a requirement to have a quarc_library downloaded and implimented and this was not provided in the software file, so this library has been requested from quanser.com for use in the project. The simulation for the rotpen_bal has thrown an error due to a possible "singularity" in the file or a need for a possible reduction in step size.

  # Model Simulation
 
 <br>![CoppeliaSim Model](https://github.com/jmmather10/482-Furuta-Pendulum/blob/main/Pendulum_Images/Happy_pendulum.jpg?raw=true "CoppeliaSim Model") 
 Pendulum model in CoppeliaSim, running using the Newton engine.<br/>
 <br>![State Space Coefficients](https://github.com/jmmather10/482-Furuta-Pendulum/blob/main/Pendulum_Images/MatLab_SS_Rep.jpg?raw=true "State Space Coefficients")
 State space representation with balance control gain and swing-up parameters.<br/>
 <br>![Connection Test](https://github.com/jmmather10/482-Furuta-Pendulum/blob/main/Pendulum_Images/Test_Connect.jpg?raw=true "Connection Test")
 Test connection MatLab mouse position output.<br/> 
 <br>![CoppeliaSim Error](https://github.com/jmmather10/482-Furuta-Pendulum/blob/main/Pendulum_Images/Connection_Error.jpg?raw=true "CoppeliaSim Error")  
 Error while trying to connect MatLab model to CoppeliaSim.<br/> 
  
  # Results
  
The governing non-linear equations of the system were derived and linearized sucessfully by using small angle approixmation. The A matrix is found by solving the linearized
governing equations of motion for the accerations of the arm and the pendulum, theata (arm) and apha (pendulum) double-dot respectively. These accerations are in terms of the
torque applied to the motor as an input. The state space representation can be seen in the images as the A, B, C, and D matrices, also displayed are the balance control gain
coefficients and the swing-up parameters. Connecting CoppeliaSim with MatLab was successful only with the SimpleTest.m file provided in the tutorial setup. There needs to be further research on how to use Simulink and implement the simulation as planned.
