clc
clear


% Element Output parameters:
% g             Gravitational constant                         (m/s^2)
% Mx            Element Mass with T-fitting                    (kg)
% Lx            Full Length of the element (w/ T-fitting)      (m)
% lx            Distance from pivot to centre Of gravity       (m)
% Jx            moment of inertia                              (kg.m^2)
% Bx            Viscous damping coefficient as seen at the 
%               element axis                                   (N.m.s/rad)

%Gravity constant
g = 9.81;

%Pendulum Output parameters ( PEND_TYPE, 'MEDIUM_12IN') 
Mp = 0.127;
Lp = 0.33655;     
lp = Lp/2; 
Jp = Mp*Lp^2/12; 
Bp = 0.0024;  

%Arm Output parameters 
Mr = 0.257;            
Lr = 0.2159;          
lr = (2+7/16)*0.0254;        
Jr = Mr*Lr^2/12;     
Br = 113.5e-3;   

Kgi = 14;       
Kge = 5;       
Kg = Kgi * Kge; 


% Motor Inertia 
Jm_rotor = 3.9e-7;      
Jtach = 0;
Jm = Jm_rotor + Jtach;
ng = 0.9;                                  
nm =0.69;                                 
kt = 1.088 * 0.2780139 * 0.0254; %=.00767           
Rm = 2.6;                                  
km = 0.804 / 1000 * (60 / ( 2 * pi )); %=.00767      

%Amplifier Max Output Voltage (V) and Output Current (A)
%VoltPAQ
VMAX_AMP = 24;
IMAX_AMP = 4;
K_AMP = 1;

% Potentiometer Sensitivity (rd/V)
K_POT = -(352 * pi / 180 / 10);

% Encoder Resolution, for a quadrature encoder, (rd/count)    
K_ENC = 2 * pi / ( 4 * 1024 );

Jt = (Jp.*Mp.*(Lr.^2)) + (Jr.*Jp) + ((1./4).*Jr.*Mp.*(Lp.^2));  

% State Space Representation
A = zeros(4);
B = [0;0;0;1];

V_m_modifier = (ng*Kg*nm*kt)/(Rm);
A(1,3) = (1/Jt) * 1;
A(2,4) = (1/Jt) * 1;
A(3,2) = (1/Jt) * ((1/4)*(Mp^2)*(Lp^2)*Lr*g);
A(3,3) = (1/Jt) * (-Br.*Jp - (1/4)*Br*Mp*(Lp^2));
A(3,4) = (1/Jt) * ((-1/2)*Mp*Lp*Lr*Bp);
A(4,2) = (1/Jt) * ((1/2)*Mp*Lp*g*(Jr + Mp*(Lr^2)));
A(4,3) = (1/Jt) * ((-1/2)*Mp*Lp*Lr*Br);
A(4,4) = (1/Jt) * (-Jr*Bp - Bp*Mp*Lr^2);
A(3,3) = A(3,3) - Kg^2*kt*km/Rm*B(3);
A(4,3) = A(4,3) - Kg^2*kt*km/Rm*B(4);

B = [0;0;0;1];
B(3,1) = V_m_modifier* (1/Jt) * (Jp + (1./4)*Mp*(Lp^2));
B(4,1) = V_m_modifier* (1/Jt) * ((1/2)*Mp*Lp*Lr);
B = Kg*kt*B/Rm;
B(3,1) = V_m_modifier* (1/Jt) * (Jp + (1/4)*Mp*(Lp^2));
B(4,1) = V_m_modifier* (1/Jt) * ((1/2)*Mp*Lp*Lr);

C = zeros(2,4);
C(1,3) = 1;
C(2,4) = 1;
D = zeros(2,1);

%Verifying Matrices
A;
B;
C;
D;

states = {'X1','X2','X3','X4'};
inputs = {'V_M'};
outputs = {'Theta','Alpha'};

% Open loop system model
furuta_ss = ss(A,B,C,D,'StateName', states,'InputName',inputs,'OutputName',outputs)

% Provides the size of the State Space model, the # of inputs, outputs and states
size(furuta_ss);

% Determines the Stability of this System :1 Means the system is stable, 0 means it is unstable
Stability_F_ss = isstable(furuta_ss);
Con_F_ss = ctrb(A,B);   % The system is controllable if Con_F_ss has a full rank n

% Determines the number of uncontrollable states - checking if it has full rank
unCon_number_states = length(A) - rank(Con_F_ss);
% Note: The un_Con_number_states = 0, therefore the SS LTI Model is full
% rank and is thus a controllable system

% Determines the Observability Matrix os the SS Model
Obs_F_ss = obsv(A,C);   % The model is observable if Obs_F_ss has a full rank n
% Note: The Obs_F_ss = 0, therefore the SS Model is full
% rank and is thus an observable system

% Number of unobservable states
unObs_num_states = length(A) - rank(Obs_F_ss);

%Transfer Function of the System in Laplace (s-domain)
Furuta_Laplace = tf(furuta_ss);               

% Cross Checking the poles of the functions with plot
Pole = pole(furuta_ss);

% Creates a pole zero map of the State Space Model
figure(1);
h = pzplot(furuta_ss);    % the x indicates a pole and o indicates a zero
grid on;

% Places the above plotted zeros and poles into a column vector
[p,z] = pzmap(furuta_ss);   

% Computes and plots the poles and zeros of each input/output pair of the dynamic system model
figure(2);
h2 = iopzplot(furuta_ss);
% For whatever reason the zeros are ploted in this setup, I do not know why

% Balance control enable range (rad)
epsilon = 12.0 * pi / 180;
% Control Specifications
zeta = 0.7;
wn = 4;
p3 = -30;
p4 = -40;

% Intermediate calculations for finding desired pole placement
sigma = zeta .* wn;
wd = wn .* (sqrt(1 - (zeta.^2)));

% Calculated Complex conjugate dominant poles
p1 = complex(-sigma,wd);    %p1 = -sigma + wd j;
p2 = complex(-sigma,-wd);   %p2 = -sigma - wd j;

desired_poles = [p1 p2 p3 p4];      % Assembles desired pole placement

[K, prec, message] = place(A,B,desired_poles);       % Calculates Controller Gains

% Note: prec tells how closely the eignevalues of A-B*K match the spcified
% locations p (prec measures the number of accurate decimal digits in the
% actual closed-loop poles). If some nonzero closed-loop pole is more than
% 10% off from the desired location, "message" will contain a warning
% message.

%% Plot Closed Loop Result - Control Design via Transformation
% Similar to Ex. 12.4 out of the textbook
% Form the compensated State space matrices using the controller gain
Anew = A - B*K;     
Bnew = B;
Cnew = C;
Dnew = D;

Tss = ss(Anew, Bnew, Cnew, Dnew);   % Forms a LTI (Linear Time Invariant) State-Space Object
T = tf(Tss);                        % Create T(s) - Transformed Model;
T = minreal(T);                     % Cancel common terms and display T(s)

new_poles = pole(T);     % Displays the poles of T

% Plot the compensated step response of the closed-loop transfer function
figure(10);
step(Tss)
title('Compensated Step Response');

% Calculated parameters 
[Knew, prec_new, message_new] = place(Anew,Bnew,desired_poles);       % Calculates new Controller Gains - not really nec?

%% Filter Parameters
% SRV02 High-pass filter in PD control used to compute velocity
% Cutoff frequency (rad/s)
wcf_1 = 2 * pi * 10.0;
% Pendulum High-pass filter in PD control used to compute velocity
% Cutoff frequency (rad/s)
wcf_2 = 2 * pi * 10.0;
%
% Pulling data from SS Model
Alpha = C(2,2);
Theta = C(1,1);

% Tunable inverted pendulum angle
% Balance control enable range (rad)
% epsilon = 12.0 * pi / 180;

% Potential Energy of Pendulum (J) - When hanging in the downward position
Ep_hanging_position = 0;                %(1./2).*m_p.*g.*L_p.*(1 - cos(alpha))         

% Potential Energy of Pendulum (J) - When Standing Upright
Ep_upright_position = Mp.*g.*Lp;      %(1./2).*m_p.*g.*L_p.*(1 - cos(alpha))         alpha = pi -> upright position

% Kinetic Energy of Pendulum (J)
%Ek = (1./2).*J_p.*(alpha_dot.^2);

% Total Energy of Pendulum (J) - Break free for hanging resting position - 
% assume E = 0 because we are not moving much, just trying to get the 
% pendulum started moving so the swing up control begins to function, the
% swing up control will not start moving if the pendulum is motionless
E_tot = 0;

% Max Torque of the Motor - Based on SRV02 Motor Parameters from Quasner
T_max = (ng*Kg*nm*kt*(5))/Rm; 
% Note: Assumption is the max applied voltage to the DC motor is 5, hence
% where the 5 came from in the above equation, replacing Vm

% Maximum acceleration of pivot (m/s^2) - Needs max torque of motor
a_max = T_max/(Mr*Lr);

% Tunable Control Gain
mu = 1;

% Reference Energy (J)
Er = Ep_upright_position;

% Function that saturates the control signal at max acceleration of pendulum pivot (u_max)
sat_u_max = 20;

%% Display
disp( ' ' );
disp( 'Balance control gain: ' );
K
disp( 'Swing-up Parameters: ' );
disp( [ '   Er = ' num2str( Er ) ' J' ] );
disp( [ '   a_max = ' num2str( a_max ) ' m/s^2' ] );

%% Connecting MATLAB to Coppelia
% Note: 4 API files need to be in the directory of the folder for API's to
% correctly connect MATLAB to Coppelia, see documentation 

% Initialize API
sim=remApi('remoteApi');

% Using the prototype file (remoteApiProto.m)
sim.simxFinish(-1);

% Note: Just in case, close all opened connections
clientID=sim.simxStart('192.168.1.151',19999,true,true,5000,5);

if (clientID>-1)
    
    disp('Connected to remote API server');                     
    set_param('Furuta_Pendulum', 'SimulationCommand', 'start')               

    %sim.simxSetJointTargetVelocity(clientID,j1,pos_val,sim.simx_opmode_streaming);
    
    while(1)  % In this while loop, we will have the communication
        
        % Step 1: Initialize Joint and Link Handles where you defined your joints and links under the set coppelia names
        [err_code_1_object, Jr] = sim.simxGetObjectHandle(clientID,'Jr',sim.simx_opmode_blocking);
        [err_code_2_object, Jp] = sim.simxGetObjectHandle(clientID,'Jp',sim.simx_opmode_blocking);
        [err_code_3_object, Lr] = sim.simxGetObjectHandle(clientID,'Lr',sim.simx_opmode_blocking);
        [err_code_4_object, Lp] = sim.simxGetObjectHandle(clientID,'Lp',sim.simx_opmode_blocking);
                %%if errorCode is not vrep.simx_return_ok, this does not mean there is an error:            
                %%it could be that the first streamed values have not yet arrived, or that the signal            
                %%is empty/non-existent  
                
% Sensor Data from Coppelia
        [err_code_1_position, theta_Jr] = sim.simxGetJointPosition(clientID, Jr , sim.simx_opmode_streaming);
        [err_code_2_position, alpha_Jp] = sim.simxGetJointPosition(clientID, Jp , sim.simx_opmode_streaming);
        [err_code_1_velocity, linear_velo_theta, theta_dot_Jr] = sim.simxGetObjectVelocity(clientID, Lr ,sim.simx_opmode_streaming);
        [err_code_2_velocity, linear_velo_alpha, alpha_dot_Jp] = sim.simxGetObjectVelocity(clientID, Lp ,sim.simx_opmode_streaming);
        
        theta_dot_about_z = theta_dot_Jr(3);
        alpha_dot_about_x = alpha_dot_Jp(1);
        alpha_dot_about_y = alpha_dot_Jp(2);
        
                 pause(.01);
                
% Actuator Data from Simulink
            % We receive the sensor data from Simulink model 'Furuta_Pendulum' and 'To Workspace theta' block via RuntimeObject
            theta_s = get_param('Furuta_Pendulum/To Workspace theta','RuntimeObject');
            theta_s.InputPort(1).Data;    % Receive the data
            %simout_t = sim('Furuta_Pendulum/To Workspace theta','SimulationMode','normal', 'SaveState','on','StateSaveName','xoutNew','SaveOutput','on','OutputSaveName','youtNew');
            %theta_s = simout_t.get('youtNew');
            %assignin('base','theta_s',theta_s);

            
            % We receive the sensor data from Simulink model 'Furuta_Pendulum' and 'To Workspace theta' block via RuntimeObject
            alpha_s = get_param('Furuta_Pendulum/To Workspace alpha','RuntimeObject');
            alpha_s.InputPort(2).Data;    % Receive the data
            %simout_a = sim('Furuta_Pendulum/To Workspace alpha','SimulationMode','normal');
            %alpha_s = simout_a.get('alpha_s');
            %assignin('base','alpha_s',alpha_s);
            
% Coppelia motion dictated by Simulink model - Will uncomment when I manage to get simulink to write to the work space
        % Note: Unknown if this code is correct couldn't get past simulink 'to workspace' block error (dot indexing error) so couldn't get to debug coppellia sim motion
            [err_code_1_set_target_position] = sim.simxSetJointTargetPosition(clientID, Jr, theta_s, sim.simx_opmode_streaming);
            [err_code_2_set_target_velocity] = sim.simxSetJointTargetVelocity(clientID, Jr, theta_dot_s, sim.simx_opmode_streaming);
            
   end
end
