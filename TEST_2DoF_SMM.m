
% This file originates from /screw_dynamics/TEST_2DoF_MetamorphicDynamixel.
% It is used to extract symblolics for equations used un Chapter for SMM Dynamics

clear;
clc;
close all;

% load Murray kinematics
% load Murray kinematics
addpath('/home/nikos/matlab_ws/screw_kinematics_library/screws')
addpath('/home/nikos/matlab_ws/screw_kinematics_library/util')
addpath('/home/nikos/matlab_ws/screw_dynamics')

%% Show reference anatomy robot
% urdf files are build from
% 2DoF_MOdular_Metamorphic_Dynamixel_MMD_for_dynamics.xacro file
[robot1] = importrobot('/home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/xacros/2DoF_for_ChapterDynamics/smm2dof.urdf'); 
reference_figure = figure;
show(robot1), hold on;
axis auto;
% hold on;
robot1.DataFormat = 'column';
%robot1.Gravity = [0 0 -9.80665];
showdetails(robot1);
ref_config = homeConfiguration(robot1);

[robot0_links,CoM_robot0_links,gsli0,gsbj0,M0_CoM,M0_s_CoM] = robot_links_subtree_new(robot1,ref_config,2);
%% Get geometry points in reference anatomy and configuration
robot = robot1;
config = ref_config; 
base_link = char(robot.BodyNames(1));
Ts1 = getTransform(robot,config,base_link);
pseudo1a = char(robot.BodyNames(2));
Ts2 = getTransform(robot,config,pseudo1a);
pseudo1b = char(robot.BodyNames(3));
Ts3 = getTransform(robot,config,pseudo1b);
pseudo2a = char(robot.BodyNames(4));
Ts4 = getTransform(robot,config,pseudo2a);
pseudo2b = char(robot.BodyNames(5));
Ts5 = getTransform(robot,config,pseudo2b);
active_module1a = char(robot.BodyNames(6));
Ts6 = getTransform(robot,config,active_module1a);
active_module1b = char(robot.BodyNames(7));
Ts7 = getTransform(robot,config,active_module1b);
TOOL = char(robot.BodyNames(8));
Ts8 = getTransform(robot,config,TOOL);
%% Inverse Kinematic Problem-Analytical Solution
% tpi_ref = [0 0]';% Reference Anatomy values are theta_p_i's specified in MMDinertia_scale_xacro_FixedAnatomy.xacro and in the urdf built from it and given in l.10
tpi = [0 0]'; % desired anatomy

% Only for symbolic
% % tpi_ref = sym('tp0', [1 2]);
%tpi = sym('tp', [1 2]);
% % ref_config = sym('t0', [1 2]);

%% Active screws
wi(:,1) = [0 0 1]';
wi(:,2) = [0 1 0]';
%% Pseudo screws as viewd for tpi = 0!!! Not the reference anatomy!!!
wpi(:,1) = [1 0 0]';
wpi(:,2) = [0 1 0]';
%% Pi points are extracted from reference Anatomy in zero configuration!!!
pi(:,1)= Ts1(1:3,4); % p1 on ξa1
pi(:,2)= Ts7(1:3,4); % p2 on ξa2
pi(:,3)= Ts3(1:3,4); % p3 on ξp1
pi(:,4)= Ts5(1:3,4); % p4 on ξp2
pi(:,5)= Ts8(1:3,4); % 2DoF_TCP
%% Active twists - reference anatomy
xi_ai(:,1) = createtwist(wi(:,1),pi(:,1));
xi_ai(:,2) = createtwist(wi(:,2),pi(:,2));
%% Pseudo twists
xi_pi(:,1) = createtwist(wpi(:,1),pi(:,3));
xi_pi(:,2) = createtwist(wpi(:,2),pi(:,4));
%% Pseudo exponenentials (can be calculated since anatomy is specified by user)
% exp_pi_ref(:,:,1) = twistexp(xi_pi(:,1), tpi_ref(1));
% exp_pi_ref(:,:,2) = twistexp(xi_pi(:,2), tpi_ref(2));
%% Specify 0tfs(zero configuration and reference anatomy)
gs10 = [ 1.0000         0         0         0;...
              0    1.0000         0         0;...
              0         0    1.0000         0;...
              0         0         0    1.0000];
gs20 = [ 0.0000    1.0000   -0.0000    0.1750;...
        -0.0000    0.0000    1.0000    0.0250;...
         1.0000   -0.0000    0.0000    0.3354;...
              0         0         0    1.0000];          
gst0 = [     0.0000    1.0000   -0.0000    0.2250;...
             -0.0000    0.0000    1.0000    0.0250;...
              1.0000   -0.0000    0.0000    0.3354;...
                  0         0         0    1.0000];
config = [0 0]';
% % config = sym('t', [1 2]);
%% POE FKP
[g1,g2,gst] = MMD_POE_FKP_2DoF(xi_ai,xi_pi,config,tpi,gst0,gs10,gs20);

%% Calculate Jacobians
% gd1 = getTransform(robot2,config,idler1);

[Js, Jb, ~] = Jacobians_MMD_2DoF(xi_ai, xi_pi, config, tpi, gst0, gst); 
figure(reference_figure);
xi1_ref = drawtwist(Js(:,1)); hold on
xi2_ref = drawtwist(Js(:,2)); hold on

% [Js_test, Jb_test, Error_test] = Jacobians_MMD_2DoF(nxi_ai, config, gst0, gd_ref);

% [met_Js, met_Jb, Error] = Jacobians_MMD(xi_ai, config, xi_pi, tpi, gst0, gd_test);
% figure(test_figure);
% show(robot2,config), hold on;
% xi1_test = drawtwist(met_Js(:,1)); hold on
% xi2_test = drawtwist(met_Js(:,2)); hold on

%% Solve IKP
% % nJs = 0; % not used now
% % angles2 = inverse_2DoF_MMD(gd,npi,xi_ai,nJs,xi_pi,tpi,exp_pi,reference_figure,test_figure,gst0);
% % gd_test1 = getTransform(robot2,angles2(:,1),idler1);
% % error1 = gd-gd_test1 % good
% % gd_test2 = getTransform(robot2,angles2(:,2),idler1);
% % error2 = gd-gd_test2 % good
% % gd_test3 = getTransform(robot2,angles2(:,3),idler1);
% % error3 = gd-gd_test3
% % gd_test4 = getTransform(robot2,angles2(:,4),idler1);
% % error4 = gd-gd_test4
%% Dynamics from MATLAB
% MAX_Joint_Vel = [3.3510 3.3510]';
% MAX_Joint_Accel = [10 10]';
MAX_Joint_Vel = [1 1]';
MAX_Joint_Accel = [10 10]';
% Referece anatomy
figure(reference_figure);
show(robot1);
axis auto;
hold on;
gravTorq_ref = gravityTorque(robot1,config);
jointTorq_ref = inverseDynamics(robot1,config,MAX_Joint_Vel,MAX_Joint_Accel);
% Test anatomy
% figure(test_figure);
% show(robot2);
% axis auto;
% hold on;
% gravTorq_test = gravityTorque(robot2,config);
% jointTorq_test = inverseDynamics(robot2,config,MAX_Joint_Vel,MAX_Joint_Accel);
%% Dynamics from twists and POE
%% Step 1.
%   1. Define CoM coordinate systems for reference anatomy-configuration
gsli0(:,:,1) = [     1     0     0     CoM_robot0_links(1,1);...
                     0     1     0     CoM_robot0_links(2,1);...
                     0     0     1     CoM_robot0_links(3,1);...
                     0     0     0     1];
gsli0(:,:,2) = [  1.0000         0         0         CoM_robot0_links(1,2);...
                       0    1.0000         0         CoM_robot0_links(2,2);...
                       0         0    1.0000         CoM_robot0_links(3,2);...
                       0         0         0         1.0000];  
                   
%   2. FKP for CoM coordinate systems
%   Active exponentials
exp_ai(:,:,1) = twistexp(xi_ai(:,1), config(1));
exp_ai(:,:,2) = twistexp(xi_ai(:,2), config(2));
%   Passive exponentials
exp_pi(:,:,1) = twistexp(xi_pi(:,1), tpi(1));
exp_pi(:,:,2) = twistexp(xi_pi(:,2), tpi(2));
Pi1 = exp_pi(:,:,1)*exp_pi(:,:,2);
% Pi1_ref = exp_pi_ref(:,:,1)*exp_pi_ref(:,:,2);

%   3. FK mappings
gsli(:,:,1) = sym(zeros(4));
gsli(:,:,2) = sym(zeros(4));
gsli(:,:,1) = exp_ai(:,:,1)*gsli0(:,:,1);
gsli(:,:,2) = exp_ai(:,:,1)*Pi1*exp_ai(:,:,2)*gsli0(:,:,2);
%% Step 2.
%  1. Define body velocities of CoM, through Body Jacobian
i = 1;
[Jbsli(:,:,i),Jbsli_POE(:,:,i)] = Jbody_CoM_2DoF(xi_ai, exp_ai, Pi1, gsli0, i);
% [Jbsli_ref(:,:,i), Jssli_ref(:,:,i), Jbsli_ref_POE(:,:,i)] = Jbody_CoM_2DoF(xi_ai, exp_ai, Pi1_ref, gsli0, gsli, i);

i = 2;
[Jbsli(:,:,i),Jbsli_POE(:,:,i)] = Jbody_CoM_2DoF(xi_ai, exp_ai, Pi1, gsli0, i);
% [Jbsli_ref(:,:,i),Jssli_ref(:,:,i), Jbsli_ref_POE(:,:,i)] = Jbody_CoM_2DoF(xi_ai, exp_ai, Pi1_ref, gsli0, gsli, i);

%% Step 3.
%  1. Define the generalized inertia matrix for each link, based on Murray p.163

% Symbolic matrices are used to symbolically define the equations
% % mass1 = 0.855; % [kg]
% % mass1 = mass1.*eye(3);
% % % Inertia1 = [4.0487283e+06 1.6444686e+04 1.4186275e+05;...
% % %             1.6444686e+04 4.0350489e+06 -2.1783575e+05;...
% % %             1.4186275e+05 -2.1783575e+05 3.4893573e+05]; % [g*mm^2]
% % % Inertia1 = 1e-09.*Inertia1;   % [kg*m^2]
% % Inertia1 = sym('I1',[3 3]);
% % M1 = [mass1 zeros(3); zeros(3) Inertia1];        
% % 
% % % Properties of pseudojoints
% % mass2 = 4; %[kg]
% % mass2 = mass2.*eye(3);
% % Inertia2 = sym('I2',[3 3]);
% % M2 = [mass2 zeros(3); zeros(3) Inertia2]; 

% 2. Calculate Manipulator Inertia, Coriolis and Gravity Matrix
% must be: M1(2,2) == M2(2,2)
theta_dot = MAX_Joint_Vel;
theta_dotdot = MAX_Joint_Accel;
% % theta_dot = sym('dt', [2 1]);
% % theta_dotdot = sym('ddt', [2 1]);
Mi(:,:,1)  =  M0_CoM(:,:,1); % link inertia frame
Mi(:,:,2)  = M0_CoM(:,:,2);
Mi_s(:,:,1)  = M0_s_CoM(:,:,1); % spatial frame
Mi_s(:,:,2)  = M0_s_CoM(:,:,2);
% Jis = Jssli_ref;
Jib = Jbsli;

% % Ji(:,1,1) = sym('Ji01', [6 1]);
% % Ji(:,2,1) = zeros(6,1);
% % Ji(:,:,2) = sym('Ji02', [6 2]);

[M0b, M0_POE, dM0_POE, C0_POE] = manipulator_inertia_matrix(xi_ai, exp_ai, Pi1 ,gsli0, Mi, Jib, theta_dot);
[N0_POE] = calculate_gravity_matrix(xi_ai, gsli0,[Mi(1,1,1) Mi(1,1,2)]',  Pi1, config);
M0m = massMatrix(robot1,config);
tau_C0m = velocityProduct(robot1,config,theta_dot);
tau_C0_POE = C0_POE*theta_dot;
% % % PD computed torque control
% % qd = [0.1 -0.1]';
% % dqd = [0 0]';
% % ddqd = [1 -1]';
% % [tvec,dy] = control_2DoF_Metamorphic_Dynamixel(M0_POE,C0_POE,N0_POE,qd,dqd,ddqd);

% Mi(:,:,1)  = M0_CoM(:,:,1); % link inertia frame
% Mi(:,:,2)  = M0_CoM(:,:,2);
% Mi_s(:,:,1)  = M0_s_CoM(:,:,1); % spatial frame
% Mi_s(:,:,2)  = M0_s_CoM(:,:,2);
% Jib = Jbsli;
% Jis = Jssli;
% [M2s, M2b, M2_POE, dM2_POE, C2_POE] = manipulator_inertia_matrix(xi_ai, exp_ai, Pi1, gsli0, Mi, Mi_s, Jis, Jib, theta_dot);
% [N2_POE] = calculate_gravity_matrix(xi_ai, gsli0,[Mi(1,1,1) Mi(1,1,2)]',  Pi1, config);
% M2m = massMatrix(robot2,config);
%  
% 3. Calculate torque
joint_torque0 = M0b*theta_dotdot + C0_POE*theta_dot + N0_POE;
dyn1_error = jointTorq_ref - joint_torque0; % Matlab - My_POE_Way
% joint_torque2 = M2b*theta_dotdot + C2_POE*theta_dot + N2_POE;
% dyn2_error = jointTorq_test - joint_torque2;