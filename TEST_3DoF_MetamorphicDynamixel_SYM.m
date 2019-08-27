clear;
clc;
close all;
% Makes Asada Test of 3 DoF Serial Metamorphic manipulator
% Asada propositions are examined
% Symbolic matrices extraction for 3 DoF MMD with 3x2 pseudojoints

%% load libraries
% load Murray kinematics
addpath('/home/nikos/matlab_ws/kinematics/robotlinks')
addpath('/home/nikos/matlab_ws/kinematics/screws') 
addpath('/home/nikos/matlab_ws/kinematics/util')
% load Js-Jb calculation function
addpath('/home/nikos/matlab_ws/project_ABBpaper/matlab_files')

%% Reference anatomy
% [robot1] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/reference_3DoF.urdf'); % with 3 Pseudo pairs
[robot1] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/reference_3DoF_2Pseudo.urdf'); % with 2 Pseudo pairs
% reference_figure = figure;
show(robot1)
robot1.DataFormat = 'column';
robot1.Gravity = [0 0 -9.80665];
ref_config = homeConfiguration(robot1);
robot_DoF = 3;
[robot0_links,CoM_robot0_links,gsli0,gsbj0,M0_CoM,M0_s_CoM] = robot_links_subtree_new(robot1,ref_config,robot_DoF);

%%  Extract Geometry points for each body from FKP in reference anatomy and zero configuration!!!
robot = robot1;
config = [0 0 0]'; 
base_link = char(robot.BodyNames(1));
Ts1 = getTransform(robot,config,base_link);
% drawframe(Ts1, scale) 
% hold on;
adaptor = char(robot.BodyNames(2));
Ts2 = getTransform(robot,config,adaptor);
% drawframe(Ts2, scale) 
% hold on;
frame = char(robot.BodyNames(3));
Ts3 = getTransform(robot,config,frame);
% drawframe(Ts3, scale) 
% hold on;
PseudoConnector1a = char(robot.BodyNames(4));
Ts4 = getTransform(robot,config,PseudoConnector1a);
% text(Ts2(1,4),Ts2(2,4),Ts2(3,4),'\leftarrow AM1');
% drawframe(Ts4, scale_active,'true') % Active joint1
% hold on;
PseudoConnector1b = char(robot.BodyNames(5));
Ts5 = getTransform(robot,config,PseudoConnector1b);
% drawframe(Ts5, scale) 
% hold on;
PseudoConnector2a = char(robot.BodyNames(6));
Ts6 = getTransform(robot,config,PseudoConnector2a);
% drawframe(Ts6, scale)
% hold on;
PseudoConnector2b = char(robot.BodyNames(7));
Ts7 = getTransform(robot,config,PseudoConnector2b);
% drawframe(Ts7, scale)
% hold on;
adaptor1 = char(robot.BodyNames(8));
Ts8 = getTransform(robot,config,adaptor1);
% drawframe(Ts8, scale)
% hold on;
AM1 = char(robot.BodyNames(9));
Ts9 = getTransform(robot,config,AM1);
% drawframe(Ts9, scale)
% hold on;
frame1 = char(robot.BodyNames(10));
Ts10 = getTransform(robot,config,frame1);
% drawframe(Ts10, scale_active,'true') % Active joint2
% hold on;
idler1 = char(robot.BodyNames(11));
Ts11 = getTransform(robot,config,idler1);
% drawframe(Ts11, scale)
% hold on;
PseudoConnector3a = char(robot.BodyNames(12));
Ts12 = getTransform(robot,config,PseudoConnector3a);
% drawframe(Ts12, scale)
% hold on;
PseudoConnector3b = char(robot.BodyNames(13));
Ts13 = getTransform(robot,config,PseudoConnector3b);
% drawframe(Ts13, scale) ;
% hold on;
PseudoConnector4a = char(robot.BodyNames(14));
Ts14 = getTransform(robot,config,PseudoConnector4a);
% drawframe(Ts14, scale)
% hold on;
PseudoConnector4b = char(robot.BodyNames(15));
Ts15 = getTransform(robot,config,PseudoConnector4b);
% drawframe(Ts15, scale)
% hold on;
adaptor2 = char(robot.BodyNames(16));
Ts16 = getTransform(robot,config,adaptor2);
% drawframe(Ts16, scale)
% hold on;
AM2 = char(robot.BodyNames(17));
Ts17 = getTransform(robot,config,AM2);
% drawframe(Ts17, scale)
% hold on;
frame2 = char(robot.BodyNames(18));
Ts18 = getTransform(robot,config,frame2);
% drawframe(Ts18, scale_active,'true') % Active joint3
% hold on;
idler2 = char(robot.BodyNames(19));
Ts19 = getTransform(robot,config,idler2);
% drawframe(Ts19, scale)
% hold on;
%% Pi points are extracted from reference Anatomy in zero configuration!!!
pi(:,1)= Ts4(1:3,4); % p1 on ξa1
pi(:,2)= Ts10(1:3,4); % p2 on ξa2
pi(:,3)= Ts5(1:3,4); % p3 on ξp1
pi(:,4)= Ts7(1:3,4); % p4 on ξp2

%% Active screws
wi(:,1) = [0 0 1]';
wi(:,2) = [0 1 0]';
wi(:,3) = [0 1 0]';
%% Pseudo screws as viewd for tpi = 0!!!
wpi(:,1) = [0 1 0]';
wpi(:,2) = [1 0 0]';
wpi(:,3) = [0 1 0]';
wpi(:,4) = [1 0 0]';

%% Active twists - reference anatomy
xi_ai(:,1) = createtwist(wi(:,1),pi(:,1));
xi_ai(:,2) = createtwist(wi(:,2),pi(:,2));
% xi_ai = sym('xi',[6 2], 'real');
%% Pseudo twists
xi_pi(:,1) = createtwist(wpi(:,1),pi(:,3));
xi_pi(:,2) = createtwist(wpi(:,2),pi(:,4));


