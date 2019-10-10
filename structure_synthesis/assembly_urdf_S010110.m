clear;
clc;
close all;
%% Metamorphic manipulator structure for test: kinematic_verification_S010110.m
%% Structure: 0 - 1 0 - 1 1 0
%% Active1 - C01 - Passive1 - Active2 -  C01 - Passive2 - Passive3 - Active3
%% C01: 6 Dof connection between active(0) and passive(1) link
%% C01: Px,Py,Pz,Rx,Ry,Rz the tf between frame-PseudoConnector1a rigid bodies

%% load libraries
% load Murray kinematics
addpath('/home/nikos/matlab_ws/kinematics/robotlinks')
addpath('/home/nikos/matlab_ws/kinematics/screws') 
addpath('/home/nikos/matlab_ws/kinematics/util')
% load Js-Jb calculation function
addpath('/home/nikos/matlab_ws/project_ABBpaper/matlab_files')
% load main folder
addpath('/home/nikos/matlab_ws/modular_dynamixel/')
% load geom3d library
addpath('/home/nikos/matlab_ws/geom3d')
addpath('/home/nikos/matlab_ws/geom3d/geom3d')
addpath('/home/nikos/matlab_ws/geom2d/geom2d')
addpath('/home/nikos/matlab_ws/geom2d/utils')
%% Reference anatomy
% robot urdf is built from: /home/nikos/matlab_ws/modular_dynamixel/kinematic_verification_01.xacro
[robot1] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/S010110_1.urdf');  
robot1.DataFormat = 'column';
robot1.Gravity = [0 0 -9.80665];
config = homeConfiguration(robot1);
reference_figure = figure;
%% Show reference C01
figure(reference_figure);
show(robot1,config,'PreservePlot',false);
hold on;
axis auto;
box on;
% robot_DoF = 1;
%% Zero Structure-Anatomy-Configuration tfs
%% all pi,wi for twists,expos creation must be calculated from this
%% AND recalculated depending the synthetic variables!!!
scale = 0.025; scale_active = 0.075;

%% First STRUCTURAL BLOCK: 10
adaptor = char(robot1.BodyNames(2)); Ts2 = getTransform(robot1,config,adaptor); drawframe(Ts2, scale); hold on;
frame = char(robot1.BodyNames(3)); Ts3 = getTransform(robot1,config,frame); drawframe(Ts3, scale_active); hold on;
PseudoConnector1a = char(robot1.BodyNames(4)); Ts4 = getTransform(robot1,config,PseudoConnector1a); drawframe(Ts4, scale); hold on;
PseudoConnector1b = char(robot1.BodyNames(5)); Ts5 = getTransform(robot1,config,PseudoConnector1b); drawframe(Ts5, scale); hold on;
frame1 = char(robot1.BodyNames(8)); Ts8 = getTransform(robot1,config,frame1); drawframe(Ts8, scale); hold on;
TOOL = char(robot1.BodyNames(9)); Ts9 = getTransform(robot1,config,TOOL); drawframe(Ts9, scale); hold on;

%% Second STRUCTURAL BLOCK: 110
frame = char(robot1.BodyNames(3)); Ts3 = getTransform(robot1,config,frame); drawframe(Ts3, scale); hold on;
frame1 = char(robot1.BodyNames(8)); Ts8 = getTransform(robot1,config,frame1); drawframe(Ts8, scale); hold on;
frame1_2 = char(robot1.BodyNames(16)); Ts16 = getTransform(robot1,config,frame1_2); drawframe(Ts16, scale); hold on;
TOOL_2 = char(robot1.BodyNames(17)); Ts17 = getTransform(robot1,config,TOOL_2); drawframe(Ts17, scale); hold on;


%% Evaluation tf's
g_s_frame = getTransform(robot1,config,frame)
g_s_frame1 = getTransform(robot1,config,frame1)
g_s_frame1_2 = getTransform(robot1,config,frame1_2)
% g_s_TOOL_2 = getTransform(robot1,config,TOOL_2)