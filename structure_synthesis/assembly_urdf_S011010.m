clear;
clc;
close all;
%% Metamorphic manipulator structure for test: kinematic_verification_S010110.m
%% Structure: 0 - 1 1 0 - 1 0
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
% robot urdf is built from: /home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/S011010.xacro
% [robot1] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/structure_max_Wk1.urdf');  
[robot1] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/S011010_POE_DH_conv.urdf');
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

%% First STRUCTURAL BLOCK: 110
adaptor = char(robot1.BodyNames(2)); Ts2 = getTransform(robot1,config,adaptor); drawframe(Ts2, scale); hold on;
frame = char(robot1.BodyNames(3)); Ts3 = getTransform(robot1,config,frame); drawframe(Ts3, scale_active); hold on;
PseudoConnector1a = char(robot1.BodyNames(4)); Ts4 = getTransform(robot1,config,PseudoConnector1a); drawframe(Ts4, scale); hold on;
PseudoConnector1b = char(robot1.BodyNames(5)); Ts5 = getTransform(robot1,config,PseudoConnector1b); drawframe(Ts5, scale); hold on;
PseudoConnector2a = char(robot1.BodyNames(6)); Ts6 = getTransform(robot1,config,PseudoConnector2a); drawframe(Ts6, scale); hold on;
PseudoConnector2b = char(robot1.BodyNames(7)); Ts7 = getTransform(robot1,config,PseudoConnector2b); drawframe(Ts7, scale); hold on;

frame1 = char(robot1.BodyNames(10)); Ts10 = getTransform(robot1,config,frame1); drawframe(Ts10, scale); hold on;
TOOL = char(robot1.BodyNames(11)); Ts11 = getTransform(robot1,config,TOOL); drawframe(Ts11, scale); hold on;

%% Second STRUCTURAL BLOCK: 10
frame = char(robot1.BodyNames(3)); Ts3 = getTransform(robot1,config,frame); drawframe(Ts3, scale); hold on;
PseudoConnector1a = char(robot1.BodyNames(4)); Ts4 = getTransform(robot1,config,PseudoConnector1a); drawframe(Ts4, scale); hold on;
frame1 = char(robot1.BodyNames(10)); Ts10 = getTransform(robot1,config,frame1); drawframe(Ts10, scale); hold on;
TOOL = char(robot1.BodyNames(11)); Ts11 = getTransform(robot1,config,TOOL); drawframe(Ts11, scale); hold on;
PseudoConnector1a_2 = char(robot1.BodyNames(12)); Ts12 = getTransform(robot1,config,PseudoConnector1a_2); drawframe(Ts12, scale); hold on;
PseudoConnector1b_2 = char(robot1.BodyNames(13)); Ts13 = getTransform(robot1,config,PseudoConnector1b_2); drawframe(Ts13, scale); hold on;
frame1_2 = char(robot1.BodyNames(16)); Ts16 = getTransform(robot1,config,frame1_2); drawframe(Ts16, scale); hold on;
TOOL_2 = char(robot1.BodyNames(17)); Ts17 = getTransform(robot1,config,TOOL_2); drawframe(Ts17, scale); hold on;


%% Evaluation tf's for S010110
% g_s_frame = getTransform(robot1,config,frame);
% g_s_PseudoConnector1a = getTransform(robot1,config,PseudoConnector1a);
% g_s_frame1 = getTransform(robot1,config,frame1); %must = S2.gsn(:,:,2)
% g_s_frame1_2 = getTransform(robot1,config,frame1_2); %must = S3.gsn(:,:,3)
% g_s_TOOL = getTransform(robot1,config,TOOL); % must = NEW_t0_FRAME in SB10
% g_s_PseudoConnector1a_2 = getTransform(robot1,config,PseudoConnector1a_2);%must = S2.Cg
% g_s_PseudoConnector1b_2 = getTransform(robot1,config,PseudoConnector1b_2);%must = S2.Cg
% g_s_PseudoConnector2b_2 = getTransform(robot1,config,PseudoConnector2b_2);%must = S2.Cg
% g_s_TOOL_2 = getTransform(robot1,config,TOOL_2); % must = NEW_t0_FRAME in SB110
% 
% gcorrect = getTransform(robot1,config,PseudoConnector1a_2,TOOL);
% 
% gslk1 = g_s_PseudoConnector1a;
% gslk2_local = inv(g_s_TOOL)*g_s_PseudoConnector1a_2;
% gslk2 = g_s_PseudoConnector1a_2;
% 
% %% call AUTO GA assembly
% [g,Js] = kinematic_verification_S010110(reference_figure);
%%
%% for S011010
g_s_frame = getTransform(robot1,config,frame);
g_s_frame1 = getTransform(robot1,config,frame1); %must = S2.gsn(:,:,2)
g_s_frame1_2 = getTransform(robot1,config,frame1_2); %must = S3.gsn(:,:,3)
g_s_TOOL = getTransform(robot1,config,TOOL); % must = NEW_t0_FRAME in SB10
g_s_PseudoConnector1a = getTransform(robot1,config,PseudoConnector1a);%must = S2.Cg
g_s_PseudoConnector1b = getTransform(robot1,config,PseudoConnector1b);%must = S2.Cg

% g_s_PseudoConnector1b_2 = getTransform(robot1,config,PseudoConnector1b_2);%must = S2.Cg
g_s_TOOL_2 = getTransform(robot1,config,TOOL_2); % must = S3.Rk

g_rel2 = getTransform(robot1,config,frame1,frame); 
g_rel3 = getTransform(robot1,config,frame1_2,frame1); 

gl2lj3 = getTransform(robot1,config,PseudoConnector1b_2,frame1); % must gn3
glj3l3 = getTransform(robot1,config,frame1_2,PseudoConnector1b_2); % must gn4
% gcorrect = getTransform(robot1,config,PseudoConnector1a_2,TOOL);
% 
% gslk1 = g_s_PseudoConnector1a;
% gslk2_local = inv(g_s_TOOL)*g_s_PseudoConnector1a_2;
% gslk2 = g_s_PseudoConnector1a_2;

%% call AUTO GA assembly
[g,grel,Js] = structure_assembly_GA(reference_figure);
ga2_error = g(:,:,2)-g_s_frame1; % 1.0e-15
ga3_error = g(:,:,3)-g_s_frame1_2; % 1.0e-5
gtool_error = g(:,:,4)-g_s_TOOL_2;
grel2_error = grel(:,:,2) - g_rel2;
grel3_error = grel(:,:,3) - g_rel3;
%% Extract Modified DH params from POE
[DH] = POE2DH_twists_geometry(g,grel,Js);