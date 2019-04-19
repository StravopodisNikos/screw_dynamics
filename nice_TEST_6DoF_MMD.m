clear;
clc;
close all;

%% Add Libraries

%% End libraries call

%% Import robot models
RefRobotName = '/home/nikos/matlab_ws/modular_dynamixel/reference_MMD_inverse.urdf';
[RefRobot,RefFig,RefConfig,NumDoF] = ImportRobotRefAnatomyModel(RefRobotName);

TestRobotName = '/home/nikos/matlab_ws/modular_dynamixel/x_test_inverse_MMD1.urdf';
[TestRobot,TestFig,TestConfig,NumDoF] = ImportRobotTestAnatomyModel(TestRobotName);

%% Breaks inital robot models to links
[RefRobot_links,CoM_RefRobot_links,gsli0,gsbj0,M0b_CoM,M0s_CoM] = robot_links_subtree_new(RefRobot,RefConfig,NumDoF);

[TestRobot_links,CoM_TestRobot_links,gsli1,gsbj1,M1b_CoM,M1s_CoM] = robot_links_subtree_new(TestRobot,TestConfig,NumDoF);
