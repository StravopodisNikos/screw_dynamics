clear;
clc;
close all;

%% Add Libraries
% load Murray kinematics
addpath('/home/nikos/matlab_ws/kinematics/robotlinks')
addpath('/home/nikos/matlab_ws/kinematics/screws') 
addpath('/home/nikos/matlab_ws/kinematics/util')
% load Js-Jb calculation function
addpath('/home/nikos/matlab_ws/project_ABBpaper/matlab_files')
%% End libraries call

%% Import robot models
RefRobotName = '/home/nikos/matlab_ws/modular_dynamixel/reference_MMD_inverse.urdf';
[RefRobot,RefFig,RefConfig,NumDoF] = ImportRobotRefAnatomyModel(RefRobotName);

TestRobotName = '/home/nikos/matlab_ws/modular_dynamixel/x_test_inverse_MMD1.urdf';
[TestRobot,TestFig,TestConfig,NumDoF] = ImportRobotTestAnatomyModel(TestRobotName);

%% Breaks inital robot models to links
[RefRobot_links,CoM_RefRobot_links,gsli0,gsbj0,M0b_CoM,M0s_CoM] = robot_links_subtree_new(RefRobot,RefConfig,NumDoF);
[TestRobot_links,CoM_TestRobot_links,gsli1,gsbj1,M1b_CoM,M1s_CoM] = robot_links_subtree_new(TestRobot,TestConfig,NumDoF);

%%  Extract Geometry points for each body from FKP 
%   in reference anatomy and zero configuration!!!
[pi_ref,gst0] = ExtractGeometryPoints(RefRobot,RefConfig);
[pi_test,gst1] = ExtractGeometryPoints(TestRobot,TestConfig);

%% Specify reference anatomy-With regard to URDF Pseudo Configuration!!!
tpi_ref = [0 0 0 0 0 0]';% Reference Anatomy values are theta_p_i's specified in MMDinertia_scale_xacro_FixedAnatomy.xacro and in the urdf built from it and given in l.10
%% Specify test anatomy-With regard to Matlab Pseudo Configuration!!!
% Always write the corresponding urdf file name
tpi1 = [0.7854 0.7854 0 -0.7854 1.5708 -0.7854]'; % For robot file: x_test_inverse_MMD1.urdf

%% Define twists for active-pseudo and exponentials only for pseudo
[wi,wpi,xi_ai,xi_pi,exp_pi_ref,exp_pi_test,Pi_ref,Pi_test] = ScrewDefinitions_6DoF_MMD(pi_ref,pi_test,tpi_ref,tpi1);

%% Calculate CoM Jacobians-Recalculate for every configuration
[Jbsli_ref,Jssli_ref,Jbsli_POE_ref,gsli_ref] = calculate_CoM_BodyJacobians_6DoF_MMD(RefConfig,xi_ai,Pi_ref,gsli0);
[Jbsli_test,Jssli_test,Jbsli_POE_test,gsli_test] = calculate_CoM_BodyJacobians_6DoF_MMD(TestConfig,xi_ai,Pi_test,gsli0);

%% IKP solver
%  1. Specify wanted tf
% For MMD IKP solver gst0 is Ts33 in reference anatomy+configuration
TOOL_ref = char(RefRobot.BodyNames(33));
TOOL_test = char(TestRobot.BodyNames(33));
gd_ref = getTransform(RefRobot,RefConfig,TOOL_ref); 
gd_ref_POE = MMD_POE_FKP(RefConfig,xi_ai,tpi_ref,xi_pi,gst0);
gd_test = getTransform(TestRobot,TestConfig,TOOL_test); 
gd_test_POE = MMD_POE_FKP(TestConfig,xi_ai,tpi1,xi_pi,gst0);
gd = gd_test_POE; % User defined desired cartesian position and orientation
% 2. MATLAB IKP solver
RefIKS = robotics.InverseKinematics('RigidBodyTree',RefRobot); % create matlab solver
[RefConfigSol,RefSolInfo] = RefIKS('TOOL',gd,[0.01 0.01 0.01 0.01 0.01 0.01]',[0 0 0 0 0 0]');
TestIKS = robotics.InverseKinematics('RigidBodyTree',TestRobot); % create matlab solver
[TestConfigSol,TestSolInfo] = TestIKS('TOOL',gd,[0.01 0.01 0.01 0.01 0.01 0.01]',[0 0 0 0 0 0]');
% 3. POE solvers using screw Theory only for 3DoF - to be developed next
% row spherical wrist last 3 axes

%% IDP Solver
[Jbsli_ref,Jssli_ref,Jbsli_POE_ref,gsli_ref] = calculate_CoM_BodyJacobians_6DoF_MMD(RefConfigSol,xi_ai,Pi_ref,gsli0);
[Jbsli_test,Jssli_test,Jbsli_POE_test,gsli_test] = calculate_CoM_BodyJacobians_6DoF_MMD(TestConfigSol,xi_ai,Pi_test,gsli0);

theta_dot = [1 0 0 0 0 0]';
theta_dotdot = [10 0 0 0 0 0]';

[M0_POE,V0_POE,N0_POE,M0m,V0m,N0m,JointTorque0_POE,JointTorq0M] = InverseDynamicProblemPOE_6DoF_MMD(RefRobot,RefConfigSol,theta_dot,theta_dotdot,M0b_CoM,M0s_CoM,Jssli_ref,Jbsli_ref,xi_ai, Pi_ref, gsli0);
[M1_POE,V1_POE,N1_POE,M1m,V1m,N1m,JointTorque1_POE,JointTorq1M] = InverseDynamicProblemPOE_6DoF_MMD(TestRobot,TestConfigSol,theta_dot,theta_dotdot,M0b_CoM,M0s_CoM,Jssli_test,Jbsli_test,xi_ai, Pi_test, gsli0);
