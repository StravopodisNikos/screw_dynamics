clear;
clc;
close all;

% load Murray kinematics
addpath('/home/nikos/matlab_ws/kinematics/robotlinks')
addpath('/home/nikos/matlab_ws/kinematics/screws') 
addpath('/home/nikos/matlab_ws/kinematics/util')
% load Js-Jb calculation function
addpath('/home/nikos/matlab_ws/project_ABBpaper/matlab_files')

%% Show reference anatomy robot
% % [robot1] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/reference_MMD_inverse.urdf'); 
% % % we use 12 DoF for evaluation of matlab-pseudo-configuration
% % % [robot1] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/MMDinertia_scale.urdf'); 
% % 
% % reference_figure = figure;
% % show(robot1);
% % axis auto;
% % % hold on;
% % robot1.DataFormat = 'column';
% % robot1.Gravity = [0 0 -9.80665];
% % showdetails(robot1);
% % ref_config = homeConfiguration(robot1);

robotname = '/home/nikos/matlab_ws/modular_dynamixel/reference_MMD_inverse.urdf';
[RefRobot,RefFig,RefConfig,NumDoF] = ImportRobotRefAnatomyModel(robotname);
[robot0_links,CoM_robot0_links,gsli0,gsbj0,M0_CoM,M0_s_CoM] = robot_links_subtree_new(robot1,ref_config,6);
%% Test configuration
% Change configuration for 12 DoF structure. Fast Display of metamorphic
% anatomies. Can be used only for MMDinertia.urdf & MMDinertia_scale.urdf
% ref_config(1) = 0; % Active1
% ref_config(2) = 0; % Passive1 test
% ref_config(3) = 0; % Passive2
% ref_config(4) = 0; % Active2
% ref_config(5) = 0; % Passive3
% ref_config(6) = 0; % Passive4
% ref_config(7) = 0; % Active3
% ref_config(8) = 0.7854; % Passive5 test
% ref_config(9) = 0.7854; % Passive6
% ref_config(10) = 0; % Active4
% ref_config(11) = 0; % Active5
% ref_config(12) = 0; % Active6

% Show configuration in reference anatomy
figure(reference_figure);
show(robot1,ref_config,'PreservePlot',false);
hold on;
axis auto;
box on;

%% Show test-anatomy robot
test_figure = figure;

% [robot] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/test_MMD_inverse.urdf'); 
% [robot2] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/x_anatII.urdf');
% [robot2] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/x_test_inverse_MMD.urdf');
[robot2] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/x_test_inverse_MMD1.urdf');

show(robot2);
axis auto;
hold on;
robot2.DataFormat = 'column';
robot2.Gravity = [0 0 -9.80665];
showdetails(robot2);
config = homeConfiguration(robot2);
[robot2_links,CoM_robot2_links,gsli2,gsbj2,M2_CoM,M2_s_CoM] = robot_links_subtree_new(robot2,config,6);

%% FKP for each body in reference anatomy and zero configuration!!!
robot = robot1;
scale = 0.025;
scale_active = 0.075;
config = [0 0 0 0 0 0]'; 
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
PseudoConnector5a = char(robot.BodyNames(20));
Ts20 = getTransform(robot,config,PseudoConnector5a);
% drawframe(Ts20, scale);
% hold on;
PseudoConnector5b = char(robot.BodyNames(21));
Ts21 = getTransform(robot,config,PseudoConnector5b);
% drawframe(Ts21, scale) 
% hold on;
PseudoConnector6a = char(robot.BodyNames(22));
Ts22 = getTransform(robot,config,PseudoConnector6a);
% drawframe(Ts22, scale)
% hold on;
PseudoConnector6b = char(robot.BodyNames(23));
Ts23 = getTransform(robot,config,PseudoConnector6b);
% drawframe(Ts23, scale)
% hold on;
casebody4 = char(robot.BodyNames(24));
Ts24 = getTransform(robot,config,casebody4);
% drawframe(Ts24, scale) 
% hold on;
wrist_link1 = char(robot.BodyNames(25));
Ts25 = getTransform(robot,config,wrist_link1);
% drawframe(Ts25, scale) 
% hold on;
shaft45 = char(robot.BodyNames(26));
Ts26 = getTransform(robot,config,shaft45);
% drawframe(Ts26, scale_active,'true') % Active joint4
% hold on;
case45 = char(robot.BodyNames(27));
Ts27 = getTransform(robot,config,case45);
% drawframe(Ts27, scale) 
% hold on;
wrist_link2 = char(robot.BodyNames(28));
Ts28 = getTransform(robot,config,wrist_link2);
% drawframe(Ts28, scale) 
% hold on;
case56a = char(robot.BodyNames(29));
Ts29 = getTransform(robot,config,case56a);
% drawframe(Ts29, scale_active,'true') % Active joint5 
% hold on;
case56b = char(robot.BodyNames(30));
Ts30 = getTransform(robot,config,case56b);
% drawframe(Ts30, scale) 
% hold on;
case56c = char(robot.BodyNames(31));
Ts31 = getTransform(robot,config,case56c);
% drawframe(Ts31, scale) 
% hold on;
wrist_link3 = char(robot.BodyNames(32));
Ts32 = getTransform(robot,config,wrist_link3);
% drawframe(Ts32, scale) 
% hold on;
TOOL = char(robot.BodyNames(33));
Ts33 = getTransform(robot,config,TOOL);
% drawframe(Ts33, scale_active,'true') % Active joint6 
% hold on;

% Show new configuration in reference anatomy
figure(reference_figure);
show(robot1,config,'PreservePlot',false);
hold on;
axis auto;
box on;

%% FKP for each body in test anatomy - zero configuration!
robot = robot2;
scale = 0.025;
scale_active = 0.075;
nconfig = [0 0 0 0 0 0]'; % zero configuration
base_link = char(robot.BodyNames(1));
nTs1 = getTransform(robot,nconfig,base_link);
% drawframe(Ts1, scale); 
% hold on;
adaptor = char(robot.BodyNames(2));
nTs2 = getTransform(robot,nconfig,adaptor);
% drawframe(Ts2, scale) 
% hold on;
frame = char(robot.BodyNames(3));
nTs3 = getTransform(robot,nconfig,frame);
% drawframe(Ts3, scale) 
% hold on;
PseudoConnector1a = char(robot.BodyNames(4));
nTs4 = getTransform(robot,nconfig,PseudoConnector1a);
% drawframe(Ts4, scale_active,'true') % Active joint1
% hold on;
PseudoConnector1b = char(robot.BodyNames(5));
nTs5 = getTransform(robot,nconfig,PseudoConnector1b);
% drawframe(Ts5, scale) 
% hold on;
PseudoConnector2a = char(robot.BodyNames(6));
nTs6 = getTransform(robot,nconfig,PseudoConnector2a);
% drawframe(Ts6, scale)
% hold on;
PseudoConnector2b = char(robot.BodyNames(7));
nTs7 = getTransform(robot,nconfig,PseudoConnector2b);
% drawframe(Ts7, scale)
% hold on;
adaptor1 = char(robot.BodyNames(8));
nTs8 = getTransform(robot,nconfig,adaptor1);
% drawframe(Ts8, scale)
% hold on;
AM1 = char(robot.BodyNames(9));
nTs9 = getTransform(robot,nconfig,AM1);
% drawframe(Ts9, scale)
% hold on;
frame1 = char(robot.BodyNames(10));
nTs10 = getTransform(robot,nconfig,frame1);
% drawframe(Ts10, scale_active,'true') % Active joint2
% hold on;
idler1 = char(robot.BodyNames(11));
nTs11 = getTransform(robot,nconfig,idler1);
% drawframe(Ts11, scale)
% hold on;
PseudoConnector3a = char(robot.BodyNames(12));
nTs12 = getTransform(robot,nconfig,PseudoConnector3a);
% drawframe(Ts12, scale)
% hold on;
PseudoConnector3b = char(robot.BodyNames(13));
nTs13 = getTransform(robot,nconfig,PseudoConnector3b);
% drawframe(nTs13, scale) 
% hold on;
PseudoConnector4a = char(robot.BodyNames(14));
nTs14 = getTransform(robot,nconfig,PseudoConnector4a);
% drawframe(Ts14, scale)
% hold on;
PseudoConnector4b = char(robot.BodyNames(15));
nTs15 = getTransform(robot,nconfig,PseudoConnector4b);
% drawframe(Ts15, scale)
% hold on;
adaptor2 = char(robot.BodyNames(16));
nTs16 = getTransform(robot,nconfig,adaptor2);
% drawframe(Ts16, scale)
% hold on;
AM2 = char(robot.BodyNames(17));
nTs17 = getTransform(robot,nconfig,AM2);
% drawframe(Ts17, scale)
% hold on;
frame2 = char(robot.BodyNames(18));
nTs18 = getTransform(robot,nconfig,frame2);
% drawframe(Ts18, scale_active,'true') % Active joint3
% hold on;
idler2 = char(robot.BodyNames(19));
nTs19 = getTransform(robot,nconfig,idler2);
% drawframe(Ts19, scale)
% hold on;
PseudoConnector5a = char(robot.BodyNames(20));
nTs20 = getTransform(robot,nconfig,PseudoConnector5a);
% drawframe(Ts20, scale)
% hold on;
PseudoConnector5b = char(robot.BodyNames(21));
nTs21 = getTransform(robot,nconfig,PseudoConnector5b);
% drawframe(Ts21, scale) 
% hold on;
PseudoConnector6a = char(robot.BodyNames(22));
nTs22 = getTransform(robot,nconfig,PseudoConnector6a);
% drawframe(Ts22, scale)
% hold on;
PseudoConnector6b = char(robot.BodyNames(23));
nTs23 = getTransform(robot,nconfig,PseudoConnector6b);
% drawframe(Ts23, scale)
% hold on;
casebody4 = char(robot.BodyNames(24));
nTs24 = getTransform(robot,nconfig,casebody4);
% drawframe(Ts24, scale) 
% hold on;
wrist_link1 = char(robot.BodyNames(25));
nTs25 = getTransform(robot,nconfig,wrist_link1);
% drawframe(Ts25, scale) 
% hold on;
shaft45 = char(robot.BodyNames(26));
nTs26 = getTransform(robot,nconfig,shaft45);
% drawframe(Ts26, scale_active,'true') % Active joint4
% hold on;
case45 = char(robot.BodyNames(27));
nTs27 = getTransform(robot,nconfig,case45);
% drawframe(Ts27, scale) 
% hold on;
wrist_link2 = char(robot.BodyNames(28));
nTs28 = getTransform(robot,nconfig,wrist_link2);
% drawframe(Ts28, scale) 
% hold on;
case56a = char(robot.BodyNames(29));
nTs29 = getTransform(robot,nconfig,case56a);
% drawframe(Ts29, scale_active,'true') % Active joint5 
% hold on;
case56b = char(robot.BodyNames(30));
nTs30 = getTransform(robot,nconfig,case56b);
% drawframe(Ts30, scale) 
% hold on;
case56c = char(robot.BodyNames(31));
nTs31 = getTransform(robot,nconfig,case56c);
% drawframe(Ts31, scale) 
% hold on;
wrist_link3 = char(robot.BodyNames(32));
nTs32 = getTransform(robot,nconfig,wrist_link3);
% drawframe(Ts32, scale) 
% hold on;
TOOL = char(robot.BodyNames(33));
nTs33 = getTransform(robot,nconfig,TOOL);
% drawframe(Ts33, scale_active,'true') % Active joint6 
% hold on;

%% Inverse Kinematic Problem-Analytical Solution
%% Specify reference anatomy-With regard to URDF Pseudo Configuration!!!
tpi_ref = [0 0 0 0 0 0]';% Reference Anatomy values are theta_p_i's specified in MMDinertia_scale_xacro_FixedAnatomy.xacro and in the urdf built from it and given in l.10
%% Specify wanted anatomy-With regard to Matlab Pseudo Configuration!!!
% IF I WANT θp1 to be 0.7854 must tp1 = -0.7854: 
% because 1.5708-0.7854 = 0.7854
% IF I WANT θp3 to be 0 must tp3 = -1.5708: 
% because 1.5708-1.5708 = 0

% tpi = [0 0 0 0 0 0]'; % For robot file: reference_MMD_inverse.urdf
% tpi = [-pi/4 0 pi/4 0 0 0]'; % For robot file: x_anatII.urdf
% tpi = [-0.7854 0 1.5708 0 0 0]'; % For robot file: x_test_inverse_MMD.urdf
tpi = [0.7854 0.7854 0 -0.7854 1.5708 -0.7854]'; % For robot file: x_test_inverse_MMD1.urdf

% test for new configuration the reference anatomy
config = [1 1 -1 0 0 0]';

%% Active screws
wi(:,1) = [0 0 1]';
wi(:,2) = [0 1 0]';
wi(:,3) = [0 1 0]';
wi(:,4) = [1 0 0]';
wi(:,5) = [0 1 0]';
wi(:,6) = [1 0 0]';
%% Pseudo screws as viewd for tpi = 0!!! Not the reference anatomy!!!
wpi(:,1) = [0 1 0]';
wpi(:,2) = [1 0 0]';
wpi(:,3) = [0 1 0]';
wpi(:,4) = [1 0 0]';
wpi(:,5) = [0 1 0]';
wpi(:,6) = [1 0 0]';
%% Pi points are extracted from reference Anatomy in zero configuration!!!
pi(:,1)= Ts5(1:3,4); % p1
pi(:,2)= Ts7(1:3,4); % p2
pi(:,3)= Ts10(1:3,4); % p3
pi(:,4)= Ts13(1:3,4); % p4
pi(:,5)= Ts15(1:3,4); % p5
pi(:,6)= Ts18(1:3,4); % p6
pi(:,7)= Ts21(1:3,4); % p7
pi(:,8)= Ts23(1:3,4); % p8
pi(:,9)= Ts24(1:3,4); % TCP
pi(:,10)= Ts26(1:3,4); % wp1
pi(:,11)= Ts30(1:3,4); % wp2
pi(:,12)= Ts33(1:3,4); % wp3
%% New Pi points are extracted from test Anatomy
npi(:,1)= nTs5(1:3,4); % p1
npi(:,2)= nTs7(1:3,4); % p2
npi(:,3)= nTs10(1:3,4); % p3
npi(:,4)= nTs13(1:3,4); % p4
npi(:,5)= nTs15(1:3,4); % p5
npi(:,6)= nTs18(1:3,4); % p6
npi(:,7)= nTs21(1:3,4); % p7
npi(:,8)= nTs23(1:3,4); % p8
npi(:,9)= nTs24(1:3,4); % TCP
npi(:,10)= nTs26(1:3,4); % wp1
npi(:,11)= nTs30(1:3,4); % wp2
npi(:,12)= nTs33(1:3,4); % wp3
%% Active twists - reference anatomy
xi_ai(:,1) = createtwist(wi(:,1),pi(:,1));
xi_ai(:,2) = createtwist(wi(:,2),pi(:,3));
xi_ai(:,3) = createtwist(wi(:,3),pi(:,6));
xi_ai(:,4) = createtwist(wi(:,4),pi(:,10));
xi_ai(:,5) = createtwist(wi(:,5),pi(:,11));
xi_ai(:,6) = createtwist(wi(:,6),pi(:,12));
%% Active twists - test anatomy == nJs!!! swsto
% tp1, tp3 must folloe the rules specified in l.354-357
nxi_ai(:,1) = createtwist(wi(:,1),npi(:,1));
nxi_ai(:,2) = createtwist(roty(tpi(1))*rotx(tpi(2))*wi(:,2),npi(:,3));
nxi_ai(:,3) = createtwist(roty(tpi(1))*rotx(tpi(2))*roty(tpi(3))*rotx(tpi(4))*wi(:,3),npi(:,6));
nxi_ai(:,4) = createtwist(roty(tpi(1))*rotx(tpi(2))*roty(tpi(3))*rotx(tpi(4))*roty(tpi(5))*rotx(tpi(6))*wi(:,4),pi(:,10));
nxi_ai(:,5) = createtwist(roty(tpi(1))*rotx(tpi(2))*roty(tpi(3))*rotx(tpi(4))*roty(tpi(5))*rotx(tpi(6))*wi(:,5),pi(:,11));
nxi_ai(:,6) = createtwist(roty(tpi(1))*rotx(tpi(2))*roty(tpi(3))*rotx(tpi(4))*roty(tpi(5))*rotx(tpi(6))*wi(:,6),pi(:,12));
%% Active exponentials
exp_ai(:,:,1) = twistexp(xi_ai(:,1), config(1));
exp_ai(:,:,2) = twistexp(xi_ai(:,2), config(2));
exp_ai(:,:,3) = twistexp(xi_ai(:,3), config(3));
exp_ai(:,:,4) = twistexp(xi_ai(:,4), config(4));
exp_ai(:,:,5) = twistexp(xi_ai(:,5), config(5));
exp_ai(:,:,6) = twistexp(xi_ai(:,6), config(6));
%% Pseudo twists
xi_pi(:,1) = createtwist(wpi(:,1),pi(:,1));
xi_pi(:,2) = createtwist(wpi(:,2),pi(:,2));
xi_pi(:,3) = createtwist(wpi(:,3),pi(:,4));
xi_pi(:,4) = createtwist(wpi(:,4),pi(:,5));
xi_pi(:,5) = createtwist(wpi(:,5),pi(:,7));
xi_pi(:,6) = createtwist(wpi(:,6),pi(:,8));
%% Pseudo exponenentials (can be calculated since anatomy is specified by user)
exp_pi_ref(:,:,1) = twistexp(xi_pi(:,1), tpi_ref(1));
exp_pi_ref(:,:,2) = twistexp(xi_pi(:,2), tpi_ref(2));
exp_pi_ref(:,:,3) = twistexp(xi_pi(:,3), tpi_ref(3));
exp_pi_ref(:,:,4) = twistexp(xi_pi(:,4), tpi_ref(4));
exp_pi_ref(:,:,5) = twistexp(xi_pi(:,5), tpi_ref(5));
exp_pi_ref(:,:,6) = twistexp(xi_pi(:,6), tpi_ref(6));

exp_pi(:,:,1) = twistexp(xi_pi(:,1), tpi(1));
exp_pi(:,:,2) = twistexp(xi_pi(:,2), tpi(2));
exp_pi(:,:,3) = twistexp(xi_pi(:,3), tpi(3));
exp_pi(:,:,4) = twistexp(xi_pi(:,4), tpi(4));
exp_pi(:,:,5) = twistexp(xi_pi(:,5), tpi(5));
exp_pi(:,:,6) = twistexp(xi_pi(:,6), tpi(6));

Pi(:,:,1) = exp_pi(:,:,1)*exp_pi(:,:,2);
Pi_ref(:,:,1) = exp_pi_ref(:,:,1)*exp_pi_ref(:,:,2);
Pi(:,:,2) = exp_pi(:,:,3)*exp_pi(:,:,4);
Pi_ref(:,:,2) = exp_pi_ref(:,:,1)*exp_pi_ref(:,:,2);
Pi(:,:,3) = exp_pi(:,:,5)*exp_pi(:,:,6);
Pi_ref(:,:,3) = exp_pi_ref(:,:,5)*exp_pi_ref(:,:,6);
% %% Draw Active Twists for reference anatomy and configuration
% % f = figure('Name','Joints');5
% axis_xi_a1 = drawtwist(xi_ai(:,1));
% hold on;
% axis_xi_a2 = drawtwist(xi_ai(:,2));
% hold on;
% axis_xi_a3 = drawtwist(xi_ai(:,3));
% hold on;

%   3. FK mappings of CoM 
gsli(:,:,1) = exp_ai(:,:,1)*gsli0(:,:,1);
gsli(:,:,2) = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*gsli0(:,:,2);
gsli(:,:,3) = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gsli0(:,:,3);
gsli(:,:,4) = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*exp_ai(:,:,4)*gsli0(:,:,4);
gsli(:,:,5) = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*exp_ai(:,:,4)*exp_ai(:,:,5)*gsli0(:,:,5);
gsli(:,:,6) = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*exp_ai(:,:,4)*exp_ai(:,:,5)*exp_ai(:,:,6)*gsli0(:,:,6);
%% Jbsli0
for i=1:6
    [Jbsli(:,:,i),Jssli(:,:,i),Jbsli_POE(:,:,i)] = Jbody_CoM_6DoF(xi_ai, exp_ai, Pi, gsli0, gsli, i);
    [Jbsli_ref(:,:,i),Jssli_ref(:,:,i),Jbsli_ref_POE(:,:,i)] = Jbody_CoM_6DoF(xi_ai, exp_ai, Pi_ref, gsli0, gsli0, i);
end
%% Calculate Spatial Jacobian for test anatomy in zero configuration
% For Spatial Jacobian, gst0 is for 3dof Ts24 in test anatomy-configuration: gst0 = nTs24
% % [nJs] = jacobians_3dof_MMD(nxi_ai(:,1), nxi_ai(:,2), nxi_ai(:,3), nconfig(1), nconfig(2), nconfig(3), nTs33);
% [pa1,pa2,pa3] = Plot_current_axes_twists(Js,f)

%% Specify wanted tf
% For MMD IKP solver gst0 is Ts33 in reference anatomy+configuration
gst0 = [     0.0008    1.0000   -0.0080    0.1711;...
            -1.0000    0.0008    0.0016    0.0002;...
             0.0016    0.0080    1.0000    0.7412;...
                  0         0         0    1.0000];
           
% gd = Ts24; % first solve only for 3 DoF, not the Wrist
% angles = inverse_MMD(gd,pi,xi_ai,xi_pi,tpi,exp_pi,f,gst0) % Not ready yet
% % test IKP solution sets
% test(1:3,1) = angles(:,3);
% test(4:6,1) = [0 0 0]';
% gd_test = getTransform(robot,test,casebody4)
% error = gd-gd_test


gd_2 = getTransform(robot1,config,TOOL); 
gd_2_POE = MMD_POE_FKP(config,xi_ai,[0 0 0 0 0 0]',xi_pi,gst0);
% gd_2-gd_2_POE
% figure(reference_figure); % plots goal position
% scatter3(gd_2(1,4), gd_2(2,4), gd_2(3,4),'LineWidth',50,'MarkerEdgeColor',[1 0 0], 'MarkerFaceColor',[1 0 0]), hold on

% test for new configuration the test anatomy
gd = getTransform(robot2,config,TOOL); % this FKP MUST be different from gd_2
gd_POE = MMD_POE_FKP(config,xi_ai,tpi,xi_pi,gst0);

gd_wanted = gd_POE;
gd_wanted(1,4) = gd_POE(1,4)+0.1;
gd_wanted(2,4) = gd_POE(2,4)+0.2;
gd_wanted(3,4) = gd_POE(3,4)-0.1;

%% Calculate Jacobians


% % % gd-gd_POE
% % %% MATLAB IKP solver
% % ik = robotics.InverseKinematics('RigidBodyTree',robot2); % create solver
% % [configSol,solInfo] = ik('TOOL',gd_POE,[0.01 0.01 0.01 0.01 0.01 0.01]',[1 -1 0 0 0 0]');
% % 
% % %% MMD IKP solver
% % angles2 = inverse_MMD(gd_POE,npi,xi_ai,nJs,xi_pi,tpi,exp_pi,reference_figure,test_figure,gst0);
% % angles3 = inverse_MMD_2(gd_POE,npi,xi_ai,nJs,xi_pi,tpi,exp_pi,reference_figure,test_figure,gst0);
% % %% test IKP solution sets
% % test(1:3,1) = angles3(1:3,2);
% % test(4:6,1) = configSol(4:6,1);
% % % gd_test is Ts24(tpi= [x_anatII], tai = [? ? ? ? ? ?]
% % gd_test = getTransform(robot2,test,TOOL); % MATLAB fkp solver: this FKP counts         
% % gd_test_POE = MMD_POE_FKP(test,xi_ai,tpi,xi_pi,gst0)
% % % evaluate solutions
% % error = gd-gd_test
% % error_POE = gd_POE-gd_test_POE

% % %% Show configuration of test anatomy WITH reference!
% % 
% % IKP_test_figure = figure;
% % show(robot2,config);
% % hold on;
% % 
% % figure(IKP_test_figure);
% % show(robot2,test);
% % hold on;
% % % box on;

%% Apply external force to end-effector
% fext = externalForce(robot,bodyname,wrench,configuration) 
% composes the external force matrix assuming that wrench is in the bodyname frame 
% for the specified configuration. The force matrix fext is given in the base frame.
% [Tx Ty Tz Fx Fy Fz] wrench vector
wrench = [0 0 0 0 0 50];
fext_TOOL = externalForce(robot,'TOOL',wrench,config); % ONLY for 6 DoF
%% POE Dynamics
theta_dot = [0 1 0 0 0 0]';
theta_dotdot = [0 10 0 0 0 0]';
Mi  =  M0_CoM; % link inertia frame
Mi_s  = M0_s_CoM; % spatial frame
Jis = Jssli_ref;
Jib = Jbsli_ref;
[M1s, M1b, M1_POE, dM1_POE, C1_POE] = manipulator_inertia_matrix_6DoF(xi_ai, exp_ai, Pi_ref, gsli0, Mi, Mi_s, Jis, Jib, theta_dot);
M1m = massMatrix(robot1,config);
V1b = C1_POE*theta_dot;
V1m = velocityProduct(robot1,config,theta_dot);
[N1_POE] = calculate_gravity_matrix_6DoF(xi_ai, gsli0,[Mi(1,1,1) Mi(1,1,2) Mi(1,1,3) Mi(1,1,4) Mi(1,1,5) Mi(1,1,6)]',  Pi_ref, config);
gravTorq1 = gravityTorque(robot1,config);

joint_torque1 = M1_POE*theta_dotdot + C1_POE*theta_dot + N1_POE;
jointTorq_ref = inverseDynamics(robot1,config,theta_dot,theta_dotdot);
dyn1_error = jointTorq_ref - joint_torque1; % Matlab - My_POE_Way


Jis = Jssli;
Jib = Jbsli;
[M2s, M2b, M2_POE, dM2_POE, C2_POE] = manipulator_inertia_matrix_6DoF(xi_ai, exp_ai, Pi, gsli0, Mi, Mi_s, Jis, Jib, theta_dot);
M2m = massMatrix(robot2,config);
V2b = C2_POE*theta_dot;
V2m = velocityProduct(robot2,config,theta_dot);
[N2_POE] = calculate_gravity_matrix_6DoF(xi_ai, gsli0,[Mi(1,1,1) Mi(1,1,2) Mi(1,1,3) Mi(1,1,4) Mi(1,1,5) Mi(1,1,6)]',  Pi, config);
gravTorq2 = gravityTorque(robot2,config);

joint_torque2 = M2_POE*theta_dotdot + C2_POE*theta_dot + N2_POE;
jointTorq_test = inverseDynamics(robot2,config,theta_dot,theta_dotdot);
dyn2_error = jointTorq_test - joint_torque2; % Matlab - My_POE_Way
%% Joint torques calculation [N*m]
% Gravity Compensation
%% ONLY when 12 DoF considered
% joints_numberAM = [1 4 7 10 11 12]';
% joints_numberPM = [2 3 5 6 8 9]';
% gravTorq = gravityTorque(robot,config);
% gravTorqAM = [gravTorq(1) gravTorq(4) gravTorq(7) gravTorq(10) gravTorq(11) gravTorq(12)]';
% gravTorqPM = [gravTorq(2) gravTorq(3) gravTorq(5) gravTorq(6) gravTorq(8) gravTorq(9)]';
%% ONLY when 6 DoF considered
gravTorq = gravityTorque(robot,config);
joints_numberAM = [1 2 3 4 5 6]';
gravTorqAM = [gravTorq(1) gravTorq(2) gravTorq(3) gravTorq(4) gravTorq(5) gravTorq(6)]';

%%
f2 = figure('Name','Stall Joint Torques for every joint','NumberTitle','off');
plot(joints_numberAM, gravTorqAM,'r-o','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','r','DisplayName','Gravity Compensation Torque Active Modules (Stall)');
hold on;
% plot(joints_numberPM, gravTorqPM,'b-o','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','b','DisplayName','Gravity Compensation Torque Passive Modules(Stall)');
% hold on;
title('Stall Joint torques of Active Modules [Mass of pseudojoints: (1,2) = 8kg (3,4) = 4kg, (5,6) = 2kg]')
xlabel('Joints [i]')
ylabel('Torque [Nm]')
title(legend,'Gravity compensation torques in each joint')

% Maximum Joint velocities
% MAX_Joint_Vel_all = [3.3510 0 0 3.3510 0 0 3.3510  0 0 4.0841 4.0841 4.0841]'; % for 12 DoF
MAX_Joint_Vel_all = [3.3510 3.3510 3.3510 4.0841 4.0841 4.0841]'; % for 6 DoF

MAX_Joint_Vel_1 = [MAX_Joint_Vel_all(1)*1 0 0 0 0 0]';
MAX_Joint_Vel_2 = [MAX_Joint_Vel_all(1)*1 MAX_Joint_Vel_all(2)*1 0 0 0 0]';
MAX_Joint_Vel_3 = [MAX_Joint_Vel_all(1)*1 MAX_Joint_Vel_all(2)*1 MAX_Joint_Vel_all(3)*1 0 0 0]';
MAX_Joint_Vel_4 = [MAX_Joint_Vel_all(1)*1 MAX_Joint_Vel_all(2)*1 MAX_Joint_Vel_all(3)*1 MAX_Joint_Vel_all(4)*1 0 0 ]';
MAX_Joint_Vel_5 = [MAX_Joint_Vel_all(1)*1 MAX_Joint_Vel_all(2)*1 MAX_Joint_Vel_all(3)*1 MAX_Joint_Vel_all(4)*1 MAX_Joint_Vel_all(5)*1 0 ]';
MAX_Joint_Vel_6 = [MAX_Joint_Vel_all(1)*1 MAX_Joint_Vel_all(2)*1 MAX_Joint_Vel_all(3)*1 MAX_Joint_Vel_all(4)*1 MAX_Joint_Vel_all(5)*1 MAX_Joint_Vel_all(6)*1 ]';
% MAX_Joint_Accel_all = [10 0 0 10 0 0 10 0 0 10 10 10]';% for 12 DoF
MAX_Joint_Accel_all = [10 10 10 10 10 10]';% for 6 DoF

jointTorq_conf_1 = inverseDynamics(robot,config,MAX_Joint_Vel_1,MAX_Joint_Accel_all); 
jointTorq_conf_2 = inverseDynamics(robot,config,MAX_Joint_Vel_2,MAX_Joint_Accel_all); 
jointTorq_conf_3 = inverseDynamics(robot,config,MAX_Joint_Vel_3,MAX_Joint_Accel_all);
jointTorq_conf_4 = inverseDynamics(robot,config,MAX_Joint_Vel_4,MAX_Joint_Accel_all); 
jointTorq_conf_5 = inverseDynamics(robot,config,MAX_Joint_Vel_5,MAX_Joint_Accel_all); 
jointTorq_conf_6 = inverseDynamics(robot,config,MAX_Joint_Vel_6,MAX_Joint_Accel_all);

% Plots Joint Torques for each velocity case
joints_number = [1 2 3 4 5 6]';
f1 = figure('Name','Joint Torques for different Joint velocity cases','NumberTitle','off');
plot(joints_number, jointTorq_conf_1,'b--s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','b','DisplayName','1');
hold on;
plot(joints_number, jointTorq_conf_2,'m-s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','m','DisplayName','1,2');
hold on;
plot(joints_number, jointTorq_conf_3,'c--s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','c','DisplayName','1,2,3');
hold on;
plot(joints_number, jointTorq_conf_4,'k-s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','DisplayName','1,2,3,4');
hold on;
plot(joints_number, jointTorq_conf_5,'g--s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','g','DisplayName','1,2,3,4,5');
hold on;
plot(joints_number, jointTorq_conf_6,'y-s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','y','DisplayName','1,2,3,4,5,6');
hold on;
% plot(joints_number, gravTorq,'r-o','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','r','DisplayName','Gravity Compensation Torque (Stall)');
% hold on;
title('Joint torques of each joint for Maximum Joint Velocities [Mass of pseudojoints: (1,2) = 8kg (3,4) = 4kg, (5,6) = 2kg & 5kg payload]')
xlabel('Joints [i]')
ylabel('Torque [Nm]')
title(legend,'Different cases for max velocities of Active Joints')
%% Polynomial Trajectory Generation
cdt = 2.5; 
tt = 0:cdt:10; %1x5

qi = [1 -1 -1 1 0.5 0.5]';
qi2 = [1 -1 -1 1 0.5 0.5]';
qm = [qi(1)+1.5708/2 qi(2)-1.5708/2 qi(3)-1.5708/2 0 0 0]';
qm2 = [qi(1)+1.5708 qi(2)-1.5708 qi(3)-1.5708 0 0 0]';
qf = [qm(1)+1.5708 qm(2)-1.5708 qm(3)-1.5708 0 0 0]';
% p = exampleHelperPolynomialTrajectory(qi,qf,tDuration)
tWaypoints = [1 2 3 4 5]; %1x5
qWaypoints = [qi qi2 qm qm2 qf]'; %5x6
[qDesired, qdotDesired, qddotDesired, tt] = exampleHelperJointTrajectoryGeneration(tWaypoints, qWaypoints, tt);


