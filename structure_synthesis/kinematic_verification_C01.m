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
% load main folder
addpath('/home/nikos/matlab_ws/modular_dynamixel/')

%% Reference anatomy
% robot urdf is built from: /home/nikos/matlab_ws/modular_dynamixel/kinematic_verification_01.xacro
[robot1] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/C010.urdf'); % with 1 Pseudo 
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
robot_DoF = 1;
%% Zero Structure-Anatomy-Configuration tfs
scale = 0.025; scale_active = 0.075;
adaptor = char(robot1.BodyNames(2)); Ts2 = getTransform(robot1,config,adaptor); drawframe(Ts2, scale); hold on;
frame = char(robot1.BodyNames(3)); Ts3 = getTransform(robot1,config,frame); drawframe(Ts3, scale_active); hold on;
PseudoConnector1a = char(robot1.BodyNames(4)); Ts4 = getTransform(robot1,config,PseudoConnector1a); drawframe(Ts3, scale); hold on;
PseudoConnector1b = char(robot1.BodyNames(5)); Ts5 = getTransform(robot1,config,PseudoConnector1b); drawframe(Ts5, scale); hold on;
frame1 = char(robot1.BodyNames(8)); Ts8 = getTransform(robot1,config,frame1); drawframe(Ts8, scale); hold on;

% g_st = getTransform(robot,config,t(source),s(target)) source: frame from --> target: frame to
g_s_li0 = getTransform(robot1,config,frame);
g_s_lj0 = getTransform(robot1,config,PseudoConnector1b);
g_s_li10 = getTransform(robot1,config,frame1);

g_li_lj0 = getTransform(robot1,config,PseudoConnector1b,frame);
g_lj_li10 = getTransform(robot1,config,frame1,PseudoConnector1b);
g_li_li10 = getTransform(robot1,config,frame1,frame);
g0(:,:,1) = g_s_li0;
g0(:,:,2) = g_s_lj0;
g0(:,:,3) = g_s_li10;
g0(:,:,4) = g_li_lj0;
g0(:,:,5) = g_lj_li10;
g0(:,:,6) = g_li_li10;
%% Test anatomy θpj=1.5708
[robot2] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/C010_1.urdf'); % with 1 Pseudo 
robot2.DataFormat = 'column';
robot1.Gravity = [0 0 -9.80665];
config2 = homeConfiguration(robot2);
test1_figure = figure;
figure(test1_figure);
show(robot2,config2,'PreservePlot',false);
hold on;
axis auto;
box on;
robot_DoF = 1;
scale = 0.025; scale_active = 0.075;
frame_1 = char(robot2.BodyNames(3)); Ts3_1 = getTransform(robot2,config2,frame_1); drawframe(Ts3_1, scale_active); hold on;
frame1_1 = char(robot2.BodyNames(8)); Ts8_1 = getTransform(robot2,config2,frame1_1); drawframe(Ts8_1, scale); hold on;
%% Create twists Symbolic
pi(:,1)= sym('p0i',[3 1]); % p0i on ξai
pi(:,2)= sym('p1j',[3 1]); % p1j on ξpj
pi(:,3)= sym('p0i1',[3 1]); % p0_i+1 on ξa_i+1
wi(:,1)= sym('w0i',[3 1]); % w0i on ξai
wi(:,2)= sym('w1j',[3 1]); % w1j on ξpj
wi(:,3)= sym('w0i1',[3 1]); % w0_i+1 on ξa_i+1
%% Create twists Numeric
% pi(:,1)= Ts3(1:3,4); % p1 on ξi
% pi(:,2)= Ts5(1:3,4); % p2 on ξj
% pi(:,3)= Ts8(1:3,4); % p3 on ξi+1
% wi(:,1) = [0 0 1]';
% wi(:,2) = [0 1 0]';
% wi(:,3) = [0 1 0]';
xi(:,1) = createtwist(wi(:,1),pi(:,1)); %ξi
xi(:,2) = createtwist(wi(:,2),pi(:,2)); %ξj
xi(:,3) = createtwist(wi(:,3),pi(:,3)); %ξi+1
%% Fwd Kinematic Mapping
ti = sym('ti',[3 1], 'real');
ti(1) = 'ti';
ti(2) = 'tp1';
ti(3) = 'ti1';
% ti = [1.24 1.5708 1.23]';
expi(:,:,1) = twistexp(xi(:,1), ti(1));
expi(:,:,2) = twistexp(xi(:,2), ti(2));
expi(:,:,3) = twistexp(xi(:,3), ti(3));
g(:,:,1) = expi(:,:,1)*g0(:,:,1); % g_s_li
g(:,:,2) = expi(:,:,1)*expi(:,:,2)*g0(:,:,2); % g_s_lj
g(:,:,3) = expi(:,:,1)*expi(:,:,2)*expi(:,:,3)*g0(:,:,3); % g_s_li1
%% Express twists with respect to previous reference frames
xi_i1 = inv(ad(g_s_li0))*xi(:,3);  % reference WORKS
g_li_li1_0 = twistexp(xi_i1,0)*g0(:,:,6) % IT WORKS SINCE: g_li_li1_0
% = g_li_li10
xi_j = inv(ad(g_s_li0))*xi(:,2);
xj_i1 = inv(ad(g_s_lj0))*xi(:,3);

% Spatial Jacobian is instantaneous twists with respect to {s}
% Js(:,1)=sym(xi(:,1)); % for sym only
Js(:,1)=xi(:,1);
% Js(:,2)=ad(expi(:,:,1))*xi(:,2);
Js(:,2)=ad(expi(:,:,1)*expi(:,:,2))*xi(:,3);

% Relative Jacobian
% Jr = xi(:,1);
% Jr(:,2) = ad(expi(:,:,1)*expi(:,:,2))*xi_i1;

% xi1_44 = twist(Js(:,2)); % the so(3) element of the second twist
% composition rule 
% g_li_li1 = inv(g0(:,:,1))*g0(:,:,3);
% drawtwist(Js(:,2))
% xi_i1_inst = ad(inv(g_li_li1))*Js(:,2); % instantaneous1-This is just to
% rememeber my mistake
% xi_i1_insta = inv(ad(g(:,:,1)))*Jr(:,2); % instantaneous2 as in eq.3.10 Murray p.94
% xi_i1_instb = inv(ad(g(:,:,1)))*xi(:,3);
% xi_i1_instc = inv(ad(g(:,:,1)))*Js(:,2);
% xi_i1_instd = inv(ad(expi(:,:,1)*g_s_li0))*Js(:,2);
% g_li_li1a = twistexp(xi_i1,ti(3))*g0(:,:,6) % as in eq.3.7 Murray p.94
g_li_li1a = twistexp(xi_j,ti(2))*g0(:,:,4)*twistexp(xj_i1,ti(3))*g0(:,:,5)
% g_li_li1 = getTransform(robot1,[1.24 1.23]',frame1,frame)

% g_li_li1b = twistexp(xi_i1_instb,ti(3))*g0(:,:,6)
% g_li_li1c = twistexp(xi_i1_instc,ti(3))*g0(:,:,6)
% g_li_li1d = twistexp(xi_i1_instd,ti(3))*g0(:,:,6)
% xi_i1_inst_44 = twist(xi_i1_inst); % the so(3) element of the second twist relative to the first twist
% % xi_i1_inst_44_ij are the elements of ξ^(wedge) \in se(3)
% a = fopen('xi_i1_inst_44_11.txt','w');
% fprintf(a,'%s\n',char(xi_i1_inst_44(1,1)));
% fclose(a);


g_li_li1_1 = getTransform(robot2,[1.24 1.23]',frame1_1,frame_1)