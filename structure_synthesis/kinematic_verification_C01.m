clear;
clc;
close all;
%% Metamorphic manipulator of 2 DoF with 1 PseudoJoint is modeled as:
%% Active - C01 - Passive - Active => 0 - C01 - 1 - 0
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
%% all pi,wi for twists,expos creation must be calculated from this
%% AND recalculated depending the synthetic variables!!!
scale = 0.025; scale_active = 0.075;
adaptor = char(robot1.BodyNames(2)); Ts2 = getTransform(robot1,config,adaptor); drawframe(Ts2, scale); hold on;
frame = char(robot1.BodyNames(3)); Ts3 = getTransform(robot1,config,frame); drawframe(Ts3, scale_active); hold on;
PseudoConnector1a = char(robot1.BodyNames(4)); Ts4 = getTransform(robot1,config,PseudoConnector1a); drawframe(Ts4, scale); hold on;
PseudoConnector1b = char(robot1.BodyNames(5)); Ts5 = getTransform(robot1,config,PseudoConnector1b); drawframe(Ts5, scale); hold on;
frame1 = char(robot1.BodyNames(8)); Ts8 = getTransform(robot1,config,frame1); drawframe(Ts8, scale); hold on;

% g_st = getTransform(robot,config,t(source),s(target)) source: frame from --> target: frame to
g_s_li0 = getTransform(robot1,config,frame);
g_s_lj0 = getTransform(robot1,config,PseudoConnector1b);
g_s_li10 = getTransform(robot1,config,frame1);

g_s_lk0 = getTransform(robot1,config,PseudoConnector1a);
g_li_lk0 = getTransform(robot1,config,PseudoConnector1a,frame);
g_lk_lj0 = getTransform(robot1,config,PseudoConnector1b,PseudoConnector1a);
g_lk_li0 = getTransform(robot1,config,PseudoConnector1b,frame);
g_lk_li10 = getTransform(robot1,config,PseudoConnector1b,frame1);

g_li_lj0 = getTransform(robot1,config,PseudoConnector1b,frame);
g_lj_li10 = getTransform(robot1,config,frame1,PseudoConnector1b);
g_li_li10 = getTransform(robot1,config,frame1,frame);
g0(:,:,1) = g_s_li0;
g0(:,:,2) = g_s_lj0;
g0(:,:,3) = g_s_li10;
g0(:,:,4) = g_li_lj0;
g0(:,:,5) = g_lj_li10;
g0(:,:,6) = g_li_li10;

g0(:,:,7) = g_li_lk0; % with these 2: i->k->j->i1
g0(:,:,8) = g_lk_lj0;
g0(:,:,9) = g_s_lk0;
%% Extracs initial geometry attributes
pi(:,1)= Ts3(1:3,4); % p1 on ξi
pi(:,2)= Ts5(1:3,4); % p2 on ξj
pi(:,3)= Ts8(1:3,4); % p3 on ξi+1
pi(:,4)= Ts4(1:3,4); % p1 on ξk
wi(:,1) = [0 0 1]';
wi(:,2) = [0 1 0]';
wi(:,3) = [0 1 0]';
%% Create twists Numeric
xi(:,1) = createtwist(wi(:,1),pi(:,1)); %ξi
xi(:,2) = createtwist(wi(:,2),pi(:,2)); %ξj
xi(:,3) = createtwist(wi(:,3),pi(:,3)); %ξi+1
%% Extract relative twists
% This are the relative twists of manipulator in reference
% anatomy-configuration, as from eq.3.10
xi_j = inv(ad(g_s_li0))*xi(:,2);
xj_i1 = inv(ad(g_s_lj0))*xi(:,3);
xi_i1_0 = inv(ad(g_s_li0))*xi(:,3);

%% Test1 anatomy: frame__PseudoConnector1a=[0,0,0,0,0,0] θpj=1.5708
%% tests metamorphic_joint: PseudoConnector1a__PseudoConnector1b
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
PseudoConnector1a_1 = char(robot2.BodyNames(4)); Ts4_1 = getTransform(robot2,config2,PseudoConnector1a_1); drawframe(Ts4_1, scale); hold on;
PseudoConnector1b_1 = char(robot2.BodyNames(5)); Ts5_1 = getTransform(robot2,config2,PseudoConnector1b_1); drawframe(Ts5_1, scale); hold on;
%% Test2 anatomy: frame__PseudoConnector1a=[1.5708,0,0,0,0,0] θpj=0
%% tests synthetic_joint: frame_PseudoConnector1a
[robot3] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/C010_2.urdf'); % with 1 Pseudo 
robot3.DataFormat = 'column';
robot3.Gravity = [0 0 -9.80665];
config3 = homeConfiguration(robot3);
test2_figure = figure;
show(robot3,config3,'PreservePlot',false);
hold on;
axis auto;
box on;
frame_2 = char(robot3.BodyNames(3)); Ts3_2 = getTransform(robot3,config3,frame_2); drawframe(Ts3_2, scale_active); hold on;
frame1_2 = char(robot3.BodyNames(8)); Ts8_2 = getTransform(robot3,config3,frame1_2); drawframe(Ts8_2, scale); hold on;
PseudoConnector1a_2 = char(robot3.BodyNames(4)); Ts4_2 = getTransform(robot3,config3,PseudoConnector1a_2); drawframe(Ts4_2, scale); hold on;
PseudoConnector1b_2 = char(robot3.BodyNames(5)); Ts5_2 = getTransform(robot3,config3,PseudoConnector1b_2); drawframe(Ts5_2, scale); hold on;

%% Create twists Symbolic
% pi(:,1)= sym('p0i',[3 1]); % p0i on ξai
% pi(:,2)= sym('p1j',[3 1]); % p1j on ξpj
% pi(:,3)= sym('p0i1',[3 1]); % p0_i+1 on ξa_i+1
% wi(:,1)= sym('w0i',[3 1]); % w0i on ξai
% wi(:,2)= sym('w1j',[3 1]); % w1j on ξpj
% wi(:,3)= sym('w0i1',[3 1]); % w0_i+1 on ξa_i+1
%% Fwd Kinematic Mapping
% ti = sym('ti',[3 1], 'real');
% ti(1) = 'ti';
% ti(2) = 'tp1';
% ti(3) = 'ti1';
ti = [0 0 0]'; % from here i control the 3 angles: 1:active1 2:passive1 3:active2
Rx01 = 1.5708; Ry01=0; Rz01=0; Px01=0.05; Py01=0.05; Pz01=0.05; % from here i control the 
% 6 Euler variables of the synthetic joint. These variables are ONLY LOCAL
% in the frame-PseudoConnector1a. i.e. Describe how "Pseudoconector1a"
% changes with respect to "frame".

%% Here i test only metamorphic_joint:PseudoConnector1a__PseudoConnector1b
% Change only in theta_j with fixed structure. I did it to study the twists
% and DH transform
% % expi(:,:,1) = twistexp(xi(:,1), ti(1));
% % expi(:,:,2) = twistexp(xi(:,2), ti(2));
% % expi(:,:,3) = twistexp(xi(:,3), ti(3));
% % g(:,:,1) = expi(:,:,1)*g0(:,:,1); % g_s_li
% % g(:,:,2) = expi(:,:,1)*expi(:,:,2)*g0(:,:,2); % g_s_lj
% % g(:,:,3) = expi(:,:,1)*expi(:,:,2)*expi(:,:,3)*g0(:,:,3); % g_s_li1

%% Here i test FKM of frame__PseudoConnector1a. This is the joint that
% genetic algorithm changes its EULER variables.
% My goal is to convert this variables to POE
% g_fP1a = [R_fP1a p
%            0^T   1]
R_fP1a = rotz(Rz01)*roty(Ry01)*rotx(Rx01);
[wmegaR thetaR] = rotparam(R_fP1a); % this is SO(3) element-skew matrix and angle
exp_wmegaR = skewexp(wmegaR,thetaR); % ==R_fP1a ALWAYS the same
%  find twist of synthetic joint_it is "equivalent to the reference" since
%  twist must be built for reference structure
twist_fP1a = createtwist(wmegaR,[g_s_lk0(1,4)+Px01 g_s_lk0(2,4)+Py01 g_s_lk0(3,4)+Pz01]'); %ξk
% exp_fP1a = twistexp(twist_fP1a,thetaR);

g_fP1a = [exp_wmegaR [g_s_lk0(1,4)+Px01 g_s_lk0(2,4)+Py01 g_s_lk0(3,4)+Pz01]'; 0 0 0 1];
[twist_g_fP1a theta_g_fP1a] = homtotwist(g_fP1a); % this is SE(3) element that represents 
% the rigid motion of the structure change induced by synthetic_joint:frame_PseudoConnector1a

% In order expos to be computed, the initial wi,pi must be transformed due
% to the synthetic tf in the NEW reference twist of the structure due to
% synthetic joint rotation only
w2n = exp_wmegaR*g_lk_lj0(1:3,1:3)*wi(:,2); g_sP1b = g_fP1a*g_lk_lj0;
xi2new = createtwist(w2n,g_sP1b(1:3,4)); %ξj'
w3n = exp_wmegaR*g_lk_lj0(1:3,1:3)*g_lj_li10(1:3,1:3)*wi(:,3); g_sf1 = g_fP1a*g_lk_lj0*g_lj_li10;
xi3new = createtwist(w3n,g_sf1(1:3,4)); %ξi+1'

% figure(test2_figure); % for visual evaluation
% xik_graph = drawtwist(twist_fP1a); hold on;
% xi2n_graph = drawtwist(xi2new); hold on;
% xi3n_graph = drawtwist(xi3new); hold on;

% Now, extract new relative twists
xi_j_new = inv(ad(g_fP1a*g_lk_li0))*xi2new;
xi_i1_new = inv(ad(g_fP1a*g_lk_li0))*xi3new;
xj_i1_new = inv(ad(g_fP1a*g_lk_lj0))*xi3new;

% After new structure is completed, the active-passive-active expos are
% constructed as usual
expi(:,:,1) = twistexp(xi(:,1), ti(1));
expi(:,:,2) = twistexp(xi2new, ti(2));
expi(:,:,3) = twistexp(xi3new, ti(3));

% For zero thetas the following are the g_s_tfs!!! synthetic joint CHANGES the
% zero tfs!!! So, g0 is for reference structure anatomy and gn is new
% structure
gn(:,:,1) = g0(:,:,1); % g_s_li0
gn_s_li0 = getTransform(robot3,config3,frame_2);
gn(:,:,2) = g_fP1a*g_lk_lj0; % g_s_lj0n = g_s_lk*g_lk_lj0
gn_s_lj0 = getTransform(robot3,config3,PseudoConnector1b_2);
gn(:,:,3) = g_fP1a*g_lk_lj0*g_lj_li10; % g_s_li10n
gn_s_li10 = getTransform(robot3,config3,frame1_2);
gn(:,:,4) = inv(gn(:,:,1))*gn(:,:,2); % g_li_lj0
gn_li_lj0 = getTransform(robot3,config3,PseudoConnector1b_2,frame_2);
gn(:,:,5) = inv(gn(:,:,2))*gn(:,:,3); % g_lj_li10
gn_lj_li10 = getTransform(robot3,config3,frame1_2,PseudoConnector1b_2);

gn(:,:,6) = inv(gn(:,:,1))*gn(:,:,3); % g_li_li10
gn_li_li10 = getTransform(robot3,config3,frame1_2,frame_2)
% Here is POE FKM for new structure
g(:,:,1) = expi(:,:,1)*gn(:,:,1); % g_s_li
g(:,:,2) = expi(:,:,1)*expi(:,:,2)*gn(:,:,2); % g_s_lj
g(:,:,3) = expi(:,:,1)*expi(:,:,2)*expi(:,:,3)*gn(:,:,3); % g_s_li1

%% Jacobians
% g_li_li1_0 = twistexp(xi_i1_0,0)*g0(:,:,6); % IT WORKS SINCE: g_li_li1_0 = g_li_li10

% Only metamorphic joint
% % % Spatial Jacobian is instantaneous twists with respect to {s}
% % % Js(:,1)=sym(xi(:,1)); % for sym only
% % Js(:,1)=xi(:,1);
% % % Js(:,2)=ad(expi(:,:,1))*xi(:,2);
% % Js(:,2)=ad(expi(:,:,1)*expi(:,:,2))*xi(:,3);
% % % Relative Jacobian: NOT STUDIED YET
% % Jr = xi(:,1);
% % Jr(:,2) = ad(expi(:,:,1)*expi(:,:,2))*xi_i1_0;

% Synthetic and metamorphic
Js(:,1) = xi(:,1);
Js(:,2) = ad(expi(:,:,1))*twist_fP1a;
Js(:,3) = ad(expi(:,:,1)*expi(:,:,2))*xi2new;
Js(:,4) = ad(expi(:,:,1)*expi(:,:,2)*expi(:,:,3))*xi3new;
% figure(test2_figure); % for visual evaluation
% xi_graph = drawtwist(Js(:,1)); hold on;
% xk_graph = drawtwist(Js(:,2)); hold on;
% xj_graph = drawtwist(Js(:,3)); hold on;
% xi1_graph = drawtwist(Js(:,4)); hold on;

%% Relative POE FKM
% % This is eq.3.7 since relative twist has been found for reference
% g_li_li1 = twistexp(xi_i1_0,ti(3))*g0(:,:,6); % as in eq.3.7 Murray p.94
% This is the noPOE FKM eq.3.8 only Tf matrix between i-->i+1 joints

 % for metamorphic only
% g_li_li1a = twistexp(xi_j,ti(2))*g0(:,:,4)*twistexp(xj_i1,ti(3))*g0(:,:,5);

% for synthetic and metamorphic
g_li_li1a = twistexp(xi_j_new,ti(2))*gn(:,:,4)*twistexp(xj_i1_new,ti(3))*gn(:,:,5); % for synthetic+metamorphic

%% Evaluation MyFKM-MATLAB
% metamorphic only test
% % % g_li_li1 = getTransform(robot1,[1.24 1.23]',frame1,frame)
% % g_li_li1_1 = getTransform(robot2,[0 0]',frame1_1,frame_1);
% % g_lj_li1_1 = getTransform(robot2,config,frame1_1,PseudoConnector1b_1);

% synthetic test
g_li_li1_1 = getTransform(robot3,config3,frame1_2,frame_2);
error_g_li_li1_1 = g_li_li1a - g_li_li1_1
g_s_li_1 = getTransform(robot3,config3,frame_2);
g_s_li1_1 = getTransform(robot3,config3,frame1_2);
g_s_lj_1 = getTransform(robot3,config3,PseudoConnector1b_2);
error_g_s_li = g(:,:,1)-g_s_li_1
error_g_s_lj = g(:,:,2)-g_s_lj_1
error_g_s_li1 = g(:,:,3)-g_s_li1_1

%% Graphical examination of twists
% g-> ξ
[xi_i1_abs th_i_i1_abs] = homtotwist(g_li_li1a); % the absolute transformation twist- READS structure change
g = g_li_li1a*inv(gn(:,:,6));
[xi_i1_rel th_i_i1_rel] = homtotwist(g); % the relative transformation twist - READS pseudojoint change
figure(test2_figure); % for visual evaluation
xi_i1_abs_graph = drawtwist(xi_i1_abs); hold on;
xi_i1_rel_graph = drawtwist(xi_i1_rel); hold on;

%% up to this for Lefteris GA %%

%% Convert from POE to DH - UNDER DEVELOPMENT
[DH_i_i1_1] = POE2DH_LiaoWu(xi_i1_abs,[0 0 0 0 1 0]');
[DH_i_i1_2] = POE2DH_LiaoWu(xi_i1_abs,xi(:,3));
[DH_i_i1_3] = POE2DH_LiaoWu(xi_i1_abs,Js(:,2));

[DH_i_i1_4] = POE2DH_LiaoWu(xi_i1_rel,[0 0 0 0 1 0]');
[DH_i_i1_5] = POE2DH_LiaoWu(xi_i1_rel,xi(:,3));
[DH_i_i1_6] = POE2DH_LiaoWu(xi_i1_rel,Js(:,2)); % this one gives solution

% [DH_i_i1_7] = POE2DH_LiaoWu(xi_i1_abs*th_i_i1_abs,[0 0 0 0 0 1]');
% [DH_i_i1_8] = POE2DH_LiaoWu(xi_i1_rel*th_i_i1_rel,[0 0 0 0 0 1]');

% Solve system
syms aim1 Lim1 di thi
% Solution of System 1
sol_thi = solve(DH_i_i1_1(4),thi); field1 = 'thi';  value1 = sol_thi;
sol_aim1 = solve(DH_i_i1_1(5),aim1); sol_aim1 = subs(sol_aim1,'thi',sol_thi); field2 = 'aim1';  value2 = sol_aim1;
sol_di = solve(DH_i_i1_1(1),di); sol_di = subs(sol_di,{'thi','aim1'},{sol_thi,sol_aim1}); field3 = 'di';  value3 = sol_di;
sol_Lim1 = solve(DH_i_i1_1(3),Lim1); sol_Lim1 = subs(sol_Lim1,{'thi','aim1','di'},{sol_thi,sol_aim1,sol_di}); field4 = 'Lim1';  value4 = sol_Lim1;
sol_sys1 = struct(field1,value1,field2,value2,field3,value3,field4,value4);
% Solution of System 2
sol_thi = solve(DH_i_i1_2(4),thi); field1 = 'thi';  value1 = sol_thi;
sol_aim1 = solve(DH_i_i1_2(5),aim1); sol_aim1 = subs(sol_aim1,'thi',sol_thi); field2 = 'aim1';  value2 = sol_aim1;
sol_di = solve(DH_i_i1_2(1),di); sol_di = subs(sol_di,{'thi','aim1'},{sol_thi,sol_aim1}); field3 = 'di';  value3 = sol_di;
sol_Lim1 = solve(DH_i_i1_2(3),Lim1); sol_Lim1 = subs(sol_Lim1,{'thi','aim1','di'},{sol_thi,sol_aim1,sol_di}); field4 = 'Lim1';  value4 = sol_Lim1;
sol_sys2 = struct(field1,value1,field2,value2,field3,value3,field4,value4);
% Solution of System 3
sol_thi = solve(DH_i_i1_3(4),thi); field1 = 'thi';  value1 = sol_thi;
sol_aim1 = solve(DH_i_i1_3(5),aim1); sol_aim1 = subs(sol_aim1,'thi',sol_thi); field2 = 'aim1';  value2 = sol_aim1;
sol_di = solve(DH_i_i1_3(1),di); sol_di = subs(sol_di,{'thi','aim1'},{sol_thi,sol_aim1}); field3 = 'di';  value3 = sol_di;
sol_Lim1 = solve(DH_i_i1_3(3),Lim1); sol_Lim1 = subs(sol_Lim1,{'thi','aim1','di'},{sol_thi,sol_aim1,sol_di}); field4 = 'Lim1';  value4 = sol_Lim1;
sol_sys3 = struct(field1,value1,field2,value2,field3,value3,field4,value4);
% Solution of System 4
sol_thi = solve(DH_i_i1_4(4),thi); field1 = 'thi';  value1 = sol_thi;
sol_aim1 = solve(DH_i_i1_4(5),aim1); sol_aim1 = subs(sol_aim1,'thi',sol_thi); field2 = 'aim1';  value2 = sol_aim1;
sol_di = solve(DH_i_i1_4(1),di); sol_di = subs(sol_di,{'thi','aim1'},{sol_thi,sol_aim1}); field3 = 'di';  value3 = sol_di;
sol_Lim1 = solve(DH_i_i1_4(3),Lim1); sol_Lim1 = subs(sol_Lim1,{'thi','aim1','di'},{sol_thi,sol_aim1,sol_di}); field4 = 'Lim1';  value4 = sol_Lim1;
sol_sys4 = struct(field1,value1,field2,value2,field3,value3,field4,value4);
% Solution of System 5
sol_thi = solve(DH_i_i1_5(4),thi); field1 = 'thi';  value1 = sol_thi;
sol_aim1 = solve(DH_i_i1_5(5),aim1); sol_aim1 = subs(sol_aim1,'thi',sol_thi); field2 = 'aim1';  value2 = sol_aim1;
sol_di = solve(DH_i_i1_5(1),di); sol_di = subs(sol_di,{'thi','aim1'},{sol_thi,sol_aim1}); field3 = 'di';  value3 = sol_di;
sol_Lim1 = solve(DH_i_i1_5(3),Lim1); sol_Lim1 = subs(sol_Lim1,{'thi','aim1','di'},{sol_thi,sol_aim1,sol_di}); field4 = 'Lim1';  value4 = sol_Lim1;
sol_sys5 = struct(field1,value1,field2,value2,field3,value3,field4,value4);
% Solution of System 6
sol_thi = solve(DH_i_i1_6(4),thi); field1 = 'thi';  value1 = sol_thi;
sol_aim1 = solve(DH_i_i1_6(5),aim1); sol_aim1 = subs(sol_aim1,'thi',sol_thi); field2 = 'aim1';  value2 = sol_aim1;
sol_di = solve(DH_i_i1_6(1),di); sol_di = subs(sol_di,{'thi','aim1'},{sol_thi,sol_aim1}); field3 = 'di';  value3 = sol_di;
sol_Lim1 = solve(DH_i_i1_6(3),Lim1); sol_Lim1 = subs(sol_Lim1,{'thi','aim1','di'},{sol_thi,sol_aim1,sol_di}); field4 = 'Lim1';  value4 = sol_Lim1;
sol_sys6 = struct(field1,value1,field2,value2,field3,value3,field4,value4);
% % % Solution of System 7
% % % sol_thi = solve(DH_i_i1_7(4),thi); field1 = 'thi';  value1 = sol_thi; %
% % % eq4 is 0==0
% % sol_aim1 = solve(DH_i_i1_7(5),aim1); sol_aim1 = subs(sol_aim1,'thi',sol_thi); field2 = 'aim1';  value2 = sol_aim1;
% % sol_di = solve(DH_i_i1_7(1),di); sol_di = subs(sol_di,{'thi','aim1'},{sol_thi,sol_aim1}); field3 = 'di';  value3 = sol_di;
% % sol_Lim1 = solve(DH_i_i1_7(3),Lim1); sol_Lim1 = subs(sol_Lim1,{'thi','aim1','di'},{sol_thi,sol_aim1,sol_di}); field4 = 'Lim1';  value4 = sol_Lim1;
% % sol_sys7 = struct(field1,value1,field2,value2,field3,value3,field4,value4);
% % % Solution of System 8
% % sol_thi = solve(DH_i_i1_8(4),thi); field1 = 'thi';  value1 = sol_thi;
% % sol_aim1 = solve(DH_i_i1_8(5),aim1); sol_aim1 = subs(sol_aim1,'thi',sol_thi); field2 = 'aim1';  value2 = sol_aim1;
% % sol_di = solve(DH_i_i1_8(1),di); sol_di = subs(sol_di,{'thi','aim1'},{sol_thi,sol_aim1}); field3 = 'di';  value3 = sol_di;
% % sol_Lim1 = solve(DH_i_i1_8(3),Lim1); sol_Lim1 = subs(sol_Lim1,{'thi','aim1','di'},{sol_thi,sol_aim1,sol_di}); field4 = 'Lim1';  value4 = sol_Lim1;
% % sol_sys8 = struct(field1,value1,field2,value2,field3,value3,field4,value4);