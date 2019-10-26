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
% load geom3d library
addpath('/home/nikos/matlab_ws/geom3d')
addpath('/home/nikos/matlab_ws/geom3d/geom3d')
addpath('/home/nikos/matlab_ws/geom2d/geom2d')
addpath('/home/nikos/matlab_ws/geom2d/utils')
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
% robot_DoF = 1;
%% Zero Structure-Anatomy-Configuration tfs
%% all pi,wi for twists,expos creation must be calculated from this
%% AND recalculated depending the synthetic variables!!!
scale = 0.025; scale_active = 0.075;
adaptor = char(robot1.BodyNames(2)); Ts2 = getTransform(robot1,config,adaptor); drawframe(Ts2, scale); hold on;
frame = char(robot1.BodyNames(3)); Ts3 = getTransform(robot1,config,frame); drawframe(Ts3, scale_active); hold on;
PseudoConnector1a = char(robot1.BodyNames(4)); Ts4 = getTransform(robot1,config,PseudoConnector1a); drawframe(Ts4, scale); hold on;
PseudoConnector1b = char(robot1.BodyNames(5)); Ts5 = getTransform(robot1,config,PseudoConnector1b); drawframe(Ts5, scale); hold on;
frame1 = char(robot1.BodyNames(8)); Ts8 = getTransform(robot1,config,frame1); drawframe(Ts8, scale); hold on;
TOOL = char(robot1.BodyNames(9)); Ts9 = getTransform(robot1,config,TOOL); drawframe(Ts9, scale); hold on;

% g_st = getTransform(robot,config,t(source),s(target)) source: frame from --> target: frame to
g_s_li0 = getTransform(robot1,config,frame);
g_s_lj0 = getTransform(robot1,config,PseudoConnector1b);
g_s_li10 = getTransform(robot1,config,frame1);
g_s_lk0 = getTransform(robot1,config,PseudoConnector1a);
g_s_TOOL = getTransform(robot1,config,TOOL);

g_li_lk0 = getTransform(robot1,config,PseudoConnector1a,frame);
g_lk_lj0 = getTransform(robot1,config,PseudoConnector1b,PseudoConnector1a);
g_lk_li0 = getTransform(robot1,config,PseudoConnector1b,frame);
g_lk_li10 = getTransform(robot1,config,PseudoConnector1b,frame1);
g_lk_TOOL = getTransform(robot1,config,TOOL,PseudoConnector1a);

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
g0(:,:,10) = g_lk_TOOL;
%% Extracs initial geometry attributes
pi(:,1)= Ts3(1:3,4); % p1 on ξi
pi(:,2)= Ts5(1:3,4); % p2 on ξj
pi(:,3)= Ts8(1:3,4); % p3 on ξi+1
pi(:,4)= Ts4(1:3,4); % p4 on ξk
wi(:,1) = [0 0 1]';
wi(:,2) = [0 1 0]';
wi(:,3) = [0 1 0]';
%% Create twists Numeric
xi(:,1) = createtwist(wi(:,1),pi(:,1)); %ξi
xi(:,2) = createtwist(wi(:,2),pi(:,2)); %ξj
xi(:,3) = createtwist(wi(:,3),pi(:,3)); %ξi+1
% save('SB10_zero_data','pi','wi','g0','xi'); % for kinematic_verification_S0101110.m in this folder

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
ti = sym('ti',[3 1], 'real');
ti(1) = 'ti';
ti(2) = 'tp1';
ti(3) = 'ti1';
% % ti = [0 -0.7854 0]'; % from here i control the 3 angles: 1:active1 2:passive1 3:active2

% R01 = sym('R01',[3 1], 'real');
% R01(1) = 'Rx01'; R01(2) = 'Ry01'; R01(3) = 'Rz01';
R01(1) = 0; R01(2) = 0; R01(3) = 0;
% P01 = sym('P01',[3 1], 'real');
% P01(1) = 'Px01'; P01(2) = 'Py01'; P01(3) = 'Pz01';
P01(1) = 0.1; P01(2) = 0; P01(3) = 0;
% Rx01 = 0; Ry01=0; Rz01=0; Px01=0.1; Py01=0; Pz01=0; % from here i control the 

% 6 Euler variables of the synthetic joint. These variables are ONLY LOCAL
% in the frame-PseudoConnector1a. i.e. Describe how "Pseudoconector1a"
% changes with respect to "frame". MUST be same with parameters defined in
% lines 14-20 in kinematic_verification_01.xacro

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
% My goal is to convert this variables to POE using twists
% g_fP1a = [R_fP1a p
%            0^T   1]

R_fP1a = rotz(R01(3))*roty(R01(2))*rotx(R01(1));
% R_fP1a = rotz(Rz01)*roty(Ry01)*rotx(Rx01);

[wmegaR thetaR] = rotparam(R_fP1a); % this is SO(3) element-skew matrix and angle
exp_wmegaR = skewexp(wmegaR,thetaR); % ==R_fP1a ALWAYS the same
%  find twist of synthetic joint_it is "equivalent to the reference" since
%  twist must be built for reference structure

twist_fP1a = createtwist(wmegaR,[g_s_lk0(1,4)+P01(1) g_s_lk0(2,4)+P01(2) g_s_lk0(3,4)+P01(3)]'); %ξk
% % twist_fP1a = createtwist(wmegaR,[g_s_lk0(1,4)+Px01 g_s_lk0(2,4)+Py01 g_s_lk0(3,4)+Pz01]'); %ξk
% exp_fP1a = twistexp(twist_fP1a,thetaR);
% g_fP1a = exp_fP1a*g_s_lk0;

g_fP1a = [exp_wmegaR [g_s_lk0(1,4)+P01(1) g_s_lk0(2,4)+P01(2) g_s_lk0(3,4)+P01(3)]'; 0 0 0 1];
% % g_fP1a = [exp_wmegaR [g_s_lk0(1,4)+Px01 g_s_lk0(2,4)+Py01 g_s_lk0(3,4)+Pz01]'; 0 0 0 1];
% % [twist_g_fP1a theta_g_fP1a] = homtotwist(g_fP1a); % this is SE(3) element that represents 
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

% After new structure is completed, the active-passive-active expos are
% constructed as usual
expi(:,:,1) = twistexp(xi(:,1), ti(1));
expi(:,:,2) = twistexp(xi2new, ti(2));
expi(:,:,3) = twistexp(xi3new, ti(3));

% For zero thetas the following are the g_s_tfs!!! synthetic joint CHANGES the
% zero tfs!!! So, g0 is for reference structure anatomy and gn is new
% structure
gn(:,:,1) = sym(g0(:,:,1)); % g_s_li0
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
gn_li_li10 = getTransform(robot3,config3,frame1_2,frame_2);
gn(:,:,7) = g_fP1a*g_lk_TOOL;

% Now, extract new relative twists
xs_i_new = iad(eye(4))*xi(:,1);
xi_j_new = iad(gn(:,:,1))*xi2new;
xi_i1_new = iad(gn(:,:,1))*xi3new;
xj_i1_new = iad(gn(:,:,2))*xi3new;

% xi_j_new_dot = iad(gn(:,:,4)) * xi_j_new;
% Cim1 = inv(gn(:,:,4));
% Ci = gn(:,:,5);

% Here is POE FKM for new structure
gn1(:,:,1) = expi(:,:,1)*gn(:,:,1); % g_s_li
gn1(:,:,2) = expi(:,:,1)*expi(:,:,2)*gn(:,:,2); % g_s_lj
gn1(:,:,3) = expi(:,:,1)*expi(:,:,2)*expi(:,:,3)*gn(:,:,3); % g_s_li1
gn1(:,:,4) = expi(:,:,1)*expi(:,:,2)*expi(:,:,3)*gn(:,:,7); % g_s_TOOL
%% Jacobians
% g_li_li1_0 = twistexp(xi_i1_0,0)*g0(:,:,6); % IT WORKS SINCE: g_li_li1_0 = g_li_li10

% Only metamorphic joint
% % % Spatial Jacobian is instantaneous twists with respect to {s}
% % % Js(:,1)=sym(xi(:,1)); % for sym only
% % Js(:,1)=x       i(:,1);
% % % Js(:,2)=ad(expi(:,:,1))*xi(:,2);
% % Js(:,2)=ad(expi(:,:,1)*expi(:,:,2))*xi(:,3);
% % % Relative Jacobian: NOT STUDIED YET
% % Jr = xi(:,1);
% % Jr(:,2) = ad(expi(:,:,1)*expi(:,:,2))*xi_i1_0;

% Synthetic and metamorphic
Js(:,1) = sym(xi(:,1));
Js(:,2) = ad(expi(:,:,1))*twist_fP1a;
Js(:,3) = ad(expi(:,:,1))*xi2new;
Js(:,4) = ad(expi(:,:,1)*expi(:,:,2))*xi3new;
% % figure(test2_figure); % for visual evaluation
% % xi_graph = drawtwist(Js(:,1)); hold on;
% % xk_graph = drawtwist(Js(:,2)); hold on;
% % xj_graph = drawtwist(Js(:,3)); hold on;
% % xi1_graph = drawtwist(Js(:,4)); hold on;

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
% % g_li_li1_1 = getTransform(robot3,config3,frame1_2,frame_2);
% % error_g_li_li1_1 = g_li_li1a - g_li_li1_1;
% % g_s_li_1 = getTransform(robot3,config3,frame_2);
% % g_s_li1_1 = getTransform(robot3,config3,frame1_2);
% % g_s_lj_1 = getTransform(robot3,config3,PseudoConnector1b_2);
% % error_g_s_li = gn1(:,:,1)-g_s_li_1;
% % error_g_s_lj = gn1(:,:,2)-g_s_lj_1;
% % error_g_s_li1 = gn1(:,:,3)-g_s_li1_1;

%% Graphical examination of twists
% g-> ξ for i->i1 as described in p.45 Murray
% % % [xi_i1_abs th_i_i1_abs] = homtotwist(g_li_li1a); % the absolute transformation twist- READS structure change
% % % g = g_li_li1a*inv(gn(:,:,6));
% % % [xi_i1_rel th_i_i1_rel] = homtotwist(g); % the relative transformation twist - READS pseudojoint change
% % % 
% % % % g-> ξ for s->i
% % % [xs_i_abs th_s_i_abs] = homtotwist(gn1(:,:,1));
% % % g0 = gn1(:,:,1)*inv(gn(:,:,1));
% % % [xs_i_rel th_s_i_rel] = homtotwist(g0);
% for visual evaluation
% figure(test2_figure); 
% xi_i1_abs_graph = drawtwist(xi_i1_abs); hold on;
% xi_i1_rel_graph = drawtwist(xi_i1_rel); hold on;
%% up to this for Lefteris GA %%
% [Js; g_li_li1a]
%% Convert from POE to DH - UNDER DEVELOPMENT

%% Graphical DH solution
% create figure for graphical DH representation
% % DH_figure = figure;
% % figure(DH_figure);
% % drawframe(gn1(:,:,1), scale_active); hold on; % gsli
% % text(gn1(1,4,1), gn1(2,4,1), gn1(3,4,1),'\leftarrow g_{sli}'), hold on;
% % drawframe(gn1(:,:,3), scale_active); hold on; % gsli1
% % text(gn1(1,4,3), gn1(2,4,3), gn1(3,4,3),'\leftarrow g_{sli1}'), hold on;
% % drawframe(gn1(:,:,4), scale_active); hold on; % gsli1
% % text(gn1(1,4,4), gn1(2,4,4), gn1(3,4,4),'\leftarrow g_{sTOOL}'), hold on;
% % 
% % xi_graph = drawtwist(Js(:,1)); hold on;
% % xi1_graph = drawtwist(Js(:,4)); hold on;
% % p1_1 = [xi_graph.XData(1) xi_graph.YData(1) xi_graph.ZData(1) ]';
% % p1_2 = [xi_graph.XData(2) xi_graph.YData(2) xi_graph.ZData(2) ]';
% % text(p1_2(1), p1_2(2), p1_2(3),'\leftarrow ξ_i'), hold on;
% % p2_1 = [xi1_graph.XData(1) xi1_graph.YData(1) xi1_graph.ZData(1) ]';
% % p2_2 = [xi1_graph.XData(2) xi1_graph.YData(2) xi1_graph.ZData(2) ]';
% % text(p2_2(1), p2_2(2), p2_2(3),'\leftarrow ξ_{i+1}'), hold on;

% im1Tx(Lim1) if im1->i or iTx(Li) if i->i1
q4 = sym('q4',[3 1], 'real'); q4(1)='q4x'; q4(2)='q4y'; q4(3)='q4z';
eqn = cross(-Js(4:6,4),q4)-Js(1:3,4) == 0;
a = fopen('eqn2.txt','w');
fprintf(a,'%s\n',char(eqn(2)));
fclose(a);
sol_qz = solve(eqn(1),q4(3)); % returns qz
% sol_qz2 = solve(eqn(2),q4(3));
sol_qx = solve(eqn(3),q4(1)); % returns qx~f(qy)

q4x1 = subs(sol_qx,{'q4y'},{0});
q4x2 = subs(sol_qx,{'q4y'},{1});
q4y1 = 0;
q4y2 =1;
q4z1 = sol_qz;
q4z2 = sol_qz;
p2_1 = [q4x1 q4y1 q4z1]';
p2_2 = [q4x2 q4y2 q4z2]';
p1_1 = [0 0 1]';
p1_2 = [0 0 -1]';
[Li, d, Si, Si1] = DistBetween2Segment_symfriendly(p1_1, p1_2, p2_1, p2_2); %[shortestdistance,directionvector,closepointonA,closepointonB] % L2
% % plot3([Si(1) Si1(1)],[Si(2) Si1(2)],[Si(3) Si1(3)]), hold on

%% added for POE->DH working on xi_i working only for tpj=1.5708
zero_x_i = -d/norm(d);
zero_z_i = wi(:,1);
zero_y_i = cross(zero_z_i,zero_x_i); % k^ x i^ = j^
zero_T_i = [zero_x_i zero_y_i zero_z_i Si; 0 0 0 1];
% % figure(DH_figure);
% % drawframe(zero_T_i, 0.05, true); hold on; % DHi
Ci = inv(zero_T_i)*gn1(:,:,1); 
%% working on xi_i1'
% set arbitrary point on xi_i1
nSi1 = Si1; % y=0.5 is arbitrary
zero_x_i1 = gn1(1:3,3,4); % arbitrary
zero_z_i1 = Js(4:6,4);
zero_y_i1 = cross(zero_z_i1,zero_x_i1); % k^ x i^ = j^
zero_T_i1 = [zero_x_i1 zero_y_i1 zero_z_i1 nSi1; 0 0 0 1];
% % figure(DH_figure);
% % drawframe(zero_T_i1, 0.05, true); hold on; % DHi1
Ci1 = inv(zero_T_i1)*gn1(:,:,3); 
% It must be: iMi1 = Ci*g_li_li1a*Ci1
% iMi1 = ModifiedDHmatrix(ti(2),-1.5708,Li,0); % first way as i knew the DH params
ver_iMi1 = Ci*g_li_li1a*inv(Ci1); % extracts DH matrix from POE and known geometry