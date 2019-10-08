clear;
clc;
close all;
%% Metamorphic manipulator of 2 DoF with 2 PseudoJoints is modeled as:
%% Active - C01 - Passive - Passive -  Active => 0 - C01 - 1 - 1 - 0
%% C01: 6 Dof connection between active(0) and passive(1) link
%% C01: Px,Py,Pz,Rx,Ry,Rz the tf between frame-PseudoConnector1a rigid bodies
%%  - : is known fixed tf 

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
% robot urdf is built from: /home/nikos/matlab_ws/modular_dynamixel/kinematic_verification_011.xacro
[robot1] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/C0110.urdf'); % with 1 Pseudo 
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
PseudoConnector2a = char(robot1.BodyNames(6)); Ts6 = getTransform(robot1,config,PseudoConnector2a); drawframe(Ts6, scale); hold on;
PseudoConnector2b = char(robot1.BodyNames(7)); Ts7 = getTransform(robot1,config,PseudoConnector2b); drawframe(Ts7, scale); hold on;
frame1 = char(robot1.BodyNames(10)); Ts10 = getTransform(robot1,config,frame1); drawframe(Ts10, scale); hold on;
% g_st = getTransform(robot,config,t(source),s(target)) source: frame from --> target: frame to
% Zero Tfs from {s} frame
g_s_li0 = getTransform(robot1,config,frame); %g1
g_s_lk0 = getTransform(robot1,config,PseudoConnector1a);
g_s_lj10 = getTransform(robot1,config,PseudoConnector1b);
g_s_lj20 = getTransform(robot1,config,PseudoConnector2b); %rotz(-1.5708)
g_s_li10 = getTransform(robot1,config,frame1);
% Zero Relative Tfs
g_li_lk0 = getTransform(robot1,config,PseudoConnector1a,frame);
g_lk_lj10 = getTransform(robot1,config,PseudoConnector1b,PseudoConnector1a);
g_lj1_lj20 = getTransform(robot1,config,PseudoConnector2b,PseudoConnector1b); %rotz(-1.5708)
g_lj2_li10 = getTransform(robot1,config,frame1,PseudoConnector2b); % rotz(1.5708)
g_li_li10 = getTransform(robot1,config,frame1,frame);
g_li_lj10 = getTransform(robot1,config,PseudoConnector1b,frame);
g_lj1_li10 = getTransform(robot1,config,frame1,PseudoConnector1b);
g_li_lj20 = getTransform(robot1,config,PseudoConnector2b,frame);
% save to g matrix
g0(:,:,1) = g_s_li0;
g0(:,:,2) = g_s_lk0;
g0(:,:,3) = g_s_lj10;
g0(:,:,4) = g_s_lj20;
g0(:,:,5) = g_s_li10;
g0(:,:,6) = g_li_lk0;
g0(:,:,7) = g_lk_lj10; 
g0(:,:,8) = g_lj1_lj20;
g0(:,:,9) = g_lj2_li10;
g0(:,:,10) = g_li_li10;
g0(:,:,11) = g_li_lj10;
g0(:,:,12) = g_lj1_li10;
g0(:,:,13) = g_li_lj20;
%% Extracs initial geometry attributes
pi(:,1)= Ts3(1:3,4); % p1 on ξi
pi(:,2)= Ts5(1:3,4); % p2 on ξj1
pi(:,3)= Ts7(1:3,4); % p3 on ξj2
pi(:,4)= Ts10(1:3,4); % p4 on ξi+1
pi(:,5)= Ts4(1:3,4); % p5 on ξk
wi(:,1) = [0 0 1]';
wi(:,2) = [0 1 0]';
wi(:,3) = [1 0 0]';
wi(:,4) = [0 1 0]';
%% Create twists Numeric
xi(:,1) = createtwist(wi(:,1),pi(:,1)); %ξi
xi(:,2) = createtwist(wi(:,2),pi(:,2)); %ξj1
xi(:,3) = createtwist(wi(:,3),pi(:,3)); %ξj2
xi(:,4) = createtwist(wi(:,4),pi(:,4)); %ξi+1
%% Extract relative twists
% This are the relative twists of manipulator in reference
% anatomy-configuration, as from eq.3.10
xi_j1 = inv(ad(g_s_li0))*xi(:,2);
xj1_j2 = inv(ad(g_s_lj10))*xi(:,3);
xj2_i1 = inv(ad(g_s_lj20))*xi(:,4);
xi_i1 = inv(ad(g_s_li0))*xi(:,4);
%% Test2 anatomy: frame__PseudoConnector1a=[1.5708,0,0,0,0,0] θpj=0
%% tests synthetic_joint: frame_PseudoConnector1a
[robot3] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/C0110_2.urdf'); % with 2 Pseudo 
robot3.DataFormat = 'column';
robot3.Gravity = [0 0 -9.80665];
config3 = homeConfiguration(robot3);
test2_figure = figure;
show(robot3,config3,'PreservePlot',false);
hold on;
axis auto;
box on;
frame_2 = char(robot3.BodyNames(3)); Ts3_2 = getTransform(robot3,config3,frame_2); drawframe(Ts3_2, scale_active); hold on;
PseudoConnector1a_2 = char(robot3.BodyNames(4)); Ts4_2 = getTransform(robot3,config3,PseudoConnector1a_2); drawframe(Ts4_2, scale); hold on;
PseudoConnector1b_2 = char(robot3.BodyNames(5)); Ts5_2 = getTransform(robot3,config3,PseudoConnector1b_2); drawframe(Ts5_2, scale); hold on;
PseudoConnector2a_2 = char(robot3.BodyNames(6)); Ts6_2 = getTransform(robot3,config3,PseudoConnector2a_2); drawframe(Ts6_2, scale); hold on;
PseudoConnector2b_2 = char(robot3.BodyNames(7)); Ts7_2 = getTransform(robot3,config3,PseudoConnector2b_2); drawframe(Ts7_2, scale); hold on;
frame1_2 = char(robot3.BodyNames(10)); Ts10_2 = getTransform(robot3,config3,frame1_2); drawframe(Ts10_2, scale); hold on;

%% Create twists Symbolic
% pi(:,1)= sym('pi',[3 1]); % pi on ξai
% pi(:,2)= sym('pj1',[3 1]); % pj1 on ξpj1
% pi(:,3)= sym('pj2',[3 1]); % p1j on ξpj2
% pi(:,4)= sym('pi1',[3 1]); % p_i+1 on ξa_i+1
% wi(:,1)= sym('wi',[3 1]); % wi on ξi
% wi(:,2)= sym('wj1',[3 1]); % wj1 on ξpj1
% wi(:,3)= sym('wj2',[3 1]); % wj2 on ξpj2
% wi(:,4)= sym('wi1',[3 1]); % w_i+1 on ξ_i+1
%% Fwd Kinematic Mapping
% ti = sym('ti',[4 1], 'real');
% ti(1) = 'ti';
% ti(2) = 'tp1';
% ti(3) = 'tp2';
% ti(4) = 'ti1';
ti = [0 0 0 0]'; % from here i control the 4 angles: 1:active1 2:passive1 3:passive2 4:active2 
Rx011 = 1.5708; Ry011=0; Rz011=0; Px011=0; Py011=0.05; Pz011=0.05; % from here i control the 
% 6 Euler variables of the synthetic joint. These variables are ONLY LOCAL
% in the frame-PseudoConnector1a. i.e. Describe how "Pseudoconector1a"
% changes with respect to "frame". MUST be same with parameters defined in
% lines 15-21 in kinematic_verification_011.xacro
%% Here i test FKM of frame__PseudoConnector1a. This is the joint that
% genetic algorithm changes its EULER variables.
% My goal is to convert this variables to POE using twists
% g_fP1a = [R_fP1a p
%            0^T   1]
R_fP1a = rotz(Rz011)*roty(Ry011)*rotx(Rx011);
[wmegaR thetaR] = rotparam(R_fP1a); % this is SO(3) element-skew matrix and angle
exp_wmegaR = skewexp(wmegaR,thetaR); % ==R_fP1a ALWAYS the same
%  find twist of synthetic joint_it is "equivalent to the reference" since
%  twist must be built for reference structure
twist_fP1a = createtwist(wmegaR,[g_s_lk0(1,4)+Px011 g_s_lk0(2,4)+Py011 g_s_lk0(3,4)+Pz011]'); %ξk
% exp_fP1a = twistexp(twist_fP1a,thetaR);

g_fP1a = [exp_wmegaR [g_s_lk0(1,4)+Px011 g_s_lk0(2,4)+Py011 g_s_lk0(3,4)+Pz011]'; 0 0 0 1];
[twist_g_fP1a theta_g_fP1a] = homtotwist(g_fP1a); % this is SE(3) element that represents 
% the rigid motion of the structure change induced by synthetic_joint:frame_PseudoConnector1a

% In order expos to be computed, the initial wi,pi must be transformed due
% to the synthetic tf in the NEW reference twist of the structure due to
% synthetic joint rotation only
w2n = exp_wmegaR*g_lk_lj10(1:3,1:3)*wi(:,2); g_sP1b = g_fP1a*g_lk_lj10;
xi2new = createtwist(w2n,g_sP1b(1:3,4)); %ξj1'
% w3n = exp_wmegaR*g_lk_lj10(1:3,1:3)*g_lj1_lj20(1:3,1:3)*wi(:,3); g_sP2b = g_fP1a*g_lk_lj10*g_lj1_lj20;
w3n = exp_wmegaR*wi(:,3); g_sP2b = g_fP1a*g_lk_lj10*g_lj1_lj20;
xi3new = createtwist(w3n,g_sP2b(1:3,4)); %ξj2'
w4n = exp_wmegaR*g_lk_lj10(1:3,1:3)*g_lj1_lj20(1:3,1:3)*g_lj2_li10(1:3,1:3)*wi(:,4); g_sf1 = g_fP1a*g_lk_lj10*g_lj1_lj20*g_lj2_li10;
xi4new = createtwist(w4n,g_sf1(1:3,4)); %ξi1'

% figure(test2_figure); % for visual evaluation
% xik_graph = drawtwist(twist_fP1a); hold on;
% xi2n_graph = drawtwist(xi2new); hold on;
% xi3n_graph = drawtwist(xi3new); hold on;
% xi4n_graph = drawtwist(xi4new); hold on;

% After new structure is completed, the active-passive-active expos are
% constructed as usual
expi(:,:,1) = twistexp(xi(:,1), ti(1));
expi(:,:,2) = twistexp(xi2new, ti(2));
expi(:,:,3) = twistexp(xi3new, ti(3));
expi(:,:,4) = twistexp(xi4new, ti(4));

% For zero thetas the following are the g_s_tfs!!! synthetic joint CHANGES the
% zero tfs!!! So, g0 is for "reference structure+anatomy" and gn is "new
% structure - reference anatomy"
gn(:,:,1) = g0(:,:,1); %gn_s_li0
gn_s_li0 = getTransform(robot3,config3,frame_2);
gn(:,:,2) = g_fP1a*g_lk_lj10; %gn_s_lj10
gn_s_lj10 = getTransform(robot3,config3,PseudoConnector1b_2);
gn(:,:,3) = g_fP1a*g_lk_lj10*g_lj1_lj20; %gn_s_lj20
gn_s_lj20 = getTransform(robot3,config3,PseudoConnector2b_2);
gn(:,:,4) = g_fP1a*g_lk_lj10*g_lj1_lj20*g_lj2_li10; %gn_s_li10
gn_s_li10 = getTransform(robot3,config3,frame1_2);

gn(:,:,5) = inv(gn(:,:,1))*gn(:,:,2); % gn_li_lj10
gn_li_lj10 = getTransform(robot3,config3,PseudoConnector1b_2,frame_2);
gn(:,:,6) = inv(gn(:,:,2))*gn(:,:,3); % gn_lj1_lj20
gn_lj1_lj20 = getTransform(robot3,config3,PseudoConnector2b_2,PseudoConnector1b_2);
gn(:,:,7) = inv(gn(:,:,3))*gn(:,:,4); % gn_lj2_li10
gn_lj2_li10 = getTransform(robot3,config3,frame1_2,PseudoConnector2b_2);
gn(:,:,8) = inv(gn(:,:,1))*gn(:,:,4); % gn_li_li10
gn_li_li10 = getTransform(robot3,config3,frame1_2,frame_2);
gn(:,:,9) = inv(gn(:,:,1))*gn(:,:,3); % gn_li_lj20
gn_li_lj20 = getTransform(robot3,config3,PseudoConnector2b_2,frame_2);
gn(:,:,10) = inv(gn(:,:,2))*gn(:,:,4); % gn_lj1_li10
gn_lj1_li10 = getTransform(robot3,config3,frame1_2,PseudoConnector1b_2);

% Now, extract new relative twists
xs_i_new = inv(ad(eye(4)))*xi(:,1); %si
xi_j1_new = inv(ad(gn(:,:,1)))*xi2new; %
xi_j2_new = inv(ad(gn(:,:,1)))*xi3new; %
xj1_j2_new = inv(ad(gn(:,:,2)))*xi3new; %
xj2_i1_new = inv(ad(gn(:,:,3)))*xi4new; %
xi_i1_new = inv(ad(gn(:,:,1)))*xi4new; %

% Here is POE FKM for new structure. No synthetic expo is used since it is
% taken into account for changing the zero tfs
gn1(:,:,1) = expi(:,:,1)*gn(:,:,1); % g_s_li
gn1(:,:,2) = expi(:,:,1)*expi(:,:,2)*gn(:,:,2); % g_s_lj1
gn1(:,:,3) = expi(:,:,1)*expi(:,:,2)*expi(:,:,3)*gn(:,:,3); % g_s_lj2
gn1(:,:,4) = expi(:,:,1)*expi(:,:,2)*expi(:,:,3)*expi(:,:,4)*gn(:,:,4); % g_s_li1

%% Jacobians
% Synthetic and metamorphic
Js(:,1) = xi(:,1);
Js(:,2) = ad(expi(:,:,1))*twist_fP1a;
Js(:,3) = ad(expi(:,:,1))*xi2new;
Js(:,4) = ad(expi(:,:,1)*expi(:,:,2))*xi3new;
Js(:,5) = ad(expi(:,:,1)*expi(:,:,2)*expi(:,:,3))*xi4new;
figure(test2_figure); % for visual evaluation
xi_graph = drawtwist(Js(:,1)); hold on;
xk_graph = drawtwist(Js(:,2)); hold on;
xj1_graph = drawtwist(Js(:,3)); hold on;
xj2_graph = drawtwist(Js(:,4)); hold on;
xi1_graph = drawtwist(Js(:,5)); hold on;

%% Relative POE FKM - The most precious
g_li_li1a = twistexp(xi_j1_new,ti(2))*gn(:,:,5)*twistexp(xj1_j2_new,ti(3))*gn(:,:,6)*twistexp(xj2_i1_new,ti(4))*gn(:,:,7);
g_li_li1_1 = getTransform(robot3,config3,frame1_2,frame_2);
error_g_li_li1_1 = g_li_li1a - g_li_li1_1; %works

%% Graphical examination of twists
% g-> ξ for i->i1
[xi_i1_abs th_i_i1_abs] = homtotwist(g_li_li1a); % the absolute transformation twist- READS structure change
g = g_li_li1a*inv(gn(:,:,8));
[xi_i1_rel th_i_i1_rel] = homtotwist(g); % the relative transformation twist - READS pseudojoint change

% g-> ξ for s->i
[xs_i_abs th_s_i_abs] = homtotwist(gn1(:,:,1));
g0 = gn1(:,:,1)*inv(gn(:,:,1));
[xs_i_rel th_s_i_rel] = homtotwist(g0);
% for visual evaluation
figure(test2_figure); 
xi_i1_abs_graph = drawtwist(xi_i1_abs); hold on;
xi_i1_rel_graph = drawtwist(xi_i1_rel); hold on;

%% up to this for Lefteris GA %%