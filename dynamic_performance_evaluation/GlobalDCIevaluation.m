clear;
clc;
close all;

%% load libraries
% load Murray kinematics
addpath('/home/nikos/matlab_ws/kinematics/robotlinks')
addpath('/home/nikos/matlab_ws/kinematics/screws') 
addpath('/home/nikos/matlab_ws/kinematics/util')
% load Js-Jb calculation function
addpath('/home/nikos/matlab_ws/project_ABBpaper/matlab_files')

%% Reference anatomy
% robot urdf is built from: /home/nikos/matlab_ws/modular_dynamixel/3DoF_2Pseudo_MMD_for_dynamics.xacro
[robot1] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/reference_3DoF_2Pseudo.urdf'); % with 2 Pseudo pairs 
robot1.DataFormat = 'column';
robot1.Gravity = [0 0 -9.80665];
config = homeConfiguration(robot1);
reference_figure = figure;

%% Plot in figure
figure(reference_figure);
show(robot1,config,'PreservePlot',false);
title('Reference Anatomy [0,0,0,0]'); hold on; axis auto; box on;
robot_DoF = 3;
[robot0_links,CoM_robot0_links,gsli0,gsbj0,M0_CoM,M0_s_CoM] = robot_links_subtree_new(robot1,config,robot_DoF);

%%  Extract Geometry points for each body from FKP in reference anatomy and zero configuration!!!
robot = robot1;
config = [0 0 0]'; 
scale = 0.025;
scale_active = 0.075;
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
% drawframe(Ts3, scale_active,'true')  
% hold on;
PseudoConnector1a = char(robot.BodyNames(4));
Ts4 = getTransform(robot,config,PseudoConnector1a);
% text(Ts2(1,4),Ts2(2,4),Ts2(3,4),'\leftarrow AM1');
drawframe(Ts4, scale_active,'true') % Active joint1
hold on;
PseudoConnector1b = char(robot.BodyNames(5));
Ts5 = getTransform(robot,config,PseudoConnector1b);
drawframe(Ts5, scale)  % Pseudo joint1
hold on;
PseudoConnector2a = char(robot.BodyNames(6));
Ts6 = getTransform(robot,config,PseudoConnector2a);
% drawframe(Ts6, scale) 
% hold on;
PseudoConnector2b = char(robot.BodyNames(7));
Ts7 = getTransform(robot,config,PseudoConnector2b);
drawframe(Ts7, scale) % Pseudo joint2
hold on;
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
drawframe(Ts10, scale_active,'true') % Active joint2
hold on;
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
drawframe(Ts13, scale) ; % Pseudo joint3
hold on;
PseudoConnector4a = char(robot.BodyNames(14));
Ts14 = getTransform(robot,config,PseudoConnector4a);
% drawframe(Ts14, scale)
% hold on;
PseudoConnector4b = char(robot.BodyNames(15));
Ts15 = getTransform(robot,config,PseudoConnector4b);
drawframe(Ts15, scale) % Pseudo joint4
hold on;
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
drawframe(Ts18, scale_active,'true') % Active joint3
hold on;
idler2 = char(robot.BodyNames(19));
Ts19 = getTransform(robot,config,idler2); % TCP
drawframe(Ts19, scale)
hold on;
%% Pi points are extracted from reference Anatomy in zero configuration!!!
pi(:,1)= Ts4(1:3,4); % p1 on ξa1
pi(:,2)= Ts10(1:3,4); % p2 on ξa2
pi(:,3)= Ts18(1:3,4); % p6 on ξa3
pi(:,4)= Ts5(1:3,4); % p4 on ξp1
pi(:,5)= Ts7(1:3,4); % p5 on ξp2
pi(:,6)= Ts13(1:3,4); % p6 on ξp3
pi(:,7)= Ts15(1:3,4); % p7 on ξp4
%% Active screws
wi(:,1) = [0 0 1]';
wi(:,2) = [0 1 0]';
wi(:,3) = [0 1 0]';
%% Pseudo screws 
wpi(:,1) = [0 1 0]';
wpi(:,2) = [1 0 0]';
wpi(:,3) = [0 1 0]';
wpi(:,4) = [1 0 0]';
%% Active twists
xi_ai(:,1) = createtwist(wi(:,1),pi(:,1));
xi_ai(:,2) = createtwist(wi(:,2),pi(:,2));
xi_ai(:,3) = createtwist(wi(:,3),pi(:,3));
% xi_ai = sym('xi',[6 2], 'real');
%% Pseudo twists
xi_pi(:,1) = createtwist(wpi(:,1),pi(:,4));
xi_pi(:,2) = createtwist(wpi(:,2),pi(:,5));
xi_pi(:,3) = createtwist(wpi(:,3),pi(:,6));
xi_pi(:,4) = createtwist(wpi(:,4),pi(:,7));

%% DCI evaluation is conducted using gamultiobj. The resulted anatomies are 
%% evaluated using plots of the Mb(t2,t3)|tp1,2,3,4 (extracted from GlobalMinCDI)

%% xacro file for robot build is: 
%  /home/nikos/matlab_ws/modular_dynamixel/3DoF_2Pseudo_MMD_for_dynamics.xacro

% parent matlab file is /modular_dynamixel/TEST_3DoF_MetamorphicDynamixel.m
% form this file /modular_dynamixel/CallGlobalDPMsolvers_3DoF is called
% for Global DCI using gamultiobj (Solver 6)

% First a random anatomy is generated as in l.399 TEST_3DoF_MD_SYM.m
% and the mass matrix is calculated.
% Then the extracted anatomies are visualized and 3 graphs of elements
% Mij,Mjk,Mik are plotted for results evaluation vs the corresponding
% random values as in l.442-480 TEST_3DoF_MD_SYM.m

%% Random Anatomy
[robot_RAND] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/RAND_AP2AP2A.urdf'); 
config_RAND = homeConfiguration(robot_RAND);
figure; show(robot_RAND,config_RAND,'PreservePlot',false); title('Random Anatomy [0.5,-0.5,1,-1]'); hold on; axis auto; box on;

 tp0 = [0.5 -0.5 1 -1]';
cnt=0; cnt2 = 0;
for t2=-2.4:0.08:2.4
    cnt2 = cnt2+1;
    cnt3 = 0;
    for t3=-2.4:0.08:2.4
        cnt3=cnt3+1;
        cnt = cnt+1;
        [~,Pi] = CalculatePseudoExponentials_3DoF(xi_pi,tp0);
        Exp_ai(:,:,1) = twistexp(xi_ai(:,1), 0);
        Exp_ai(:,:,2) = twistexp(xi_ai(:,2), t2);
        Exp_ai(:,:,3) = twistexp(xi_ai(:,3), t3);
%         % Jacobians
%         [~,Jb,~] = CalculateMetamorphicJacobians_3DoF([0 t2 t3]',xi_ai,tp0,xi_pi,Pi,gst0);
%         qb = [0 0 0]';
%         [J33_new] = CalculateSquareTCPJacobian_3DoF(Jb,qb);
        % Compute CoM Jacobian
        % links' Jacobians
        gsli(:,:,1) = Exp_ai(:,:,1)*gsli0(:,:,1);
        gsli(:,:,2) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*gsli0(:,:,2);
        gsli(:,:,3) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*Pi(:,:,2)*Exp_ai(:,:,3)*gsli0(:,:,3);
        for i=1:3
            [JBsli(:,:,i),JSsli(:,:,i)] = Jbody_CoM_3DoF(xi_ai, Exp_ai, Pi, gsli0, gsli, i);
        end
        [M_b_RANDff(:,:,cnt)] = CalculateOnlyBodyMassMatrix(JBsli,M0_CoM);
        M_b_RAND12(cnt3,cnt2) = abs(M_b_RANDff(1,2,cnt));
        M_b_RAND13(cnt3,cnt2) = abs(M_b_RANDff(1,3,cnt));
        M_b_RAND23(cnt3,cnt2) = abs(M_b_RANDff(2,3,cnt));
%         Wd(cnt) = CalculateDynamicManipulabilityIndex(J33_new,M_b_REFff);
    end
end
%% Min DCI anatomies as extracted CallGlobalDPMsolvers/GlobalMinDCI(6)
%% gamultiobj results are presented in:
%  /modular_dynamixel/DynamicPerformanceEvaluation_logfiles_from_CallGlobalDPMsolvers/GDCI_1
tp(1,:) = [-1.5708  0  -0.7854  -1.5708];     % GDCI1(1,3,2,1)
tp(2,:) = [-1.5708  0   0.7854  -0.7854];     % GDCI2(1,3,4,2)
tp(3,:) = [      0  0        0  -1.5708];     % GDCI3(3,3,3,1)
tp(4,:) = [      0  0  -0.7854  -1.5708];     % GDCI4(3,3,2,1)
tp(5,:) = [-1.5708  0   1.5708        0];     % GDCI5(1,3,5,3)
%% GDCI1
[robot_GDCI1] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/GDCI1.urdf'); 
config_GDCI1 = homeConfiguration(robot_GDCI1);
figure; show(robot_GDCI1,config_GDCI1,'PreservePlot',false); title('Anatomy GDCI1 [-1.5708,0,-0.7854,-1.5708]'); hold on; axis auto; box on;
cnt=0; cnt2 = 0;
for t2=-2.4:0.08:2.4
    cnt2 = cnt2+1;
    cnt3 = 0;
    for t3=-2.4:0.08:2.4
        cnt3=cnt3+1;
        cnt = cnt+1;
        [~,Pi] = CalculatePseudoExponentials_3DoF(xi_pi,tp(1,:));
        Exp_ai(:,:,1) = twistexp(xi_ai(:,1), 0);
        Exp_ai(:,:,2) = twistexp(xi_ai(:,2), t2);
        Exp_ai(:,:,3) = twistexp(xi_ai(:,3), t3);
        % links' Jacobians
        gsli(:,:,1) = Exp_ai(:,:,1)*gsli0(:,:,1);
        gsli(:,:,2) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*gsli0(:,:,2);
        gsli(:,:,3) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*Pi(:,:,2)*Exp_ai(:,:,3)*gsli0(:,:,3);
        for i=1:3
            [JBsli(:,:,i),JSsli(:,:,i)] = Jbody_CoM_3DoF(xi_ai, Exp_ai, Pi, gsli0, gsli, i);
        end
        [M_b_GDCI1ff(:,:,cnt)] = CalculateOnlyBodyMassMatrix(JBsli,M0_CoM);
        M_b_GDCI1_12(cnt3,cnt2) = M_b_GDCI1ff(1,2,cnt);
        M_b_GDCI1_13(cnt3,cnt2) = M_b_GDCI1ff(1,3,cnt);
        M_b_GDCI1_23(cnt3,cnt2) = M_b_GDCI1ff(2,3,cnt);
    end
end
%% GDCI2
[robot_GDCI2] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/GDCI2.urdf'); 
config_GDCI2 = homeConfiguration(robot_GDCI2);
figure; show(robot_GDCI2,config_GDCI2,'PreservePlot',false); title('Anatomy GDCI2 [-1.5708,0,0.7854,-0.7854]'); hold on; axis auto; box on;
cnt=0; cnt2 = 0;
for t2=-2.4:0.08:2.4
    cnt2 = cnt2+1;
    cnt3 = 0;
    for t3=-2.4:0.08:2.4
        cnt3=cnt3+1;
        cnt = cnt+1;
        [~,Pi] = CalculatePseudoExponentials_3DoF(xi_pi,tp(2,:));
        Exp_ai(:,:,1) = twistexp(xi_ai(:,1), 0);
        Exp_ai(:,:,2) = twistexp(xi_ai(:,2), t2);
        Exp_ai(:,:,3) = twistexp(xi_ai(:,3), t3);
        % links' Jacobians
        gsli(:,:,1) = Exp_ai(:,:,1)*gsli0(:,:,1);
        gsli(:,:,2) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*gsli0(:,:,2);
        gsli(:,:,3) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*Pi(:,:,2)*Exp_ai(:,:,3)*gsli0(:,:,3);
        for i=1:3
            [JBsli(:,:,i),JSsli(:,:,i)] = Jbody_CoM_3DoF(xi_ai, Exp_ai, Pi, gsli0, gsli, i);
        end
        [M_b_GDCI2ff(:,:,cnt)] = CalculateOnlyBodyMassMatrix(JBsli,M0_CoM);
        M_b_GDCI2_12(cnt3,cnt2) = M_b_GDCI2ff(1,2,cnt);
        M_b_GDCI2_13(cnt3,cnt2) = M_b_GDCI2ff(1,3,cnt);
        M_b_GDCI2_23(cnt3,cnt2) = M_b_GDCI2ff(2,3,cnt);
    end
end
%% GDCI3
[robot_GDCI3] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/GDCI3.urdf'); 
config_GDCI3 = homeConfiguration(robot_GDCI3);
figure; show(robot_GDCI3,config_GDCI3,'PreservePlot',false); title('Anatomy GDCI3 [0,0,0,-1.5708]'); hold on; axis auto; box on;
cnt=0; cnt2 = 0;
for t2=-2.4:0.08:2.4
    cnt2 = cnt2+1;
    cnt3 = 0;
    for t3=-2.4:0.08:2.4
        cnt3=cnt3+1;
        cnt = cnt+1;
        [~,Pi] = CalculatePseudoExponentials_3DoF(xi_pi,tp(3,:));
        Exp_ai(:,:,1) = twistexp(xi_ai(:,1), 0);
        Exp_ai(:,:,2) = twistexp(xi_ai(:,2), t2);
        Exp_ai(:,:,3) = twistexp(xi_ai(:,3), t3);
        % links' Jacobians
        gsli(:,:,1) = Exp_ai(:,:,1)*gsli0(:,:,1);
        gsli(:,:,2) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*gsli0(:,:,2);
        gsli(:,:,3) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*Pi(:,:,2)*Exp_ai(:,:,3)*gsli0(:,:,3);
        for i=1:3
            [JBsli(:,:,i),JSsli(:,:,i)] = Jbody_CoM_3DoF(xi_ai, Exp_ai, Pi, gsli0, gsli, i);
        end
        [M_b_GDCI3ff(:,:,cnt)] = CalculateOnlyBodyMassMatrix(JBsli,M0_CoM);
        M_b_GDCI3_12(cnt3,cnt2) = M_b_GDCI3ff(1,2,cnt);
        M_b_GDCI3_13(cnt3,cnt2) = M_b_GDCI3ff(1,3,cnt);
        M_b_GDCI3_23(cnt3,cnt2) = M_b_GDCI3ff(2,3,cnt);
    end
end
%% GDCI4
[robot_GDCI4] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/GDCI4.urdf'); 
robot_GDCI4.DataFormat = 'column';
config_GDCI4 = homeConfiguration(robot_GDCI4);
figure; show(robot_GDCI4,config_GDCI4,'PreservePlot',false); title('Anatomy GDCI4 [0,0,-0.7854,-1.5708]'); hold on; axis auto; box on;
cnt=0; cnt2 = 0;
for t2=-2.4:0.08:2.4
    cnt2 = cnt2+1;
    cnt3 = 0;
    for t3=-2.4:0.08:2.4
        cnt3=cnt3+1;
        cnt = cnt+1;
        [~,Pi] = CalculatePseudoExponentials_3DoF(xi_pi,tp(4,:));
        Exp_ai(:,:,1) = twistexp(xi_ai(:,1), 0);
        Exp_ai(:,:,2) = twistexp(xi_ai(:,2), t2);
        Exp_ai(:,:,3) = twistexp(xi_ai(:,3), t3);
        % links' Jacobians
        gsli(:,:,1) = Exp_ai(:,:,1)*gsli0(:,:,1);
        gsli(:,:,2) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*gsli0(:,:,2);
        gsli(:,:,3) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*Pi(:,:,2)*Exp_ai(:,:,3)*gsli0(:,:,3);
        for i=1:3
            [JBsli(:,:,i),JSsli(:,:,i)] = Jbody_CoM_3DoF(xi_ai, Exp_ai, Pi, gsli0, gsli, i);
        end
        [M_b_GDCI4ff(:,:,cnt)] = CalculateOnlyBodyMassMatrix(JBsli,M0_CoM);
        M_b_GDCI4_12(cnt3,cnt2) = abs(M_b_GDCI4ff(1,2,cnt));
        M_b_GDCI4_13(cnt3,cnt2) = abs(M_b_GDCI4ff(1,3,cnt));
        M_b_GDCI4_23(cnt3,cnt2) = abs(M_b_GDCI4ff(2,3,cnt));
    end
end
%% GDCI5
[robot_GDCI5] = importrobot('/home/nikos/matlab_ws/modular_dynamixel/GDCI5.urdf'); 
robot_GDCI5.DataFormat = 'column';
config_GDCI5 = homeConfiguration(robot_GDCI5);
figure; show(robot_GDCI5,config_GDCI5,'PreservePlot',false); title('Anatomy GDCI5 [-1.5708,0,1.5708,0]'); hold on; axis auto; box on;
[~,~,gsl50,~,M50_CoM,~] = robot_links_subtree_new(robot_GDCI5,config_GDCI5,robot_DoF);

cnt=0; cnt2 = 0;
% % [t2,t3] = meshgrid(-2.4:0.08:2.4,-2:0.08:2);
% % % vassilis
% %  M_b_GDCI6_12 = zeros(size(t2));
% % for i=1:size(t2,1)
% %     for j=1:size(t2,2)
% %         
% %         [~,Pi] = CalculatePseudoExponentials_3DoF(xi_pi,tp(5,:));
% %         Exp_ai(:,:,1) = twistexp(xi_ai(:,1), 0);
% %         Exp_ai(:,:,2) = twistexp(xi_ai(:,2), t2(i,j));
% %         Exp_ai(:,:,3) = twistexp(xi_ai(:,3), t3(i,j));
% %         % links' Jacobians
% %         gsli(:,:,1) = Exp_ai(:,:,1)*gsli0(:,:,1);
% %         gsli(:,:,2) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*gsli0(:,:,2);
% %         gsli(:,:,3) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*Pi(:,:,2)*Exp_ai(:,:,3)*gsli0(:,:,3);
% %         for k=1:3
% %             [JBsli(:,:,k),JSsli(:,:,k)] = Jbody_CoM_3DoF(xi_ai, Exp_ai, Pi, gsli0, gsli, k);
% %         end
% %         [M_b_GDCI6ff(:,:)] = CalculateOnlyBodyMassMatrix(JBsli,M0_CoM);
% %         M_b_GDCI6_12(i,j) = M_b_GDCI6ff(1,2);
% %         M_b_GDCI6_13(i,j) = M_b_GDCI6ff(1,3);
% %         M_b_GDCI6_23(i,j) = M_b_GDCI6ff(2,3);
% %     end
% % end
% % % end vassilis
for t2=-2.4:0.08:2.4 %51
    cnt2 = cnt2+1;
    cnt3 = 0;
    for t3=-2.4:0.08:2.4 %61
        cnt3=cnt3+1;
        cnt = cnt+1;
        [~,Pi] = CalculatePseudoExponentials_3DoF(xi_pi,tp(5,:));
        Exp_ai(:,:,1) = twistexp(xi_ai(:,1), 0);
        Exp_ai(:,:,2) = twistexp(xi_ai(:,2), t2);
        Exp_ai(:,:,3) = twistexp(xi_ai(:,3), t3);
        % links' Jacobians
        gsli(:,:,1) = Exp_ai(:,:,1)*gsli0(:,:,1);
        gsli(:,:,2) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*gsli0(:,:,2);
        gsli(:,:,3) = Exp_ai(:,:,1)*Pi(:,:,1)*Exp_ai(:,:,2)*Pi(:,:,2)*Exp_ai(:,:,3)*gsli0(:,:,3);
        for i=1:3
            [JBsli(:,:,i),JSsli(:,:,i)] = Jbody_CoM_3DoF(xi_ai, Exp_ai, Pi, gsli0, gsli, i);
        end
        [M_b_GDCI5ff(:,:,cnt)] = CalculateOnlyBodyMassMatrix(JBsli,M0_CoM);
        M_b_GDCI5_12(cnt3,cnt2) = M_b_GDCI5ff(1,2,cnt);
        M_b_GDCI5_13(cnt3,cnt2) = M_b_GDCI5ff(1,3,cnt);
        M_b_GDCI5_23(cnt3,cnt2) = M_b_GDCI5ff(2,3,cnt);
    end
end
%% Plots
[t2f,t3f] = meshgrid(-2.4:0.08:2.4,-2.4:0.08:2.4);
%% Random vs GDCI1
figure; % M12
surf(t2f,t3f,M_b_GDCI4_12,'EdgeColor','flat','FaceColor','interp'); hold on; 
surf(t2f,t3f,M_b_RAND12,'EdgeColor','interp','FaceColor','texturemap','FaceAlpha',0.5); hold on;
title('M_{12} Reference vs GDCI4'); xlabel('θ_2'); ylabel('θ_3'); zlabel('M_{12}'); colorbar;
figure; % M13
surf(t2f,t3f,M_b_GDCI4_13,'EdgeColor','flat','FaceColor','interp'); hold on; 
surf(t2f,t3f,M_b_RAND13,'EdgeColor','interp','FaceColor','texturemap','FaceAlpha',0.5); hold on;
title('M_{13} Reference vs GDCI4'); xlabel('θ_2'); ylabel('θ_3'); zlabel('M_{13}'); colorbar;
figure; % M23
surf(t2f,t3f,M_b_GDCI4_23,'EdgeColor','flat','FaceColor','interp'); hold on; 
surf(t2f,t3f,M_b_RAND23,'EdgeColor','interp','FaceColor','texturemap','FaceAlpha',0.5); hold on;
title('M_{23} Reference vs GDCI4'); xlabel('θ_2'); ylabel('θ_3'); zlabel('M_{23}'); colorbar;
