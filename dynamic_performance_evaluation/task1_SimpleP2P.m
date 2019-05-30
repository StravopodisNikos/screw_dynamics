function [tvec,dy] = task1_SimpleP2P(robot,p,figures)
% p: 3x3 matrix with vectors [pi,pm,pf] the points of task

%% Build tfs for given points-ct orientation
gdi = eye(4); gdi(1:3,4) = p(:,1);
gdm = eye(4); gdm(1:3,4) = p(:,2);
gdf = eye(4); gdf(1:3,4) = p(:,3);
%% Solve IKP for task points in Cartesian Space
IKS = robotics.InverseKinematics('RigidBodyTree',robot); % create matlab solver for given robot
[qi,qiInfo] = IKS('TOOL',gdi,[0.1 0.1 0.1 0.1 0.1 0.1]',[0 0 0 0 0 0]');
[qm,qmInfo] = IKS('TOOL',gdm,[0.1 0.1 0.1 0.1 0.1 0.01]',[0.5 0.1 0.1 0 0 0]');
[qf,qfInfo] = IKS('TOOL',gdf,[0.1 0.1 0.1 0.1 0.1 0.1]',[0.1 0.1 0.2 0.1 0 0]');

%% Define Task Characteristics
% Robot STOPS at each task point
dqi = [0 0 0 0 0 0]'; % define the joint velocity when achieving gdi. 
ddqi = [0 0 0 0 0 0]'; % define the joint acceleration when achieving gdi. 

dqm = [0 0 0 0 0 0]'; 
ddqm = [0 0 0 0 0 0]';

dqf = [0 0 0 0 0 0]';  
ddqf = [0 0 0 0 0 0]';

%% Execute P2P motion
%% pi --> pm
traj_part = 1;
[tvec_im,dy_im] = control_6DoF_Metamorphic_Dynamixel(robot,qi,qm,dqm,ddqm,traj_part);
% DCI_im = DCI;
% Torques_im = torque; % save torque data
%% pm --> pf
traj_part = 2;
[tvec_mf,dy_mf] = control_6DoF_Metamorphic_Dynamixel(robot,qm,qf,dqf,ddqf,traj_part);
tvec_mf = tvec_im(length(tvec_im))+tvec_mf; % add previous time
% DCI_mf = DCI;
% Torques_mf = torque; % save torque data

%% merge data
tvec = vertcat(tvec_im,tvec_mf);
dy = vertcat(dy_im,dy_mf);

%% Record video
VideoFigure = figure('Name','Video record of task execution');
[video1] = RecordTaskExecutionVideo(VideoFigure,robot,dy(:,1:6)',p);
end