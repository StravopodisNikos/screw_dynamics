function [RefRobot,RefFig,RefConfig,NumDoF] = ImportRobotRefAnatomyModel(robotname)
% robotname MUST BE a MATLAB robotics.RigidBodyTree Model

%% Import only robot in reference anatomy URDF
[RefRobot] = importrobot(robotname); 
disp('Robot model loaded in Reference Anatomy:');
fprintf(robotname,'\n');% Print in cmd line the name of the robot file

RefFig = figure;
show(RefRobot);
axis auto;
hold on;
RefRobot.DataFormat = 'column';
RefRobot.Gravity = [0 0 -9.80665];
showdetails(RefRobot);
RefConfig = homeConfiguration(RefRobot);
NumDoF = size(RefConfig,1);

end