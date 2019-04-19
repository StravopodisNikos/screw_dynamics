function [TestRobot,TestFig,TestConfig,NumDoF] = ImportRobotTestAnatomyModel(robotname)
% robotname MUST BE a MATLAB robotics.RigidBodyTree Model

%% Import only robot in reference anatomy URDF
[TestRobot] = importrobot(robotname); 
disp('Robot model loaded in Test Anatomy:');
fprintf(robotname,'\n');% Print in cmd line the name of the robot file

TestFig = figure;
show(TestRobot);
axis auto;
% hold on;
TestRobot.DataFormat = 'column';
TestRobot.Gravity = [0 0 -9.80665];
showdetails(TestRobot);
TestConfig = homeConfiguration(TestRobot);
NumDoF = size(TestConfig,1);

end