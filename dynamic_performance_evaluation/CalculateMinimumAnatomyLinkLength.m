function [minLm,maxVm] = CalculateMinimumAnatomyLinkLength(tpi)
% Input: variables of pseudojoint angles of current anatomy as 6x1 vector
% Output: Minimum Sum of Link lengths and maximum workspace volume
addpath('/home/nikos/matlab_ws/geom3d/geom3d')
addpath('/home/nikos/matlab_ws/geom2d/utils')
global xi_ai xi_pi gst0 

%% Calculate pseudo exponentials
exp_pi(:,:,1) = twistexp(xi_pi(:,1), tpi(1));
exp_pi(:,:,2) = twistexp(xi_pi(:,2), tpi(2));
exp_pi(:,:,3) = twistexp(xi_pi(:,3), tpi(3));
exp_pi(:,:,4) = twistexp(xi_pi(:,4), tpi(4));
exp_pi(:,:,5) = twistexp(xi_pi(:,5), tpi(5));
exp_pi(:,:,6) = twistexp(xi_pi(:,6), tpi(6));
Pi(:,:,1) = exp_pi(:,:,1)*exp_pi(:,:,2);
Pi(:,:,2) = exp_pi(:,:,3)*exp_pi(:,:,4);
Pi(:,:,3) = exp_pi(:,:,5)*exp_pi(:,:,6);
%% Calculate active exponentilas for zero config
ta0 = zeros([6 1]);
exp_a0(:,:,1) = twistexp(xi_pi(:,1), ta0(1));
exp_a0(:,:,2) = twistexp(xi_pi(:,2), ta0(2));
exp_a0(:,:,3) = twistexp(xi_pi(:,3), ta0(3));
exp_a0(:,:,4) = twistexp(xi_pi(:,4), ta0(4));
exp_a0(:,:,5) = twistexp(xi_pi(:,5), ta0(5));
exp_a0(:,:,6) = twistexp(xi_pi(:,6), ta0(6));
gst = exp_a0(:,:,1)*Pi(:,:,1)*exp_a0(:,:,2)*Pi(:,:,2)*exp_a0(:,:,3)*Pi(:,:,3)*exp_a0(:,:,4)*exp_a0(:,:,5)*exp_a0(:,:,6)*gst0;
%% Calculate zero config Spatial Jacobian of current anatomy
[anatJs, anatJb, Error] = metamorphic_jacobians_6dof(xi_ai,exp_a0, Pi, gst0, gst);
%% Columns of anatJs are the active twists of new anatomy
% Twists->Twist axes,points,lines
% axis vectors
% axis_a0(:,1) = twistaxis(anatJs(:,1));
% axis_a0(:,2) = twistaxis(anatJs(:,2));
% axis_a0(:,3) = twistaxis(anatJs(:,3));
% axis_a0(:,4) = twistaxis(anatJs(:,4));
% axis_a0(:,5) = twistaxis(anatJs(:,5));
% axis_a0(:,6) = twistaxis(anatJs(:,6));
% points on axis: pi1(:,i) - axis_a0(:,i) - pi11(:,i)
[pi1(:,1),pi11(:,1)] = Return2PointsOfTwist(anatJs(:,1));
[pi1(:,2),pi11(:,2)] = Return2PointsOfTwist(anatJs(:,2));
[pi1(:,3),pi11(:,3)] = Return2PointsOfTwist(anatJs(:,3));
[pi1(:,4),pi11(:,4)] = Return2PointsOfTwist(anatJs(:,4));
[pi1(:,5),pi11(:,5)] = Return2PointsOfTwist(anatJs(:,5));
[pi1(:,6),pi11(:,6)] = Return2PointsOfTwist(anatJs(:,6));
% create lines
L(1,:) = createLine3d(pi1(:,1)', pi11(:,1)'); %drawLine3d(L); hold on
L(2,:) = createLine3d(pi1(:,2)', pi11(:,2)');
L(3,:) = createLine3d(pi1(:,3)', pi11(:,3)');
L(4,:) = createLine3d(pi1(:,4)', pi11(:,4)');
L(5,:) = createLine3d(pi1(:,1)', pi11(:,5)');
L(6,:) = createLine3d(pi1(:,6)', pi11(:,6)');
% min distances
[md2, PT12_1, PT12_2] = distanceLines3d(L(1,:), L(2,:));
[md3, PT23_2, PT23_3] = distanceLines3d(L(2,:), L(3,:));
[md4, PT34_3, PT34_4] = distanceLines3d(L(3,:), L(4,:));
[md5, PT45_4, PT45_5] = distanceLines3d(L(4,:), L(5,:));
[md6, PT56_5, PT56_6] = distanceLines3d(L(5,:), L(6,:));

minLm = md2+md3+md4+md5+md6;

end