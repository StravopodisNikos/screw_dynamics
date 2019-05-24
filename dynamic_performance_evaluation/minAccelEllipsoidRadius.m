function [rmin] = minAccelEllipsoidRadius(c,QORE)
% From theory we know that the equation of an ellipsoid in 3D space is
% given fro,: (x-c)^T * Q * (x-c) = 1
% 1.Given 3D center translation vector c, semi-axes values in a [3 1]
% vector to descending order and the orientation of the ellipsoid principal axes
% we draw the end-effector acceleration space ellipsoid

% 2. We draw a sphere from reference frame center and we want to see the
% intersection between the  ellipsoi and the sphere. This is an
% approximation of the min distance from surface ellipsoid points to the
% center of the reference frame

% load Geometry 2-3D libraries
addpath('/home/nikos/matlab_ws/geom3d/geom3d')
addpath('/home/nikos/matlab_ws/geom2d/geom2d')
addpath('/home/nikos/matlab_ws/geom2d/utils')

[U,S,V] = svd(QORE); % units are (m/s^2)^-1
sigma = [1/S(3,3)  1/S(2,2) 1/S(1,1)]; % so I take the inverse singular values
f0 = c;
f1 = QORE(1:3,1);
f2 = QORE(1:3,2);
f3 = QORE(1:3,3);
orientEulerAngles = rotm2eul(QORE(1:3,1:3)); % zyx euler angles

AccelEllipsoid = [f0(1:3)' sigma rad2deg(orientEulerAngles)];
AccelEllipsoid0 = [[0 0 0] sigma rad2deg(orientEulerAngles)]; % no C,G

figure; hold on;
drawEllipsoid(AccelEllipsoid,'FaceColor','r','facealpha', '0.5','drawEllipses', true, 'EllipseColor', 'r', 'EllipseWidth', 1);
hold on;
drawEllipsoid(AccelEllipsoid0,'FaceColor','b','facealpha', '0.5','drawEllipses', true, 'EllipseColor', 'b', 'EllipseWidth', 1);
hold on;
axis equal;
rmin = abs(norm(f0(1:3)')-sigma(3)); 
end