function Wd_star = MaxWd(x,xi_ai,xi_pi,gst0,gsli0,M0b_CoM)
% Computes DCI to be passed to the
% Objective function for Acceleration Radius GA solver
% called inside file: /modular_dynamixel/AccelerationRadiusGA.m

% DCI closer to zero is an indicator of better dynamic isotropy of
% manipulator

% x Chromosome is:
% [q2 q3 tp1 tp2 tp3 tp4 tp5 tp6]
% q2,q3 -> double
% tp1...tp6 -> int

%% Add geometry libraries
addpath('/home/nikos/matlab_ws/geom3d/geom3d')
addpath('/home/nikos/matlab_ws/geom2d/utils')
addpath('/home/nikos/matlab_ws/geom3d/meshes3d')

%% Get chromosome values
q23 = x(1:2); % for 4 variables search
ci = x(3:8);
%% Determine tpi pseudojoint angles
step_angle = deg2rad(45);
tpi0 = [-pi/2 -pi/2 -pi/2 -pi/2 -pi/2 -pi/2]';
% m = pi/step_angle + 1; % number of pseudojoint variables set M=[i] 1<i<m 
for k=1:6
    tpi(k) = step_angle * (ci(k) - 1) + tpi0(k);
end

%% I.a. Definition of working region
% Searches in the Configuration Space of first 3DoF for all anatomies
% Calculates Dynamic Conditioning Index in each point

q = [0 q23(1) q23(2) 0 0 0];
% Recompute active exponentials
exp_ai(:,:,1) = twistexp(xi_ai(:,1), q(1));
exp_ai(:,:,2) = twistexp(xi_ai(:,2), q(2));
exp_ai(:,:,3) = twistexp(xi_ai(:,3), q(3));
exp_ai(:,:,4) = twistexp(xi_ai(:,4), q(4));
exp_ai(:,:,5) = twistexp(xi_ai(:,5), q(5));
exp_ai(:,:,6) = twistexp(xi_ai(:,6), q(6));
% Recompute pseudo exponentials
[~,Pi] = CalculatePseudoExponentials(xi_pi,tpi);

%% I.b. In this working region compute J,Jdot and dynamic matrices
% Jacobians: J,Jdot
[Js,~,~] = CalculateMetamorphicJacobians_6DoF(q',xi_ai,tpi,xi_pi,Pi,gst0);
% Compute CoM Jacobian
[Jbsli,~,~,~] = CalculateCoMBodyJacobians_6DoF(q,xi_ai,Pi,gsli0);
%% Dynamic matrices: M,V,G - Only M here for ellipsoid calculation
[Mb] = CalculateOnlyBodyMassMatrix(Jbsli,M0b_CoM);
% Calculate Dynamic Performance Measures values
Wd = calculateDynamicManipulabilityIndex(Js,Mb);
Wd_star = 1/Wd; 
end