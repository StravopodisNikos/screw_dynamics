function [c,ceq] = ConGlobalMaxWd(x,xi_ai,xi_pi,gst0,gsli0,M0b_CoM)
% Constarint function for GlobalMaxWd
% When Wd->min it constraints that det(Jb) > singular_limit
% Called inside Acceleration Radius GA solver
% called inside file: /modular_dynamixel/AccelerationRadiusGA.m

% x Chromosome is:
% [tp1 tp2 tp3 tp4 tp5 tp6]
% tp1...tp6 -> int

%% Add geometry libraries
addpath('/home/nikos/matlab_ws/geom3d/geom3d')
addpath('/home/nikos/matlab_ws/geom2d/utils')
addpath('/home/nikos/matlab_ws/geom3d/meshes3d')

%% Get chromosome values
ci = x(1:6);
%% Determine tpi pseudojoint angles
step_angle = deg2rad(45);
tpi0 = [-pi/2 -pi/2 -pi/2 -pi/2 -pi/2 -pi/2]';
% m = pi/step_angle + 1; % number of pseudojoint variables set M=[i] 1<i<m 
for k=1:6
    tpi(k) = step_angle * (ci(k) - 1) + tpi0(k);
end
% Recompute pseudo exponentials
[~,Pi] = CalculatePseudoExponentials(xi_pi,tpi);

%% I.a. Definition of working region
% Searches in the Configuration Space of first 3DoF for all anatomies
p_count = 0;
step_angle = 20;
q(1) = 0;
minWd = 5.0e+20;
for ta2=-1.5708:deg2rad(step_angle):1.5708
    for ta3=-1.5708:deg2rad(step_angle):1.5708
        q = [0 ta2 ta3 0 0 0];
        p_count = p_count + 1;
        % Recompute active exponentials
        exp_ai(:,:,1) = twistexp(xi_ai(:,1), q(1));
        exp_ai(:,:,2) = twistexp(xi_ai(:,2), q(2));
        exp_ai(:,:,3) = twistexp(xi_ai(:,3), q(3));
        exp_ai(:,:,4) = twistexp(xi_ai(:,4), q(4));
        exp_ai(:,:,5) = twistexp(xi_ai(:,5), q(5));
        exp_ai(:,:,6) = twistexp(xi_ai(:,6), q(6));
        %% I.b. In this working region compute J,Jdot and dynamic matrices
        % Jacobians: J,Jdot
        [~,Jb,~] = CalculateMetamorphicJacobians_6DoF(q',xi_ai,tpi,xi_pi,Pi,gst0);
%         det(Jb)
        % Compute CoM Jacobian
        [Jbsli,~,~,~] = CalculateCoMBodyJacobians_6DoF(q',xi_ai,Pi,gsli0);
        %% Dynamic matrices: M,V,G - Only M here for ellipsoid calculation
        [Mb] = CalculateOnlyBodyMassMatrix(Jbsli,M0b_CoM);
        % Calculate Dynamic Performance Measures values
        Wd(p_count) = calculateDynamicManipulabilityIndex(Jb,Mb);
        if Wd(p_count)<minWd
            minWd = Wd(p_count);
            minJb = Jb;
        end
    end
end
e1 = 1.0e-12;

c1 = e1 - det(minJb);

c = c1;
ceq = [];
end