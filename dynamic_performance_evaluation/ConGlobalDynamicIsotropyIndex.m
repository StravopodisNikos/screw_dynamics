function [c,ceq] = ConGlobalDynamicIsotropyIndex(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM)
% Computes Global Dynamic Isotropy Index
% Is called inside file:

% Function to be minimized using ga is F = 1 - GDII, GDII = σmin/σmax are
% singular values of acceleration ellipsoid core

% Singularities avoidance is added as nonlinear constraints:
% e1 - det(CORE) <= 0 (1)
% e2 - det(Jb) <= 0 (2)

%% I.a. Define ga state vector
q23 = x(1:2); % for 4 variables search
ci = x(3:8);
%% Determine tpi pseudojoint angles
step_angle = deg2rad(45);
tpi0 = [-pi/2 -pi/2 -pi/2 -pi/2 -pi/2 -pi/2]';
% m = pi/step_angle + 1; % number of pseudojoint variables set M=[i] 1<i<m 
for k=1:6
    tpi(k) = step_angle * (ci(k) - 1) + tpi0(k);
end

%% I.b. Set configuration
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
%% Jacobians: Js, Jb and Mass Matrix
[~,Jb,~] = CalculateMetamorphicJacobians_6DoF(q',xi_ai,tpi,xi_pi,Pi,gst0);
[Jbsli_test,~,Jbsli_POE_test,~] = calculate_CoM_BodyJacobians_6DoF_MMD(q',xi_ai,Pi,gsli0);
M = CalculateOnlyBodyMassMatrix(Jbsli_test,M0b_CoM);

%% Build Ellipsoid Core
trq_u = [44.7 25.3 25.3 9.9 9.9 9.9]'; % MAX for Dynamixel motors to be used
L = diag(trq_u);
Q = inv(Jb')*M'*inv(L')*inv(L)*M*inv(Jb); % as in paper "Evaluation of the dynamic performance...self-weight influence" PY Cheng - KJ Cheng

%% Set distance from singularity
e1 = 0.000001;
e2 = 1.0e-12;
c1 = e1 - det(Q);
c2 = e2 - det(Jb);

c = [c1; c2];
ceq = [];
end