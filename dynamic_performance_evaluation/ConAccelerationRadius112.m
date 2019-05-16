function [c,ceq] = ConAccelerationRadius112(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM)
% Nonlinear constraint function for optimal anatomy determination
% based on acceleration radius
% Constarint function for Acceleration Radius GA solver
% called inside file: /modular_dynamixel/AccelerationRadiusGA.m

% global xi_ai xi_pi gst0 gsli0 M0b_CoM % computed in master
% q = x(1:6); % for 12 variables search
% qd= x(7:12);
q23 = x(1:2); % for 4 variables search
qd23= x(3:4);
ci = x(5:10);
%% Determine tpi pseudojoint angles
step_angle = deg2rad(45);
tpi0 = [-pi/2 -pi/2 -pi/2 -pi/2 -pi/2 -pi/2]';
% m = pi/step_angle + 1; % number of pseudojoint variables set M=[i] 1<i<m 
for k=1:6
    tpi(k) = step_angle * (ci(k) - 1) + tpi0(k);
end
%% I.a. Definition of working region for anatomy selected
q = [0 q23(1) q23(2) 0 0 0];
qd = [0 qd23(1) qd23(2) 0 0 0];
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
% We calculate everything that is a function of q,qdot
%% Jacobians: J,Jdot
[J,metJbtest,metErrortest] = CalculateMetamorphicJacobians_6DoF(q',xi_ai,tpi,xi_pi,Pi,gst0);
Jdot = dotJs5(qd',J);
% [Jbsli_test,Jssli_test,Jbsli_POE_test,gsli_test] = calculate_CoM_BodyJacobians_6DoF_MMD(q',xi_ai,Pi_test,gsli0);

%% Dynamic matrices: M,V,G
% M = CalculateOnlyBodyMassMatrix(Jbsli_test,M0b_CoM);
[M,deltaM_thetak,C_POE] = compute_Mij_429_6DoF(xi_ai, exp_ai, Pi, gsli0, M0b_CoM, qd');
[G] = calculate_gravity_matrix_6DoF(xi_ai, gsli0,[M0b_CoM(1,1,1) M0b_CoM(1,1,2) M0b_CoM(1,1,3) M0b_CoM(1,1,4) M0b_CoM(1,1,5) M0b_CoM(1,1,6)]',  Pi, q');
V = C_POE*qd';
CG = V+G;
%% Torques of Dynamixel motors
trq_u = [44.7 25.3 25.3 9.9 9.9 9.9]'; % MAX for Dynamixel motors to be used
trq_l = -trq_u; % MIN torques
L = diag(trq_u);
%% Acceleration ellipsoid
Q = inv(J')*M'*inv(L')*inv(L)*M*inv(J); % as in paper "Evaluation of the dynamic performance...self-weight influence" PY Cheng - KJ Cheng
r = eig(Q);
%% Nonlinear constraint equations
A = M*inv(J);
B = CG - M*inv(J)*Jdot*qd';
A1 = A;
A2 = -A;
b1 = B - trq_u;
b2 = trq_l - B;

c = double([A1*r+b1; A2*r+b2]);
ceq =double([]);
end