function [c,ceq] = ConAccelerationRadius11(x)
global tpi1 xi_ai xi_pi Pi_test gst0 gsli0 M0b_CoM % computed in master
% q = x(1:6); % for 12 variables search
% qd= x(7:12);
q23 = x(1:2); % for 4 variables search
qd23= x(3:4);
%% I.a. Definition of working region
q = [0 q23(1) q23(2) 0 0 0];
qd = [0 qd23(1) qd23(2) 0 0 0];
% Recompute active exponentials
exp_ai(:,:,1) = twistexp(xi_ai(:,1), q(1));
exp_ai(:,:,2) = twistexp(xi_ai(:,2), q(2));
exp_ai(:,:,3) = twistexp(xi_ai(:,3), q(3));
exp_ai(:,:,4) = twistexp(xi_ai(:,4), q(4));
exp_ai(:,:,5) = twistexp(xi_ai(:,5), q(5));
exp_ai(:,:,6) = twistexp(xi_ai(:,6), q(6));

%% I.b. In this working region compute J,Jdot and dynamic matrices
% We calculate everything that is a function of q,qdot
%% Jacobians: J,Jdot
[J,metJbtest,metErrortest] = CalculateMetamorphicJacobians_6DoF(q',xi_ai,tpi1,xi_pi,Pi_test,gst0);
Jdot = dotJs5(qd',J);
[Jbsli_test,Jssli_test,Jbsli_POE_test,gsli_test] = calculate_CoM_BodyJacobians_6DoF_MMD(q',xi_ai,Pi_test,gsli0);

%% Dynamic matrices: M,V,G
% M = CalculateOnlyBodyMassMatrix(Jbsli_test,M0b_CoM);
[M,deltaM_thetak,C_POE] = compute_Mij_429_6DoF(xi_ai, exp_ai, Pi_test, gsli0, M0b_CoM, qd');
[G] = calculate_gravity_matrix_6DoF(xi_ai, gsli0,[M0b_CoM(1,1,1) M0b_CoM(1,1,2) M0b_CoM(1,1,3) M0b_CoM(1,1,4) M0b_CoM(1,1,5) M0b_CoM(1,1,6)]',  Pi_test, q');
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

c = [A1*r+b1; A2*r+b2];
ceq =[];
end