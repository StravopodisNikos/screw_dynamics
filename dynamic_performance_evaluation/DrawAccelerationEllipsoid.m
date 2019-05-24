function [rmin] = DrawAccelerationEllipsoid(tpi1, xi_ai, xi_pi, Pi_test, gst0, gsli0, M0b_CoM)
% Current Joint position and velocity
q = [1 1 1 0 0 0]';
qd = [0.5 0.5 0.5 0 0 0]';

% Recompute active exponentials
exp_ai(:,:,1) = twistexp(xi_ai(:,1), q(1));
exp_ai(:,:,2) = twistexp(xi_ai(:,2), q(2));
exp_ai(:,:,3) = twistexp(xi_ai(:,3), q(3));
exp_ai(:,:,4) = twistexp(xi_ai(:,4), q(4));
exp_ai(:,:,5) = twistexp(xi_ai(:,5), q(5));
exp_ai(:,:,6) = twistexp(xi_ai(:,6), q(6));

[J,metJbtest,~] = CalculateMetamorphicJacobians_6DoF(q',xi_ai,tpi1,xi_pi,Pi_test,gst0);
[M,deltaM_thetak,C_POE] = compute_Mij_429_6DoF(xi_ai, exp_ai, Pi_test, gsli0, M0b_CoM, qd');
C = C_POE*qd;
[G] = calculate_gravity_matrix_6DoF(xi_ai, gsli0,[M0b_CoM(1,1,1) M0b_CoM(1,1,2) M0b_CoM(1,1,3) M0b_CoM(1,1,4) M0b_CoM(1,1,5) M0b_CoM(1,1,6)]',  Pi_test, q');

trq_u = [44.7 25.3 25.3 9.9 9.9 9.9]'; % MAX for Dynamixel motors to be used
norm_trq_u = norm(trq_u);
% trq_l = -trq_u; % MIN torques
L = diag((trq_u));

dJ = dotJs5(qd,J);
xdot = J*qd;
A = inv(L)*M*inv(J);
b = inv(L)*((C+G)-M*inv(J)*dJ*inv(J)*qd);
c = - inv(A)*b; % -A^-1*b
Q = inv(J')*M'*inv(L')*inv(L)*M*inv(J); % as in paper "Evaluation of the dynamic performance...self-weight influence" PY Cheng - KJ Cheng
[rmin] = minAccelEllipsoidRadius(c,Q);
end