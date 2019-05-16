function rstar = MinAccelerationRadius12(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM)
% Computes maximum acceleration(as max min radius) to be passed to the
% Objective function for Acceleration Radius GA solver
% called inside file: /modular_dynamixel/AccelerationRadiusGA.m

% Function to be maximized is the minimum radius of sphere inside
% acceleration ellipsoid rstar = max(r)
% s.t. Tj<=0 {linear contraints} j=1,2...2n

% Searches for optimal anatomy, that is gene x also tpi

% x Chromosome is:
% [q2 q3 qd2 qd3 tp1 tp2 tp3 tp4 tp5 tp6]
% q2.q3,qd2,qd3 -> double
% tp1...tp6 -> int
%% I.a. Define working region set {Q}
% Q set is the set of all (q,dq) investigated by genetic algorithm
% varargins is set Q={q,qdot}. limits are specified from ga call
% % global tpi1 xi_ai xi_pi Pi_test gst0 gsli0 M0b_CoM
% q = varargin{1}(1:6); % for 12 variables search
% qd= varargin{1}(7:12);
% q23 = varargin{1}(1:2); % for 4 variables search
% qd23= varargin{1}(3:4);

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
% Recompute pseudo exponentials
[~,Pi] = CalculatePseudoExponentials(xi_pi,tpi);

%% I.b. In this working region compute J,Jdot and dynamic matrices
% We calculate everything that is a function of q,qdot
%% Jacobians: J,Jdot
[J,metJbtest,~] = CalculateMetamorphicJacobians_6DoF(q',xi_ai,tpi,xi_pi,Pi,gst0);
% Jdot = dotJs5(qd',J);
% [Jbsli_test,Jssli_test,Jbsli_POE_test,gsli_test] = calculate_CoM_BodyJacobians_6DoF_MMD(q',xi_ai,Pi,gsli0);
%% Dynamic matrices: M,V,G - Only M here for ellipsoid calculation
[M,deltaM_thetak,C_POE] = compute_Mij_429_6DoF(xi_ai, exp_ai, Pi, gsli0, M0b_CoM, qd');
% [G] = calculate_gravity_matrix_6DoF(xi_ai, gsli0,[M0b_CoM(1,1,1) M0b_CoM(1,1,2) M0b_CoM(1,1,3) M0b_CoM(1,1,4) M0b_CoM(1,1,5) M0b_CoM(1,1,6)]',  Pi_test, q');
% V = C_POE*qd';
% CG = V+G;
%% II. Define joint actuators torque limits (for constraint function)
% trqhat = L^{-1}*trq & L = diag{trq_u1, trq_u2... trq_u6}
trq_u = [44.7 25.3 25.3 9.9 9.9 9.9]'; % MAX for Dynamixel motors to be used
% trq_l = -trq_u; % MIN torques
L = diag(trq_u);
%% Calculate radius of spheres inside hyper-acceleration ellipsoid:
% trq = M(q)*J^{-1}*(xdd - Jdot*qd) + V(q,qd) + G(q)  (IV)
% and trq'*trq <= 1 we get the acceleration ellipsoid!!!
Q = inv(J')*M'*inv(L')*inv(L)*M*inv(J); % as in paper "Evaluation of the dynamic performance...self-weight influence" PY Cheng - KJ Cheng
% Q = inv(J')*M*L*L*M*inv(J); % "Reformulation of dynamic manipulability ellipsoid"
sigmai = svd(Q); 
rmin = max(sigmai(1:3)); % minor axis is the max sigma_i
%% Since max(min(r)) is wanted
rstar = double(1/rmin);
end