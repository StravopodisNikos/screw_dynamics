function r = GlobalAccelerationRadius(q,qd)
% Function to be minimized

%% I.a. Define working region set {Q}
% Q set is the set of all (q,dq) investigated by genetic algorithm
%varargins is set Q={q,qdot}
% q = varargin{1}(1:6);
% qd= varargin{1}(7:12);

%% I.b. In this working region compute J,Jdot and dynamic matrices
%
%% J,Jdot
% .... next ....
J = rand(6,6);
Jdot = rand(6,6);
Jbsli = rand(6,6,6);
%% from this section I get (M,V,G)
M0 = rand(6,6,6);
M = CalculateOnlyBodyMassMatrix(Jbsli,M0);
V = zeros([6 1]);
G = zeros([6 1]);
%% II. Define joint actuators torque limits (for constraint function)
% trqhat = L^{-1}*trq & L = diag{trq_u1, trq_u2... trq_u6}
trq_u = [44.7 25.3 25.3 9.9 9.9 9.9]'; % MAX for Dynamixel motors to be used
trq_l = -trq_u; % MIN torques
L = diag(trq_u);

%% III. Calculate qdd from robot dynamics for given torque limits
% Robot kinetic equation: trq = M(q)qdd + V(q,dq) + G(q) (I)
% solving for qdd: qdd_max = M^{-1}*(trq - V(q,qd) - G(q))|trq_u (IIa)
qdd_max = inv(M)*(trq_u-V-G);
% qdd_min = inv(M)*(trq_l-V-G);

%% IV. Calculate xdd from acceleration transform equation for qdd_max
% xdd = Jdot*qd + J*qdd (IIIa)
%  and from (IIa): xdd = Jdot*qd + J*( M^{-1}*(trq - V(q,qd) - G(q))) (IIb)
xdd_max = @(qdd_max) Jdot*qd + J*qdd_max;
r = xdd_max;
% xdd_min = @(qdd_min) Jdot*qd + J*qdd_min;

%% If I want acceleration ellipsoid:
% from (I) we get: trq = M(q)*J^{-1}*(xdd - Jdot*qd) + V(q,qd) + G(q)  (IV)
% eq. (IV) is my constraint function defined in file:
% /modular_dynamixel/torq_constraints.m
% From (IV) and trq'*trq <= 1 we get the acceleration ellipsoid!!!

%% V. Now (q,qd) & xdd|torque_limits are known
% Now i want to find maneuverability set MS(Q)
% MS(Q) = {xdd|trq_l<=trq_i && trq_i <=trq_u}
% r = xdd_max;
rstar0 = [100 100 100 100 100 100]';
ntheta = 12;
s(:,1) = ones([12 1]);
s(:,2) = zeros([12 1]);
[rstar,fval,exitflag,output,lambda] = fseminf(r,rstar0,ntheta,@(rstar)torq_constraints(rstar,s,q,M));
%% Since max r is wanted
r = 1/rstar;
end