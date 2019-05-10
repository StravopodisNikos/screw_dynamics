function [c,ceq,K1,K2] = torq_constraints(r,s,q,M)
% is called from fesminf inside file:
% /modular_dynamixel/GlobalAccelerationRadius.m
% qd -> w1 
trq_u = [44.7 25.3 25.3 9.9 9.9 9.9]'; % MAX for Dynamixel motors to be used
trq_l = -trq_u; % MIN torques

c = [];
ceq =[];
V = rand(6,6);
G = rand(6,1);

% trq_i = M*inv(J)*(r - Jdot*qd) + V + G;
trq_i = ones([6 1]);

K1 = trq_i - trq_u;
K2 = trq_l - trq_i;
end