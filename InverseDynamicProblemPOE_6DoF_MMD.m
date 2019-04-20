function [M_POE,V_POE,N_POE,Mm,Vm,Nm,JointTorque_POE,JointTorqM] = InverseDynamicProblemPOE_6DoF_MMD(robot,config,theta_dot,theta_dotdot,Mi,Mi_s,Jis,Jib,xi_ai, Pi, gsli0)
% Calculates dynamic parameters using my_POE's and Matlab defaults

%% Recalculate active exponentials for given configuration
exp_ai(:,:,1) = twistexp(xi_ai(:,1), config(1));
exp_ai(:,:,2) = twistexp(xi_ai(:,2), config(2));
exp_ai(:,:,3) = twistexp(xi_ai(:,3), config(3));
exp_ai(:,:,4) = twistexp(xi_ai(:,4), config(4));
exp_ai(:,:,5) = twistexp(xi_ai(:,5), config(5));
exp_ai(:,:,6) = twistexp(xi_ai(:,6), config(6));

%% my_POE's
[Ms, Mb, M_POE, dM_POE, C_POE] = manipulator_inertia_matrix_6DoF(xi_ai, exp_ai, Pi, gsli0, Mi, Mi_s, Jis, Jib, theta_dot);
V_POE = C_POE*theta_dot;
[N_POE] = calculate_gravity_matrix_6DoF(xi_ai, gsli0,[Mi(1,1,1) Mi(1,1,2) Mi(1,1,3) Mi(1,1,4) Mi(1,1,5) Mi(1,1,6)]',  Pi, config);
%% MATLAB defaults
Mm = massMatrix(robot,config);
Vm = velocityProduct(robot,config,theta_dot);
Nm = gravityTorque(robot,config);

%% IDP equation
JointTorque_POE = M_POE*theta_dotdot + C_POE*theta_dot + N_POE;
JointTorqM = inverseDynamics(robot,config,theta_dot,theta_dotdot);
IDP_error = JointTorqM - JointTorque_POE; % Matlab - My_POE_Way

end