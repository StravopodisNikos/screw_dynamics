function [M,M_POE,deltaM_thetak,C_POE] = manipulator_inertia_matrix_6DoF(xi_ai, exp_ai, Pi, Mi, Mi_s, Ji, theta_dot)
% Computes Manipulator Inertia Matrix based on eq.4.19 p.168 Murray

% Mi is generalized Inertia matrix of i-th link expressed in link reference frame - 6x6 x2 (number of links)
% Mi_s is generalized Inertia matrix of i-th link expressed in spatial frame - 6x6 x2 (number of links)

% Ji is Body Jacobian of CoM - 6x2 x2

n_Dof = size(Ji,2); % extract number of links

% Computes manipulator inertia matrix based on eq.4.19 p.168 Murray
M = zeros(n_Dof); % memory preallocation

for i = 1:n_Dof
    M = M + Ji(:,:,i)'*Mi(:,:,i)*Ji(:,:,i);
end

% Computes manipulator inertia matrix based on eq.4.29a p.176 Murray
tic
[M_POE,deltaM_thetak,C_POE] = compute_Mij_429_6DoF(xi_ai, exp_ai, Pi, Mi_s, theta_dot);
toc
end