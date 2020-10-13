function [M_b, M_POE,deltaM_thetak,C_POE] = manipulator_inertia_matrix(xi_ai, exp_ai, Pi1, gsli0, Mi_b, Jib, theta_dot)
% Computes Manipulator Inertia Matrix based on eq.4.19 p.168 Murray

% Mi is generalized Inertia matrix of i-th link expressed in link reference frame - 6x6 x2 (number of links)
% Mi_s is generalized Inertia matrix of i-th link expressed in spatial frame - 6x6 x2 (number of links)

% Ji is Body Jacobian of CoM - 6x2 x2

n_Dof = size(Jib,2); % extract number of links

% Computes manipulator inertia matrix based on eq.4.19 p.168 Murray
% M_s = zeros(n_Dof); % memory preallocation
M_b = zeros(n_Dof); % memory preallocation

% for i = 1:n_Dof
%     M_s = M_s + Jis(:,:,i)'*Mi_s(:,:,i)*Jis(:,:,i);
% end

for i = 1:n_Dof
    M_b = M_b + Jib(:,:,i)'*Mi_b(:,:,i)*Jib(:,:,i);
end

% Computes manipulator inertia matrix based on eq.4.29a p.176 Murray
[M_POE,deltaM_thetak,C_POE] = compute_Mij_429(xi_ai, exp_ai, Pi1, gsli0, Mi_b, theta_dot);

end