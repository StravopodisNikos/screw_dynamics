function [Mij_429] = compute_Mij_429(xi_ai, exp_ai, Pi1, Mi_s)
% Computes Manipulator Inertia Matrix based on eq.4.29a p.176

% l = max(i,j);
n_Dof = size(xi_ai,2); % extract number of links
Mij_429 = zeros(2);
i = n_Dof;
j = n_Dof;
for c_i=1:i
    for c_j=1:j
        l = max(c_i,c_j);
        for add=l:n_Dof
            [Aij_1] = compute_Aij_for_Ji_427(exp_ai, Pi1, add, c_i);
            [Aij_2] = compute_Aij_for_Ji_427(exp_ai, Pi1, add, c_j);
            Mij_429(c_i,c_j) = Mij_429(c_i,c_j) + xi_ai(:,c_i)'*Aij_1'*Mi_s(:,:,add)'*Aij_2*xi_ai(:,c_j);
        end
        
    end
end

end
