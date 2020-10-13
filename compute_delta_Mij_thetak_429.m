function [delta_Mij_thetak_429] = compute_delta_Mij_thetak_429(xi_ai, exp_ai, Pi1, gsli0, Mi_b, k)
% Computes Manipulator Inertia Matrix based on eq.4.29a p.176

% l = max(i,j);
n_Dof = size(xi_ai,2); % extract number of links

delta_Mij_thetak_429 = zeros(2);


% delta_Mij_thetak_429 = sym(zeros(2));

i = n_Dof;
j = n_Dof;
for c_i=1:i
    for c_j=1:j
        l = max(c_i,c_j);
        for add=l:n_Dof
            [Aki] = compute_Aij_for_Ji_427(exp_ai, Pi1, k, c_i);
            [Alk] = compute_Aij_for_Ji_427(exp_ai, Pi1, add, k);
            [Alj] = compute_Aij_for_Ji_427(exp_ai, Pi1, add, c_j);
            Lie_Br_1 = liebracket_426_new(Aki*xi_ai(:,c_i), xi_ai(:,k));
            
            [Ali] = compute_Aij_for_Ji_427(exp_ai, Pi1, add, c_i);
            [Akj] = compute_Aij_for_Ji_427(exp_ai, Pi1, k, c_j);
            Lie_Br_2 = liebracket_426_new(Akj*xi_ai(:,c_j), xi_ai(:,k));
            
            Ml = ad(inv(gsli0(:,:,add)))'*Mi_b(:,:,add)*ad(inv(gsli0(:,:,add)));
            
            delta_Mij_thetak_429(c_i,c_j) = delta_Mij_thetak_429(c_i,c_j) + (Lie_Br_1'*Alk'*Ml*Alj*xi_ai(:,c_j) + xi_ai(:,c_i)'*Ali'*Ml*Alk*Lie_Br_2);
        end
        
    end
end

end