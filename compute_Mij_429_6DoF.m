function [Mij_429,delta_Mij_thetak_429,Cij_429a] = compute_Mij_429_6DoF(xi_ai, exp_ai, Pi, Mi_s, theta_dot)
% Computes Manipulator Inertia Matrix based on eq.4.29a p.176

% l = max(i,j);
n_Dof = size(xi_ai,2); % extract number of links
Mij_429 = zeros(n_Dof);
Cij_429 = zeros(n_Dof);
Cij_429a = zeros(n_Dof);
dM2C = zeros(n_Dof);

i = n_Dof;
j = n_Dof;
for c_i=1:i
    for c_j=1:j
        l = max(c_i,c_j);
        for add=l:n_Dof
            [Aij_1] = compute_Aij_for_Ji_427_6DoF(exp_ai, Pi, add, c_i);
            [Aij_2] = compute_Aij_for_Ji_427_6DoF(exp_ai, Pi, add, c_j);
            Mij_429(c_i,c_j) = Mij_429(c_i,c_j) + xi_ai(:,c_i)'*Aij_1'*Mi_s(:,:,add)'*Aij_2*xi_ai(:,c_j);
        end
        
        % Compute  δ(Mij)/δ(θk) for all θk for each Mij
        delta_Mij_thetak_429 = zeros(n_Dof);
        delta_Mik_thetaj_429 = zeros(n_Dof);
        delta_Mkj_thetai_429 = zeros(n_Dof);
        for k=1:n_Dof
            [delta_Mij_thetak_429(:,:,k)] = compute_delta_Mij_thetak_429_6DoF(xi_ai, exp_ai, Pi, Mi_s, k);
            if k == c_j
                [delta_Mik_thetaj_429] = compute_delta_Mij_thetak_429_6DoF(xi_ai, exp_ai, Pi, Mi_s, k);
            end
            if k == c_i
                [delta_Mkj_thetai_429] = compute_delta_Mij_thetak_429_6DoF(xi_ai, exp_ai, Pi, Mi_s, k);
            end
            Cij_429(c_i,c_j) = Cij_429(c_i,c_j) + 0.5*(delta_Mij_thetak_429(c_i,c_j)+delta_Mik_thetaj_429(c_i,c_j)-delta_Mkj_thetai_429(c_i,c_j))*theta_dot(k);
        end
    end
end

% Since all δ(Mij)/δ(θk) are computed we build Cij
for c_i=1:i
    for c_j=1:j
        for k=1:n_Dof
            Cij_429a(c_i,c_j) = Cij_429a(c_i,c_j) + 0.5 * ( delta_Mij_thetak_429(c_i,c_j,k) +  delta_Mij_thetak_429(c_i,k,c_j) - delta_Mij_thetak_429(k,c_j,c_i) ) * theta_dot(k);
        end
    end
end

% we check for skew symmetric Mdot-2C
for c_i=1:i
    for c_j=1:j
        for k=1:n_Dof
            dM2C(c_i,c_j) = dM2C(c_i,c_j) + delta_Mij_thetak_429(c_i,k,c_j) - delta_Mij_thetak_429(k,c_j,c_i);
        end
    end
end
check = isskew(dM2C);

end