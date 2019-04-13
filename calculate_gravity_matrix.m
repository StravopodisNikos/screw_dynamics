function [N] = calculate_gravity_matrix(xi_ai, gsli0, mi,  Pi1, theta)

n_Dof = size(xi_ai,2); % extract number of links
g = -9.8067; % [m/s^2]
V = sym('V',[2 1]);

t = sym('t', [1 2]);
% Calculates symbolic exponentials of twist matrices 
for i = 1:n_Dof
    sexp(:,:,i)=twistexp(xi_ai(:,i), t(i));
    % Calculates symbolic FKP
    if i ==1
        sgsli0(:,:,i) = sexp(:,:,i)*gsli0(:,:,i);
    elseif i==2
        sgsli0(:,:,i) = sexp(:,:,i-1)*Pi1*sexp(:,:,i)*gsli0(:,:,i);
    end
end

for i = 1:n_Dof
    V(i) = V(i)+mi(i)*g*sgsli0(3,4,i);
    DeltaV(i) = gradient(V(i),t(i));
    DeltaV(i) = double(subs(DeltaV(i),t(i),theta(i)));

end


N = double(DeltaV)';

end