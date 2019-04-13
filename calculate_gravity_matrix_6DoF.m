function [N] = calculate_gravity_matrix_6DoF(xi_ai, gsli0, mi,  Pi, theta)

n_Dof = size(xi_ai,2); % extract number of links
g = -9.8067; % [m/s^2]
V = sym('V',[n_Dof 1]);

t = sym('t', [1 n_Dof]);
% Calculates symbolic exponentials of twist matrices 
for i = 1:n_Dof
    sexp(:,:,i)=twistexp(xi_ai(:,i), t(i));
    % Calculates symbolic FKP
    if i ==1
        sgsli0(:,:,i) = sexp(:,:,i)*gsli0(:,:,i);
    elseif i==2
        sgsli0(:,:,i) = sexp(:,:,i-1)*Pi(:,:,i-1)*sexp(:,:,i)*gsli0(:,:,i);
    elseif i==3
        sgsli0(:,:,i) = sexp(:,:,i-2)*Pi(:,:,i-2)*sexp(:,:,i-1)*Pi(:,:,i-1)*sexp(:,:,i)*gsli0(:,:,i);
    elseif i==4
        sgsli0(:,:,i) = sexp(:,:,i-3)*Pi(:,:,i-3)*sexp(:,:,i-2)*Pi(:,:,i-2)*sexp(:,:,i-1)*Pi(:,:,i-1)*sexp(:,:,i)*gsli0(:,:,i);
    elseif i==5
        sgsli0(:,:,i) = sexp(:,:,i-4)*Pi(:,:,i-4)*sexp(:,:,i-3)*Pi(:,:,i-3)*sexp(:,:,i-2)*Pi(:,:,i-2)*sexp(:,:,i-1)*sexp(:,:,i)*gsli0(:,:,i);    
    elseif i==6
        sgsli0(:,:,i) = sexp(:,:,i-5)*Pi(:,:,i-5)*sexp(:,:,i-4)*Pi(:,:,i-4)*sexp(:,:,i-3)*Pi(:,:,i-3)*sexp(:,:,i-2)*sexp(:,:,i-1)*sexp(:,:,i)*gsli0(:,:,i);
    end
end

for i = 1:n_Dof
    V(i) = V(i)+mi(i)*g*sgsli0(3,4,i);
    DeltaV(i) = gradient(V(i),t(i));
    if i==2
        DeltaV(i) = double(subs(DeltaV(i),t(i),theta(i)));
    elseif i==3
        DeltaV(i) = double(subs(DeltaV(i),[t(i-1) t(i)],[theta(i-1) theta(i)]));
    elseif i==4
        DeltaV(i) = double(subs(DeltaV(i),[t(i-2) t(i-1) t(i)],[theta(i-2) theta(i-1) theta(i)]));
    elseif i==5
        DeltaV(i) = double(subs(DeltaV(i),[t(i-3) t(i-2) t(i-1) t(i)],[theta(i-3) theta(i-2) theta(i-1) theta(i)]));
    elseif i==6
        DeltaV(i) = double(subs(DeltaV(i),[t(i-4) t(i-3) t(i-2) t(i-1) t(i)],[theta(i-4) theta(i-3) theta(i-2) theta(i-1) theta(i)]));
    end
end


N = double(DeltaV)';

end