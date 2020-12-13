function [N] = calculate_gravity_matrix_numer(dof_string, gsli, mi, Vold)
%
switch dof_string
    case '3dof'
        n_Dof = 3;
    case '6dof'
        n_Dof = 6;
end

g = -9.8067; % [m/s^2]

V = zeros(n_Dof,1);
DeltaV = zeros(n_Dof,1);
N = zeros(n_Dof,1);

for i = 1:n_Dof
    V(i) = V(i)+mi(i)*g*gsli(3,4,i);
    DeltaV(i) = (V(i) - Vold(i) );
    N(i) = DeltaV(i) / dq(i);

end

end