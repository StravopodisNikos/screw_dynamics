function [Jbsli,Jbsli_427] = Jbody_CoM_2DoF(xi_ai, exp_ai, Pi1,  gsli0, i)

Jbsli = zeros(6,2);
Jbsli_427 = zeros(6,2);

% Based on eq. p.168
if i==1
    Jbsli(:,1) = inv(ad(exp_ai(:,:,1)*gsli0(:,:,1)))*xi_ai(:,1);
elseif i==2
    Jbsli(:,1) = inv(ad(exp_ai(:,:,1)*Pi1*exp_ai(:,:,2)*gsli0(:,:,2)))*xi_ai(:,1);
    Jbsli(:,2) = inv(ad(exp_ai(:,:,2)*gsli0(:,:,2)))*xi_ai(:,2);
end

% Based on eq.4.27 p.176
if i==1
    for j=1:2
%         Jbsli_427(:,1) = ad(inv(gsli0(:,:,1)))*eye(6)*xi_ai(:,1);
        [Aij_427] = compute_Aij_for_Ji_427(exp_ai, Pi1, i, j);
        Jbsli_427(:,j) = ad(inv(gsli0(:,:,i)))*Aij_427*xi_ai(:,j);
    end
elseif i==2
%     Jbsli_427(:,1) = ad(inv(gsli0(:,:,2)))* ad(inv(Pi1*exp_ai(:,:,2))) *xi_ai(:,1);
%     Jbsli_427(:,2) = ad(inv(gsli0(:,:,2)))*eye(6)*xi_ai(:,2);
    for j=1:2
%         Jbsli_427(:,1) = ad(inv(gsli0(:,:,1)))*eye(6)*xi_ai(:,1);
        [Aij_427] = compute_Aij_for_Ji_427(exp_ai, Pi1, i, j);
        Jbsli_427(:,j) = ad(inv(gsli0(:,:,i)))*Aij_427*xi_ai(:,j);
    end
end

Jbsli_error = Jbsli-Jbsli_427;
end