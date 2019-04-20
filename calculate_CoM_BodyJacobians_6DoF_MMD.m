function [Jbsli,Jssli,Jbsli_POE,gsli] = calculate_CoM_BodyJacobians_6DoF_MMD(config,xi_ai,Pi,gsli0)
%% Returns Spatial and Body Jacobians
%  for test and reference anatomy for user-given configurations

% config is vector of joint-angles values for the desired gd for the
% anatomy evaluated. They are given from 6DoF IKP solver

%% Recalculate active exponentials for given configuration
exp_ai(:,:,1) = twistexp(xi_ai(:,1), config(1));
exp_ai(:,:,2) = twistexp(xi_ai(:,2), config(2));
exp_ai(:,:,3) = twistexp(xi_ai(:,3), config(3));
exp_ai(:,:,4) = twistexp(xi_ai(:,4), config(4));
exp_ai(:,:,5) = twistexp(xi_ai(:,5), config(5));
exp_ai(:,:,6) = twistexp(xi_ai(:,6), config(6));

%% FK mappings of CoM for reference anatomy and calculated configuration
gsli(:,:,1) = exp_ai(:,:,1)*gsli0(:,:,1);
gsli(:,:,2) = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*gsli0(:,:,2);
gsli(:,:,3) = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gsli0(:,:,3);
gsli(:,:,4) = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*exp_ai(:,:,4)*gsli0(:,:,4);
gsli(:,:,5) = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*exp_ai(:,:,4)*exp_ai(:,:,5)*gsli0(:,:,5);
gsli(:,:,6) = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*exp_ai(:,:,4)*exp_ai(:,:,5)*exp_ai(:,:,6)*gsli0(:,:,6);
%% Calculate Jacobians
for i=1:6
    [Jbsli(:,:,i),Jssli(:,:,i),Jbsli_POE(:,:,i)] = Jbody_CoM_6DoF(xi_ai, exp_ai, Pi, gsli0, gsli, i);
%     [Jbsli_ref(:,:,i),Jssli_ref(:,:,i),Jbsli_ref_POE(:,:,i)] = Jbody_CoM_6DoF(xi_ai, exp_ai, Pi_ref, gsli0, gsli0, i);
end

end