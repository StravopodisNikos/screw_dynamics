function [g1,g2,gst] = MMD_POE_FKP_2DoF(xi_ai,xi_pi,qa,qp,gst0,gs10,gs20)
% assumes A-P-P-A 2 DoF SMM - Only to write Chapter!

exp_ai(:,:,1) = twistexp(xi_ai(:,1),qa(1));
exp_ai(:,:,2) = twistexp(xi_ai(:,2),qa(2));

exp_pi(:,:,1) = twistexp(xi_pi(:,1),qp(1));
exp_pi(:,:,2) = twistexp(xi_pi(:,2),qp(2));

g1 = exp_ai(:,:,1)*gs10;
g2 = exp_ai(:,:,1)*exp_pi(:,:,1)*exp_pi(:,:,2)*exp_ai(:,:,2)*gs20;
gst = exp_ai(:,:,1)*exp_pi(:,:,1)*exp_pi(:,:,2)*exp_ai(:,:,2)*gst0;

end