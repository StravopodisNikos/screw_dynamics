function [Js, Jb, Jerror] = Jacobians_MMD_2DoF(xi_ai, xi_pi, qa, qp, gst0, gst); 
% assumes A-P-P-A 2 DoF SMM - Only to write Chapter!
Js = zeros(6,2);
Jb = zeros(6,2);
Jerror = zeros(6,2);


% Js = sym(zeros(6,2));
% Jb = sym(zeros(6,2));
% Jerror = sym(zeros(6,2));

exp_ai(:,:,1) = twistexp(xi_ai(:,1), qa(1));
exp_ai(:,:,2) = twistexp(xi_ai(:,2), qa(2));
exp_pi(:,:,1) = twistexp(xi_pi(:,1), qp(1));
exp_pi(:,:,2) = twistexp(xi_pi(:,1), qp(2));

Js(:,1) = xi_ai(:,1);
Js(:,2) = ad(exp_ai(:,:,1)*exp_pi(:,:,1)*exp_pi(:,:,2)) * xi_ai(:,2);

Jb(:,1) = inv(ad(exp_ai(:,:,1)*exp_pi(:,:,1)*exp_pi(:,:,2)*exp_ai(:,:,2)*gst0))*xi_ai(:,1);
Jb(:,2) = inv(ad(exp_ai(:,:,2)*gst0))*xi_ai(:,2);

Jst_s = ad(gst)*Jb;

Jerror = Js-Jst_s;

end