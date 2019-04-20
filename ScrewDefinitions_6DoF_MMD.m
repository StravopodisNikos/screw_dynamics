function [wi,wpi,xi_ai,xi_pi,exp_pi_ref,exp_pi_test,Pi_ref,Pi_test] = ScrewDefinitions_6DoF_MMD(pi_ref,pi_test,tpi_ref,tpi_test)
%% Based on robot structure @reference anatomy
% wi: rotations of active twists
%% Active screws
wi(:,1) = [0 0 1]';
wi(:,2) = [0 1 0]';
wi(:,3) = [0 1 0]';
wi(:,4) = [1 0 0]';
wi(:,5) = [0 1 0]';
wi(:,6) = [1 0 0]';
%% Pseudo screws
wpi(:,1) = [0 1 0]';
wpi(:,2) = [1 0 0]';
wpi(:,3) = [0 1 0]';
wpi(:,4) = [1 0 0]';
wpi(:,5) = [0 1 0]';
wpi(:,6) = [1 0 0]';
%% Active twists - reference anatomy
xi_ai(:,1) = createtwist(wi(:,1),pi_ref(:,1));
xi_ai(:,2) = createtwist(wi(:,2),pi_ref(:,3));
xi_ai(:,3) = createtwist(wi(:,3),pi_ref(:,6));
xi_ai(:,4) = createtwist(wi(:,4),pi_ref(:,10));
xi_ai(:,5) = createtwist(wi(:,5),pi_ref(:,11));
xi_ai(:,6) = createtwist(wi(:,6),pi_ref(:,12));
%% Pseudo twists
xi_pi(:,1) = createtwist(wpi(:,1),pi_ref(:,1));
xi_pi(:,2) = createtwist(wpi(:,2),pi_ref(:,2));
xi_pi(:,3) = createtwist(wpi(:,3),pi_ref(:,4));
xi_pi(:,4) = createtwist(wpi(:,4),pi_ref(:,5));
xi_pi(:,5) = createtwist(wpi(:,5),pi_ref(:,7));
xi_pi(:,6) = createtwist(wpi(:,6),pi_ref(:,8));
%% Pseudo exponenentials
%  (can be calculated since anatomy is specified by user in main programm)
exp_pi_ref(:,:,1) = twistexp(xi_pi(:,1), tpi_ref(1));
exp_pi_ref(:,:,2) = twistexp(xi_pi(:,2), tpi_ref(2));
exp_pi_ref(:,:,3) = twistexp(xi_pi(:,3), tpi_ref(3));
exp_pi_ref(:,:,4) = twistexp(xi_pi(:,4), tpi_ref(4));
exp_pi_ref(:,:,5) = twistexp(xi_pi(:,5), tpi_ref(5));
exp_pi_ref(:,:,6) = twistexp(xi_pi(:,6), tpi_ref(6));
exp_pi_test(:,:,1) = twistexp(xi_pi(:,1), tpi_test(1));
exp_pi_test(:,:,2) = twistexp(xi_pi(:,2), tpi_test(2));
exp_pi_test(:,:,3) = twistexp(xi_pi(:,3), tpi_test(3));
exp_pi_test(:,:,4) = twistexp(xi_pi(:,4), tpi_test(4));
exp_pi_test(:,:,5) = twistexp(xi_pi(:,5), tpi_test(5));
exp_pi_test(:,:,6) = twistexp(xi_pi(:,6), tpi_test(6));
%% Product of pseudo exponentials
Pi_test(:,:,1) = exp_pi_test(:,:,1)*exp_pi_test(:,:,2);
Pi_ref(:,:,1) = exp_pi_ref(:,:,1)*exp_pi_ref(:,:,2);
Pi_test(:,:,2) = exp_pi_test(:,:,3)*exp_pi_test(:,:,4);
Pi_ref(:,:,2) = exp_pi_ref(:,:,1)*exp_pi_ref(:,:,2);
Pi_test(:,:,3) = exp_pi_test(:,:,5)*exp_pi_test(:,:,6);
Pi_ref(:,:,3) = exp_pi_ref(:,:,5)*exp_pi_ref(:,:,6);

end