function DCI_star = GlobalMinMaxDCI_3DoF(x,xi_ai,xi_pi,gst0,gsli0,M0b_CoM)
% Objective function that minimizes max(DCI) in each generation

%% Add geometry libraries
addpath('/home/nikos/matlab_ws/geom3d/geom3d')
addpath('/home/nikos/matlab_ws/geom2d/utils')
addpath('/home/nikos/matlab_ws/geom3d/meshes3d')

%% Get chromosome values
% step_a1 = x(1);
step_a2 = x(1);
step_a3 = x(2);
ci = x(3:6);
%% Determine tpi pseudojoint angles
step_angle = deg2rad(15);
tpi0 = [-pi/2 -pi/2 -pi/2 -pi/2]';
% m = pi/step_angle + 1; % number of pseudojoint variables set M=[i] 1<i<m 
for k=1:4
    tpi(k) = step_angle * (floor(ci(k)) - 1) + tpi0(k);
end
% Recompute pseudo exponentials
[~,Pi] = CalculatePseudoExponentials_3DoF(xi_pi,tpi);

%% I.a. Definition of working region
% Searches in the Configuration Space of first 3DoF for all anatomies
p_count = 0;
% for ta1=-1.5708:step_a1:1.5708
    for ta2=-1.5708:step_a2:1.5708
        for ta3=-1.5708:step_a3:1.5708
            q = [0.1 ta2 ta3];
            p_count = p_count + 1;
            % Recompute active exponentials
    %         exp_ai(:,:,1) = twistexp(xi_ai(:,1), q(1));
    %         exp_ai(:,:,2) = twistexp(xi_ai(:,2), q(2));
    %         exp_ai(:,:,3) = twistexp(xi_ai(:,3), q(3));
    %         exp_ai(:,:,4) = twistexp(xi_ai(:,4), q(4));
    %         exp_ai(:,:,5) = twistexp(xi_ai(:,5), q(5));
    %         exp_ai(:,:,6) = twistexp(xi_ai(:,6), q(6));
            %% I.b. In this working region compute J,Jdot and dynamic matrices
            % Jacobians: J,Jdot
    %         [~,Jb,~] = CalculateMetamorphicJacobians_6DoF(q',xi_ai,tpi,xi_pi,Pi,gst0);
            % Compute CoM Jacobian
            [Jbsli,~,~] = CalculateCoMBodyJacobians_3DoF(q',xi_ai,Pi,gsli0);
            %% Dynamic matrices: M,V,G - Only M here for ellipsoid calculation
            [Mb] = CalculateOnlyBodyMassMatrix(Jbsli,M0b_CoM);
            % Calculate Dynamic Performance Measures values
            wx = [0 1 0]; % only M23 element considered
            [DCI(p_count),~,~,~] = CalculateDynamicConditioningIndex2_3DoF(Mb,3,wx);
        end
    end
% end

DCI_star = max(DCI); 
end