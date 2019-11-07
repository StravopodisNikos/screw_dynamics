function f_star = WeightedKinDynIsotropyInvestigation(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM, M0s_CoM)
%% Get chromosome values
step_a2 = x(1);
step_a3 = x(2);
ci = x(3:6);
%% Determine tpi pseudojoint angles
step_angle = deg2rad(15); %ci:1~13
tpi0 = [-1.5708 -1.5708 -1.5708 -1.5708]';
% m = pi/step_angle + 1; % number of pseudojoint variables set M=[i] 1<i<m 
for k=1:4
    tpi(k) = step_angle * (floor(ci(k)) - 1) + tpi0(k);
end
% Recompute pseudo exponentials
[~,Pi] = CalculatePseudoExponentials_3DoF(xi_pi,tpi);

%% I.a. Definition of working region
% Searches in the Configuration Space of first 3DoF for all anatomies
p_count = 0;
    for ta2=-1.5708:step_a2:1.5708
        for ta3=-1.5708:step_a3:1.5708
            q = [0.1 ta2 ta3];
            p_count = p_count + 1;
            % Kinematics
            [~,gd] = MMD_POE_FKP_3DoF(q',xi_ai,tpi,xi_pi,gst0);
            qs = gd(1:3,4);  % for TCP in Base Frame
            qb = zeros(3,1); % for TCP in Tool Frame 
            [Js,Jb,~] = CalculateMetamorphicJacobians_3DoF(q',xi_ai,tpi,xi_pi,Pi,gst0);
            [J33b_new] = CalculateSquareTCPJacobian_3DoF(Jb,qb);
            [J33s_new] = CalculateSquareTCPJacobian_3DoF(Js,qs);
            [Jbsli,Jssli,~] = CalculateCoMBodyJacobians_3DoF(q',xi_ai,Pi,gsli0);
            [Mb] = CalculateOnlyBodyMassMatrix(Jbsli,M0b_CoM);
            [Ms] = CalculateOnlySpatialMassMatrix(Jssli,M0s_CoM);
            % Performance measures calculation
            % Kinematic Isotropy
            Ks = J33s_new*J33s_new';
            [U3s,S3s,V3s] = svd(Ks);
            Kb = J33b_new*J33b_new';
            [U3b,S3b,V3b] = svd(Kb);
            
            % Dynamic Isotropy
            % == Mass Matrix == %
            [U1s,S1s,V1s] = svd(Ms);
            [U1b,S1b,V1b] = svd(Mb); 
            % == Acceleration Ellipsoid core == %
            trq_u = [44.7 25.3 25.3]'; 
            L = diag(trq_u);
            qb = zeros(3,1);
            [J33b_new] = CalculateSquareTCPJacobian_3DoF(Jb,qb);
            [J33s_new] = CalculateSquareTCPJacobian_3DoF(Js,qs);
            Qb = inv(J33b_new')*Mb'*L^2*Mb*inv(J33b_new); 
            Qb2 = inv(J33b_new')*Mb'*Mb*inv(J33b_new);
            [U2b,S2b,V2b] = svd(Qb);
            [U2b2,S2b2,V2b2] = svd(Qb2);
            Qs = inv(J33s_new')*Ms'*L^2*Ms*inv(J33s_new);
            Qs2 = inv(J33s_new')*Ms'*Ms*inv(J33s_new);
            [P2s,D2s] = eig(Qs);
            [U2s,S2s,V2s] = svd(Qs);
            [P2s2,D2s2] = eig(Qs2);
            [U2s2,S2s2,V2s2] = svd(Qs2);
            
            wk = 0.5;
            wd = 0.5;
            
%% Here I choose objective function for weighted kinematic+dynamic isotropy:
            if abs(det(J33s_new))<1e-10
                % Close to singularity!
                f1s(p_count) = 1e+10;
            else
            f1s(p_count) = wk*(S3s(1,1)/S3s(3,3)) + wd*(S1s(1,1)/S1s(3,3)); % Ks & Ms
%             f1b(p_count) = wk*(S3b(1,1)/S3b(3,3)) + wd*(S1b(1,1)/S1b(3,3)); % Kb & Mb

%             f2s(p_count) = wk*(S3s(1,1)/S3s(3,3)) + wd*(S2s(1,1)/S2s(3,3)); % Ks & Qs
%             f2b(p_count) = wk*(S3b(1,1)/S3b(3,3)) + wd*(S2b(1,1)/S2b(3,3)); % Kb & Qb

%             f3s2(p_count) = wk*(S3s(1,1)/S3s(3,3)) + wd*(S2s2(1,1)/S2s2(3,3)); % Ks & Qs2
%             f3b2(p_count) = wk*(S3b(1,1)/S3b(3,3)) + wd*(S2b2(1,1)/S2b2(3,3)); % Kb & Qb2
            end
            
            end
    end
    
f_star = min(f1s); % minimizes the minimum condition number = (s_max/s_min) /in [1, Inf]
end
