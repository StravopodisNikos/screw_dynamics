function [] = ExecuteExhaustiveConfigSpaceEvaluationForBestDCIpoint(xi_ai,xi_pi,gsli0,gst0,M0b_CoM)
% Executes exhaustive Configuration Space evaluation
%% Areas of different dynamic performance inside the ws of one anatomy are evaluated
% Methodology followed is:
% 1. For BIG step of tai and ALL anatomies FINDS anatomy with BEST DCI
% 2. For ANATOMY of BEST DCI Calculates FKP with small tai step
% 3. For each (x,y,z) given from FKP the reachable WS is found for current
% anatomy
% 4. For each (x,y,z), DCI,Wd are calculated so: 2 3D+1D volumes are
% produced
% 5. EAch volume has areas of different color depending the DMP values

%% 1.Find anatomy of best DCI
% step for active joint angles is 45deg
config_cnt = 0;
global_min_DCI = 10000000;
global_max_Wd = 0;
for ta1=-1.5708:deg2rad(45):1.5708
    config(1)=ta1;
    for ta2=-1.5708:deg2rad(45):1.5708
        config(2)=ta2;
        for ta3=-1.5708:deg2rad(45):1.5708
            config(3)=ta3;
            
            config_cnt = config_cnt + 1;
            config(4:6) = [0 0 0]';
            [DCI(:,:,config_cnt),Wd(:,:,config_cnt),tpi(:,:,config_cnt)] = ExhaustiveConfigSpaceEvaluation_6DoF_MMD(config,xi_ai,xi_pi,gsli0,gst0,M0b_CoM);
            [anat_min_DCI,min_pos1] = min(DCI(:,:,config_cnt));
            if anat_min_DCI<global_min_DCI
                global_min_DCI=anat_min_DCI;
                global_min_DCI_tpi = tpi(:,min_pos1,config_cnt);
            end
            [anat_max_Wd,max_pos2]= max(Wd(:,:,config_cnt));
            if anat_max_Wd>global_max_Wd
                global_max_Wd=anat_max_Wd;
                global_max_Wd_tpi = tpi(:,max_pos2,config_cnt);
            end
            
        end
    end
end

%% 2.Evaluate the previously extracted anatomy
% step for active joint angles is 10deg
config_cnt = 0;
for ta1=-1.5708:deg2rad(10):1.5708
    config(1)=ta1;
    for ta2=-1.5708:deg2rad(5):1.5708
        config(2)=ta2;
        for ta3=-1.5708:deg2rad(5):1.5708
            config(3)=ta3;
            config(4:6) = [0 0 0]';
            
            config_cnt = config_cnt + 1;
            conf_space(:,config_cnt) = config(1:3);
            tpi = global_min_DCI_tpi;
%             tpi = [0 0 0 0 0 0]';
            % Find (x,y,z) from FKP
            [g_ws_point] = MMD_POE_FKP(config,xi_ai,tpi,xi_pi,gst0);
%             ws_points(:,config_cnt) = g_ws_point(1:3,4);
            % Find DCI value;
            [exp_pi,Pi] = CalculatePseudoExponentials(xi_pi,tpi);
            [Js,Jb,Error] = CalculateMetamorphicJacobians_6DoF(config,xi_ai,tpi,xi_pi,Pi,gst0);
            [Jbsli_ref,Jssli_ref,Jbsli_POE_ref,gsli_ref] = CalculateCoMBodyJacobians_6DoF(config,xi_ai,Pi,gsli0);
            [Mb] = CalculateOnlyBodyMassMatrix(Jbsli_ref,M0b_CoM);
            [DCI3(config_cnt)] = CalculateDynamicConditioningIndex(Mb,6);
        end
    end
end
%% 3.Plot ws
x = conf_space(1,1:config_cnt);
y = conf_space(2,1:config_cnt);
z = conf_space(3,1:config_cnt);
scatter3(x,y,z,30,DCI3,'filled')
ax=gca;
ax.XDir = 'reverse';
view(45,45)
xlabel('θ1')
ylabel('θ2')
zlabel('θ3')
cb = colorbar;
cb.Label.String = 'DCI';

end