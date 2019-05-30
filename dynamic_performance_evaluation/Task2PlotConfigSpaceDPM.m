function [DPM] = Task2PlotConfigSpaceDPM(xi_ai,xi_pi,tpi,gsli0,gst0,M0b_CoM)
% Plots Configuration Space & Cartesian workspace of best anatomies extracted for each global DPM
% tpi vector MUST specify the best anatomy extracted from optimization for
% the DPM considered

%% 1. Compute DPM in Configuration Space
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
            % Find (x,y,z) from FKP
            [g_ws_point] = MMD_POE_FKP(config,xi_ai,tpi,xi_pi,gst0);
            ws_points(:,config_cnt) = g_ws_point(1:3,4);
            
            [~,Pi] = CalculatePseudoExponentials(xi_pi,tpi);
            [~,Jb,~] = CalculateMetamorphicJacobians_6DoF(config,xi_ai,tpi,xi_pi,Pi,gst0);
            [Jbsli,~,~,~] = CalculateCoMBodyJacobians_6DoF(config,xi_ai,Pi,gsli0);
            
            [Mb] = CalculateOnlyBodyMassMatrix(Jbsli,M0b_CoM);
            [DCI(config_cnt)] = CalculateDynamicConditioningIndex(Mb,6);
            [Wd(config_cnt)] = CalculateDynamicManipulabilityIndex(Jb,Mb);
            [LDII(config_cnt)] = CalculateLocalDynamicIsotropyIndex(Jb, Mb);
        end
    end
end

%%
% % Comment here in order to choose DPM!!!
%% 2. Plot DPM in Config Space
DPM = LDII; % Choose DPM to plot
x = conf_space(1,1:config_cnt);
y = conf_space(2,1:config_cnt);
z = conf_space(3,1:config_cnt);
scatter3(x,y,z,50,DPM,'filled')
ax=gca;
ax.XDir = 'reverse';
view(45,45)
xlabel('θ1')
ylabel('θ2')
zlabel('θ3')
cb = colorbar;
cb.Label.String = 'LDII'; % change depending on DPM specifed

%% 2. Plot DPM in Cartesian Space
DPM = LDII;
x = ws_points(1,1:config_cnt);
y = ws_points(2,1:config_cnt);
z = ws_points(3,1:config_cnt);
scatter3(x,y,z,50,DPM,'filled')
ax=gca;
ax.XDir = 'reverse';
view(45,45)
xlabel('x')
ylabel('y')
zlabel('z')
cb = colorbar;
cb.Label.String = 'LDII'; % change depending on DPM specifed

end