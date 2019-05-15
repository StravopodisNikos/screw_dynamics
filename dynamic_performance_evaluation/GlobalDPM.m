function [Gwd,Gdci,HPAwd,HPAdci,ws_vol,HP_DCI_vol,HP_Wd_vol] = GlobalDPM(xi_ai,tpi,xi_pi,gst0,gsli0,M0b_CoM)
% For better performance RUN in parallel pool!!!
% add geometry libraries
addpath('/home/nikos/matlab_ws/geom3d/geom3d')
addpath('/home/nikos/matlab_ws/geom2d/utils')
addpath('/home/nikos/matlab_ws/geom3d/meshes3d')

% Runs exhaustively in Configuration Space
% Calculates Dynamic Manipulability & Dynamic Conditioning Indexes in each point
% 1.Calculates the average value of the DPM inside the reachable workspace volume
% 2.Finds the HPA for each DPM given a user defined DPM limit

config_cnt = 0;
HP_DCI_points_cnt = 0;
HP_Wd_points_cnt =0;
sumDCI = 0;
sumWd = 0;
limWd = 1.50e+06;
limDCI = 0.01;
a=0.015; %m Ws volume is approximated as sum of cubes of a=1.5cm
ws_vol = 0; % m^3
HP_DCI_vol = 0;
HP_Wd_vol = 0;

%% Run for Configuration Space, only first 3 DoF
step_angle = 10;
for ta1=-1.5708:deg2rad(step_angle):1.5708
    config(1)=ta1;
    for ta2=-1.5708:deg2rad(step_angle):1.5708
        config(2)=ta2;
        for ta3=-1.5708:deg2rad(step_angle):1.5708
            config(3)=ta3;
            config(4:6) = [0 0 0]'; % wrist joints not considered
            % counter of CS point evaluated
            config_cnt = config_cnt + 1;
            % save configuration space variables for each point
            conf_space(:,config_cnt) = config(1:3);
            % Find (x,y,z) from FKP
            [g_ws_point] = MMD_POE_FKP(config,xi_ai,tpi,xi_pi,gst0);
            % save cartesian space variables
            ws_points(:,config_cnt) = g_ws_point(1:3,4);
            %% Calculate DPM at current point
            % Calculate values affected by tpi changes;
            [~,Pi] = CalculatePseudoExponentials(xi_pi,tpi);
            % Compute Jacobian
            [Js,~,Error] = CalculateMetamorphicJacobians_6DoF(config,xi_ai,tpi,xi_pi,Pi,gst0);
            % Compute CoM Jacobian 
            [Jbsli_ref,~,Jbsli_POE_ref,~] = CalculateCoMBodyJacobians_6DoF(config,xi_ai,Pi,gsli0);
            % Compute Mass matrix
            [Mb] = CalculateOnlyBodyMassMatrix(Jbsli_ref,M0b_CoM);
            % Calculate Dynamic Performance Measures values
            [DCI(config_cnt)] = CalculateDynamicConditioningIndex(Mb,6);
            [Wd(config_cnt)] = calculateDynamicManipulabilityIndex(Js,Mb);
            sumDCI = sumDCI + DCI(config_cnt);
            sumWd = sumWd + Wd(config_cnt);
            % Calculate cubic volume
            ws_vol = ws_vol + a^3;
            % Determine HPA for each DPM;
            if DCI(config_cnt) < limDCI
                HP_DCI_points_cnt = HP_DCI_points_cnt+1;
                HP_DCI_vol = HP_DCI_vol + a^3;
                HP_DCI_points(:,HP_DCI_points_cnt) = ws_points(:,config_cnt);
            end
            if Wd(config_cnt) > limWd
                HP_Wd_points_cnt = HP_Wd_points_cnt+1;
                HP_Wd_vol = HP_Wd_vol + a^3;
                HP_Wd_points(:,HP_Wd_points_cnt) = ws_points(:,config_cnt);
            end
        end
    end
end

%% Aggregates
[minLm] = CalculateMinimumAnatomyLinkLength(tpi);
[maxVm] = CalculateMaximumAnatomyWsVolume(minLm);
volumes = ws_vol/maxVm;

Gwd = sumWd/maxVm; % Murray volume
Gdci = sumDCI/maxVm;
HPAwd = HP_Wd_vol/ws_vol; % cube volumes 
HPAdci = HP_DCI_vol/ws_vol;
%% Plots

%% 1.Cartesian Space and DCI(colour bar)
f1 = figure('Name','DCI in Cartesian WS');
x = ws_points(1,1:config_cnt);
y = ws_points(2,1:config_cnt);
z = ws_points(3,1:config_cnt);
figure(f1);
scatter3(x,y,z,30,DCI,'filled')
ax1=gca;
ax1.XDir = 'reverse';
view(45,45)
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
map = [0 1 1; 1 0 0]
cb = colorbar;
cb.Label.String = 'DCI';
%% 2.Cartesian Space and Wd(colour bar)
f2 = figure('Name','Wd in Cartesian WS');
x = ws_points(1,1:config_cnt);
y = ws_points(2,1:config_cnt);
z = ws_points(3,1:config_cnt);
figure(f2);
scatter3(x,y,z,30,Wd,'filled')
ax2=gca;
ax2.XDir = 'reverse';
view(45,45)
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
cb = colorbar;
cb.Label.String = 'Wd';
%% 3.Cartesian Space and HP_DCI_vol(red colour)
% % f3 = figure('Name','HPA of DCI inside Manipulator Workspace');
% % for i = 1:config_cnt
% %     drawCube([ws_points(1,i) ws_points(2,i) ws_points(3,i)  a  0 0 0], 'FaceColor', 'c'); % Cartesian Space
% %     hold on;
% % end
% % for i =1:HP_DCI_points_cnt
% %     drawCube([HP_DCI_points(1,i) HP_DCI_points(2,i) HP_DCI_points(3,i)  a  0 0 0], 'FaceColor', 'r'); % HPA DCI
% %     hold on;
% % end
% % %% 4.Cartesian Space and HP_Wd_vol(green colour)
% % f4 = figure('Name','HPA of Wd inside Manipulator Workspace');
% % for i = 1:config_cnt
% %     drawCube([ws_points(1,i) ws_points(2,i) ws_points(3,i)  a  0 0 0], 'FaceColor', 'c'); % Cartesian Space
% %     hold on;
% % end
% % for i =1:HP_Wd_points_cnt
% %     drawCube([HP_Wd_points(1,i) HP_Wd_points(2,i) HP_Wd_points(3,i)  a  0 0 0], 'FaceColor', 'g'); % HPA Wd
% %     hold on;
% % end 

end