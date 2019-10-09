function [Si] = Build_SB10(Sim1,Si0,R,P,XIim1,SBn,p2fig)
% Solves FKP for Structural Block C01, given the previous twists and tfs
% INPUT: 1.Sim1 is the struct that contains the previous STRUCTURAL BLOCK INFO
%        after transformed due to structure changes
%        2.Si0 is the struct that contains the current STRUCTURAL BLOCK INFO
%        as default in zero state
% OUTPUT: 1. Si(Built_struct_SB10) that contains new twists and NEW zero tfs
%        of the current STRUCTURAL BLOCK
%        Si.gsli is the new zero tf of active joint
%        Si.expi(:,:,2) for j,i twists
%        Si.xi is the new active twist
%        Si.xj
%        Si.xk
%        Si.Js

%% Synthetic Joint is parameterized to exponential
Rx01 = R(1); Ry01 = R(2); Rz01 = R(3);
Px01 = P(1); Py01 = P(2); Pz01 = P(3);

R_lk = rotz(Rz01)*roty(Ry01)*rotx(Rx01);
[wmegaR thetaR] = rotparam(R_lk); % this is SO(3) element-skew matrix and angle
exp_wmegaR = skewexp(wmegaR,thetaR); % ==R_fP1a ALWAYS the same
%  find twist of synthetic joint_it is "equivalent to the reference" since
%  twist must be built for reference structure
xi_lk = createtwist(wmegaR,[Sim1.Cg(1,4)+Px01 Sim1.Cg(2,4)+Py01 Sim1.Cg(3,4)+Pz01]'); %ξk
exp_lk = twistexp(xi_lk,thetaR);
g_slk = exp_lk*Sim1.Cg; % this is the new g-s-lk after synthetic tf changes aka % this is SE(3) element that represents 
% the rigid motion of the structure change induced by synthetic_joint:frame_PseudoConnector1a

% g_fP1a = [exp_wmegaR [Sim1.Cg(1,4)+Px01 Sim1.Cg(2,4)+Py01 Sim1.Cg(3,4)+Pz01]'; 0 0 0 1];
% [twist_g_fP1a theta_g_fP1a] = homtotwist(g_fP1a); % this is SE(3) element that represents 
% % the rigid motion of the structure change induced by synthetic_joint:frame_PseudoConnector1a

% The twists of SB10 are re-calculated due to Synthetic Joint
wjn = exp_wmegaR*Si0.wi(:,2); g_sP1b = g_slk*Si0.g0(:,:,8);
xjn = createtwist(wjn,g_sP1b(1:3,4)); %ξj'
wi1n = exp_wmegaR*Si0.wi(:,3); g_sf1 = g_slk*Si0.g0(:,:,8)*Si0.g0(:,:,5);
xi1n = createtwist(wi1n,g_sf1(1:3,4)); %ξi+1'

%% Fwd Kinematic mapping
ti = [0 0]'; % from here i control the 2 angles: 1:passive1 2:active1
%% Extract new zero tfs after structure changes
gn(:,:,1) = g_slk*Si0.g0(:,:,8); % g_s_lj0n = g_s_lk*g_lk_lj0
gn(:,:,2) = g_slk*Si0.g0(:,:,8)*Si0.g0(:,:,5); % g_s_li10n
gn(:,:,3) = inv(Sim1.gsli)*gn(:,:,1); % g_li_lj0
gn(:,:,4) = inv(gn(:,:,1))*gn(:,:,2); % g_lj_li10
gn(:,:,5) = inv(Sim1.gsli)*gn(:,:,2); % g_li_li10

% Now, extract new relative twists
xi_j_new = inv(ad(Sim1.gsli))*xjn;
xi_i1_new = inv(ad(Sim1.gsli))*xi1n;
xj_i1_new = inv(ad(gn(:,:,1)))*xi1n;

% After new structure is completed, the passive-active expos are
% constructed as usual
expj = twistexp(xjn, ti(1));
expi = twistexp(xi1n, ti(2));

% Here is POE FKM for new structure
% ONLY for 3DOF
if SBn==2
    gnj = exp1*expj*gn(:,:,1); % g_s_lj
    gni = exp1*expj*expi*gn(:,:,2); % g_s_li1
elseif SBn==3
    % we must check what n.2 was
    if length(Sim1.expi)==3 % => previous was SB110
        gnj = exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3)*expj*gn(:,:,1);
        gni = exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3)*expj*expi*gn(:,:,2);
    elseif length(Sim1.expi)==2 % => previous was SB10
        gnj = exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*expj*gn(:,:,1);
        gni = exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*expj*expi*gn(:,:,2);        
    end
end

%% Relative POE FKM
g_li_li1 = twistexp(xi_j_new,ti(1))*gn(:,:,3)*twistexp(xj_i1_new,ti(2))*gn(:,:,4); % for synthetic+metamorphic
%% Relative twists
[xi_i1_abs th_i_i1_abs] = homtotwist(g_li_li1); % the absolute transformation twist- READS structure change
g = g_li_li1a*inv(gn(:,:,5));
[xi_i1_rel th_i_i1_rel] = homtotwist(g); % the relative transformation twist - READS pseudojoint change

%% Spatial Jacobian
if SBn==2
    Js(:,1) = ad(exp1)*xi_lk;
    Js(:,2) = ad(exp1)*xjn;
    Js(:,3) = ad(exp1*expj)*xi1n;
elseif SBn==3
    if length(Sim1.expi)==3 % => previous was SB110
        Js(:,1) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3))*xi_lk;
        Js(:,2) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3))*xjn;
        Js(:,3) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3)*expj)*xi1n;
    elseif length(Sim1.expi)==2 % => previous was SB10
        Js(:,1) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2))*xi_lk;
        Js(:,2) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2))*xjn;
        Js(:,3) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*expj)*xi1n;
    end
end

figure(p2fig); % for visual evaluation
xk_graph = drawtwist(Js(:,1)); hold on;
xj_graph = drawtwist(Js(:,2)); hold on;
xi1_graph = drawtwist(Js(:,3)); hold on;

%% Build OUTPUT struct

end