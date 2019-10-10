function [Si] = Build_SB110(Sim1,Si0,R,P,SBn,p2fig,ti,Xi1)
% Solves FKP for Structural Block C01, given the previous twists and tfs
% Works ONLY for 3 DoF with the predefined ruleset
% INPUT: 1.Sim1 is the struct that contains the previous STRUCTURAL BLOCK INFO
%        after transformed due to structure changes
%        2.Si0 is the struct that contains the current STRUCTURAL BLOCK INFO
%        as default in zero state
% OUTPUT: 1. Si(Built_struct_SB10) that contains new twists and NEW zero tfs
%        of the current STRUCTURAL BLOCK

%% Synthetic Joint is parameterized to exponential
Rx01 = R(1); Ry01 = R(2); Rz01 = R(3);
Px01 = P(1); Py01 = P(2); Pz01 = P(3);

R_lk = rotz(Rz01)*roty(Ry01)*rotx(Rx01);
[wmegaR thetaR] = rotparam(R_lk); % this is SO(3) element-skew matrix and angle
exp_wmegaR = skewexp(wmegaR,thetaR); % ==R_fP1a ALWAYS the same
%  find twist of synthetic joint_it is "equivalent to the reference" since
%  twist must be built for reference structure
xi_lk = createtwist(wmegaR,[Sim1.Cg(1,4)+Px01 Sim1.Cg(2,4)+Py01 Sim1.Cg(3,4)+Pz01]'); %ξk
g_slk = [exp_wmegaR [Sim1.Cg(1,4)+Px01 Sim1.Cg(2,4)+Py01 Sim1.Cg(3,4)+Pz01]'; 0 0 0 1  ];

% % exp_lk = twistexp(xi_lk,thetaR);
% % g_slk = exp_lk*Sim1.Cg; % this is the new g-s-lk after synthetic tf changes aka % this is SE(3) element that represents 
% % % the rigid motion of the structure change induced by synthetic_joint:frame_PseudoConnector1a

% g_fP1a = [exp_wmegaR [Sim1.Cg(1,4)+Px01 Sim1.Cg(2,4)+Py01 Sim1.Cg(3,4)+Pz01]'; 0 0 0 1];
% [twist_g_fP1a theta_g_fP1a] = homtotwist(g_fP1a); % this is SE(3) element that represents 
% % the rigid motion of the structure change induced by synthetic_joint:frame_PseudoConnector1a

% The twists of SB10 are re-calculated due to Synthetic Joint
wj1n = exp_wmegaR*Si0.wi(:,1); g_sP1b = g_slk*Si0.g0(:,:,7);
xj1n = createtwist(wj1n,g_sP1b(1:3,4)); %ξj1'
wj2n = exp_wmegaR*Si0.wi(:,2); g_sP2b = g_slk*Si0.g0(:,:,7)*Si0.g0(:,:,8);
xj2n = createtwist(wj2n,g_sP2b(1:3,4)); %ξj2'
wi1n = exp_wmegaR*Si0.wi(:,3); g_sf1 = g_slk*Si0.g0(:,:,7)*Si0.g0(:,:,8)*Si0.g0(:,:,9);
xi1n = createtwist(wi1n,g_sf1(1:3,4)); %ξi+1'
Xi_for_struct(:,1) = xj1n;
Xi_for_struct(:,2) = xj2n;
Xi_for_struct(:,3) = xi1n;

%% Fwd Kinematic mapping
%% Extract new zero tfs after structure changes
gn(:,:,1) = g_slk*Si0.g0(:,:,7); %gn_s_lj10
gn(:,:,2) = g_slk*Si0.g0(:,:,7)*Si0.g0(:,:,8); %gn_s_lj20
gn(:,:,3) = g_slk*Si0.g0(:,:,7)*Si0.g0(:,:,8)*Si0.g0(:,:,9); %gn_s_li10
gn(:,:,4) = inv(Sim1.g0)*gn(:,:,1); % gn_li_lj10
gn(:,:,5) = inv(gn(:,:,1))*gn(:,:,2); % gn_lj1_lj20
gn(:,:,6) = inv(gn(:,:,2))*gn(:,:,3); % gn_lj2_li10
gn(:,:,7) = inv(Sim1.g0)*gn(:,:,3); % gn_li_li10
gn(:,:,8) = inv(Sim1.g0)*gn(:,:,2); % gn_li_lj20
gn(:,:,9) = inv(gn(:,:,1))*gn(:,:,3); % gn_lj1_li10

NEW_t0_FRAME = g_slk*Si0.Cg; % new {s} frame for next body is the gst of previous set of SBs'

% Now, extract new relative twists
xi_j1_new = inv(ad(Sim1.g0))*xj1n; 
xi_j2_new = inv(ad(Sim1.g0))*xj2n; 
xj1_j2_new = inv(ad(gn(:,:,1)))*xj2n; 
xj2_i1_new = inv(ad(gn(:,:,2)))*xi1n; 
xi_i1_new = inv(ad(Sim1.g0))*xi1n; 

% After new structure is completed, the passive-active expos are
% constructed as usual
exp1 = twistexp(Xi1,ti(1));
expj1 = twistexp(xj1n, ti(2));
expj2 = twistexp(xj2n, ti(3));
expi1 = twistexp(xi1n, ti(4));
exp_for_struct(:,:,1) = expj1;
exp_for_struct(:,:,2) = expj2;
exp_for_struct(:,:,3) = expi1;

% Here is POE FKM for new structure
% ONLY for 3DOF
if SBn==2
    gnj1 = exp1*expj1*gn(:,:,1); % g_s_lj1
    gnj2 = exp1*expj1*expj2*gn(:,:,2); % g_s_lj1
    gni1 = exp1*expj1*expj2*expi1*gn(:,:,3); % g_s_li1
    gnst = exp1*expj1*expj2*expi1*NEW_t0_FRAME;
elseif SBn==3
    % we must check what n.2 was
    if size(Sim1.expi,3)==3 % => previous was SB110
        gnj1 = exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3)*expj1*gn(:,:,1);
        gnj2 = exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3)*expj1*expj2*gn(:,:,2);
        gni1 = exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3)*expj1*expj2*expi1*gn(:,:,3);
        gnst = exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3)*expj1*expj2*expi1*NEW_t0_FRAME;
    elseif size(Sim1.expi,3)==2 % => previous was SB10
        gnj1 = exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*expj1*gn(:,:,1);
        gnj2 = exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*expj1*expj2*gn(:,:,1);
        gni1 = exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*expj1*expj2*expi1*gn(:,:,3);
        gnst = exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*expj1*expj2*expi1*NEW_t0_FRAME;
    end
end
gsn(:,:,1) = gnj1;
gsn(:,:,2) = gnj2;
gsn(:,:,3) = gni1;
%% Relative POE FKM
g_li_li1 = twistexp(xi_j1_new,ti(2))*gn(:,:,4)*twistexp(xj1_j2_new,ti(3))*gn(:,:,5)*twistexp(xj2_i1_new,ti(4))*gn(:,:,6);
%% Relative twists
[xi_i1_abs th_i_i1_abs] = homtotwist(g_li_li1); % the absolute transformation twist- READS structure change
g = g_li_li1*inv(gn(:,:,5));
[xi_i1_rel th_i_i1_rel] = homtotwist(g); % the relative transformation twist - READS pseudojoint change

%% Spatial Jacobian
if SBn==2 % ok for 10
    Js(:,1) = ad(exp1)*xi_lk;
    Js(:,2) = ad(exp1)*xj1n;
    Js(:,3) = ad(exp1*expj1)*xj2n;
    Js(:,4) = ad(exp1*expj1*expj2)*xi1n;
elseif SBn==3 % not yet here
    if size(Sim1.expi,3)==3 % => previous was SB110
        Js(:,1) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3))*xi_lk;
        Js(:,2) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3))*xj1n;
        Js(:,3) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3)*expj1)*xj2n;
        Js(:,4) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*Sim1.expi(:,:,3)*expj1*expj2)*xi1n;
    elseif size(Sim1.expi,3)==2 % => previous was SB10
        Js(:,1) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2))*xi_lk;
        Js(:,2) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2))*xj1n;
        Js(:,3) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*expj1)*xj2n;
        Js(:,4) = ad(exp1*Sim1.expi(:,:,1)*Sim1.expi(:,:,2)*expj1*expj2)*xi1n;
    end
end

figure(p2fig); % for visual evaluation
xk_graph = drawtwist(Js(:,1)); hold on;
xj1_graph = drawtwist(Js(:,2)); hold on;
xj2_graph = drawtwist(Js(:,3)); hold on;
xi1_graph = drawtwist(Js(:,3)); hold on;

% % Js
% % g_li_li1
%% Build OUTPUT struct
f1 = 'g0'; v1 = gni1;
f2 = 'Cg'; v2 = NEW_t0_FRAME; %
f3 = 'expi'; v3 = exp_for_struct;
f4 = 'xi'; v4 = Xi_for_struct;
f5 = 'Sframe'; v5 = NEW_t0_FRAME; %This is the new s(i)->t(i-1) frame only for synthetic config
f6 = 'Js'; v6 = Js;
Si = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6);
end