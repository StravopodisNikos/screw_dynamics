clear;
clc;
close all;

%% load libraries
% load Murray kinematics
addpath('/home/nikos/matlab_ws/kinematics/robotlinks')
addpath('/home/nikos/matlab_ws/kinematics/screws') 
addpath('/home/nikos/matlab_ws/kinematics/util')
% load Js-Jb calculation function
addpath('/home/nikos/matlab_ws/project_ABBpaper/matlab_files')
% load main folder
addpath('/home/nikos/matlab_ws/modular_dynamixel/')
% load geom3d library
addpath('/home/nikos/matlab_ws/geom3d')
addpath('/home/nikos/matlab_ws/geom3d/geom3d')
addpath('/home/nikos/matlab_ws/geom2d/geom2d')
addpath('/home/nikos/matlab_ws/geom2d/utils')

point2figure = figure;

%% load zero data for STRUCTURAL BLOCKS
% data are obtained by .m files:
% modular_dynamixel/structural_synthesis/kinematic_verification_C01.m for SB01
% modular_dynamixel/structural_synthesis/kinematic_verification_C011.m for SB011
load('SB10_zero_data.mat');
pi_10 = pi(:,2:4); % j-i1-k(!)
wi_10 = wi(:,2:3); % j-i1
g0_10 = g0;
gsc_10 = g0(:,:,9); % g_s_lk0
g_lk_TOOL_10 = g0(:,:,10);
xi_10 = xi;
load('SB110_zero_data.mat');
pi_110 = pi(:,2:5); % j1-j2-i1-k(!)
wi_110 = wi(:,2:4); % j1-j2-i1
g0_110 = g0;
gsc_110 = g0(:,:,2); % g_s_lk0
g_lk_TOOL_110 = g0(:,:,14);
xi_110 = xi;
%% First the reference structure-anatomy data are loaded

%% Structure string selected by ga
% S = S1 - S2 - S3 => here we select ONLY for evaluation structure: 
% 0 -C- 10 -C- 110 : Active1 - Synthetic1- Passive1- Active2 -Passive2-
% Passive3 - Active3
%% Arbitrarily selected %%
% SB0 = '0';
% SB10 = '10';
% SB110 = '110';
structure(1,:) = 'xxSB0';
structure(2,:) = 'xSB10';
% structure(2,:) = 'SB110';
structure(3,:) = 'SB110'; % Normally this is extracted by ga 
% structure(3,:) = 'xSB10';
%% END OF MANUAL INTERFERENCE %%
% First is always a lonely active joint at z axis rotation
t0 = 0; % first active angle

p1_0 = [0 0 0.0570]';
w1_0 = [0 0 1]';
x1 = createtwist(w1_0,p1_0); %ξa1
% zero tf to the first active frame
g_s_li1_0 = [   1.0000      0         0         0;...
                 0          1.0000    0         0;...
                 0          0         1.0000    0.0570;...
                 0          0         0         1.0000];
% zero tf to the first connection Point of Synthetic Joint
% (zero tf between {s} -> frame
g_s_lk1_0 = [1.0000    0         0          0.0910;...
             0         1.0000    0          0;...
             0         0         1.0000     0.0570;...
             0         0         0          1.0000];
exp1 = twistexp(x1,t0);
f1 = 'pi'; v1 = p1_0;
f2 = 'wi'; v2 = w1_0;
f3 = 'g0'; v3 = g_s_li1_0;
f4 = 'Cg'; v4 = g_s_lk1_0; % only for this, because the first!
f5 = 'expi'; v5 = exp1;
f6 = 'xi'; v6 = x1;
f7 = 'Sframe'; v7 = zeros(4);
s1 = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6,f7,v7);
% For second string part a check if '010' or '0110' takes place
% f5-f6-f7 are only initialization since 10,110 never start! Values are
% given ONLY inside functions
f1 = 'pi'; v1 = pi_10; 
f2 = 'wi'; v2 = wi_10;
f3 = 'g0'; v3 = g0_10;
f4 = 'Cg'; v4 = g_lk_TOOL_10;
f5 = 'expi'; v5 = exp1; 
f6 = 'xi'; v6 = x1;
f7 = 'Sframe'; v7 = zeros(4);
s2 = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6,f7,v7);
% For third string part a check if '10' or '110' takes place
% f5-f6-f7 are only initialization since 10,110 never start! Values are
% given ONLY inside functions
f1 = 'pi'; v1 = pi_110; 
f2 = 'wi'; v2 = wi_110;
f3 = 'g0'; v3 = g0_110;
f4 = 'Cg'; v4 = g_lk_TOOL_110;
f5 = 'expi'; v5 = exp1;
f6 = 'xi'; v6 = x1;
f7 = 'Sframe'; v7 = zeros(4);
s3 = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6,f7,v7);

%% Starts FKP solution

%% Active1

%% First structural block=structure(2)=SB01(only here) 
% Checks for first structural block
% It builds it, i.e. returns:
% 1. Active and Passive Joint Twists
% 2. If acive are s
switch structure(2,:) % WORKS for everything
    case 'xSB10'
        t10 = [1.5708 0]';
        n = 2; % Structural Block number
        R1 = [1 1 1]; % normally ga gives them
        P1 = [0.05 0 0.05]; % normally ga gives them
        [S2] = Build_SB10(s1,s2,R1,P1,n,point2figure,[t0; t10],x1);
    case 'SB110'
        t110 = [1.5708 1.55708 0]';
        n = 2; % Structural Block number
        R1 = [1 1 1]; % normally ga gives them
        P1 = [0.05 0 0.05]; % normally ga gives them
        [S2] = Build_SB110(s1,s3,R1,P1,n,point2figure,[t0; t110],x1);
    otherwise
        warning('Unexpected structural block entered!')
end
%% Second structural block=structure(3)=SB011(only here) 
% Checks for first structural block
% It builds it, i.e. returns:
% 1. Active and Passive Joint Twists
% 2. If acive are s
switch structure(3,:)
    case 'xSB10'
        t10 = [0 0]';
        n = 3; % Structural Block number
        R2 = [0 0 0]; % normally ga gives them
        P2 = [0 0 0]; % normally ga gives them
        [S3] = Build_SB10(S2,s2,R2,P2,n,point2figure,[t0; t10],x1);
    case 'SB110'
        t110 = [0 0 0]';
        n = 2; % Structural Block number
        R1 = [0 0 0]; % normally ga gives them
        P1 = [0 0 0]; % normally ga gives them
        [S3] = Build_SB110(S2,s3,R1,P1,n,point2figure,[t0; t110],x1);
    otherwise
        warning('Unexpected structural block entered!')
end

%% Form Metamorphic Manipulator Structure Kinematics and Jacobians
a=1;
