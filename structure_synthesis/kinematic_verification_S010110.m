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

%% load zero data for STRUCTURAL BLOCKS
% data are obtained by .m files:
% modular_dynamixel/structural_synthesis/kinematic_verification_C01.m for SB01
% modular_dynamixel/structural_synthesis/kinematic_verification_C011.m for SB011
load('SB10_zero_data.mat');
pi_10 = pi(:,2:4); % j-i1-k(!)
wi_10 = wi(:,2:3); % j-i1
g0_10 = g0(:,:,1:8);
gsc_10 = g0(:,:,9); % g_s_lk0
load('SB110_zero_data.mat');
pi_110 = pi(:,2:5); % j1-j2-i1-k(!)
wi_110 = wi(:,2:4); % j1-j2-i1
g0_110 = g0;
gsc_110 = g0(:,:,2); % g_s_lk0
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
structure(3,:) = 'SB110'; % Normally this is extracted by ga 
%% END OF MANUAL INTERFERENCE %%
% First is always a lonely active joint at z axis rotation
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
field1 = 'pi'; value1 = p1_0;
field2 = 'wi'; value2 = w1_0;
field3 = 'g0'; value3 = g_s_li1_0;
field4 = 'Cg'; value4 = g_s_lk1_0;
s1 = struct(field1,value1,field2,value2,field3,value3,field4,value4);

% For second string part a check if '010' or '0110' takes place
field1 = 'pi'; value1 = pi_10; 
field2 = 'wi'; value2 = wi_10;
field3 = 'g0'; value3 = g0_10;
field4 = 'Cg'; value4 = gsc_10;% the previous gsC {s} - > Connection Point (where P1a connects with previous frame)
s2 = struct(field1,value1,field2,value2,field3,value3,field4,value4);
% For third string part a check if '10' or '110' takes place
field1 = 'pi'; value1 = pi_110; 
field2 = 'wi'; value2 = wi_110;
field3 = 'g0'; value3 = g0_110;
field4 = 'Cg'; value4 = gsc_110;% the previous gsC {s} - > Connection Point (where P1a connects with previous frame)
s3 = struct(field1,value1,field2,value2,field3,value3,field4,value4);


%% Starts FKP solution

%% Active1

%% First structural block=structure(2)=SB01(only here) 
% Checks for first structural block
% It builds it, i.e. returns:
% 1. Active and Passive Joint Twists
% 2. If acive are s
switch structure(2,:)
    case 'xSB10'
        n = 2; % Structural Block number
        XIim1 = x1;
        R1 = [0 0 0]; % normally ga gives them
        P1 = [0 0 0]; % normally ga gives them
        [S2] = Build_SB10(s1,s2,R1,P1,XIim1,n);
    case 'SB110'
%         [] = Build_SB110(s0,s1,R1,P1)
    otherwise
        warning('Unexpected structural block entered!')
end
%% Second structural block=structure(3)=SB011(only here) 
% Checks for first structural block
% It builds it, i.e. returns:
% 1. Active and Passive Joint Twists
% 2. If acive are s
switch structure(3)
    case 'xSB10'
        R2 = [0 0 0]; % normally ga gives them
        P2 = [0 0 0]; % normally ga gives them
        [Si] = Build_SB10(S2,s3,R2,P2,XIim1,n)
    case 'SB110'
        n = 3; % Structural Block number
        
    otherwise
        warning('Unexpected structural block entered!')
end

%% Form Metamorphic Manipulator Structure Kinematics and Jacobians

