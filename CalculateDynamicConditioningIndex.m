function [DCI] = CalculateDynamicConditioningIndex(M,nDoF)
% Calculates Dynamic conditioning Index for serial manipulator
% A measure that quantifies how far is a given confguration from dynaic
% isotropy
%% DCI is dimension and configuration dependent!!!

% M: Generalized Inertia Matrix of Manipulator

%% Sigma is a scalar defined such that the norm of difference matrix is minimum for fixed q
sigma = (1/nDoF)*trace(M); % 1x1
%% Get Difference Matrix between the generalized inertia matrix and its nearest isotropic matrix
D = M - sigma.*eye(6); 
% mask = triu(true(size(D)));
d1 = diag(D); % 6 elements
d2 = D(1,2:6)'; % 5 elements
d3 = D(2,3:6)'; % 4 elements
d4 = D(3,4:6)'; % 3 elements
d5 = D(4,5:6)'; % 2 elements
d6 = D(5,6)'; % 1
d = vertcat(d1,d2,d3,d4,d5,d6); % extrac n*(n+1)/2 vector
%% Define Weighting matrix depending on the user-desired relations between decoupling and isotropy
%% and the dimensional inhomogeneities of inertia matrix

w1 = [0 0 0 0 0 0 ]'; %
w2 = [0.001 0.001 0.001 0.001 0.001]';
w3 = [0.001 0.001 0.001 0.001]';
w4 = [0.001 0.001 0.001]';
w5 = [0.001 0.001]';
w6 = 0.001;
scale_weights = 1000; % for identity
w = scale_weights*vertcat(w1,w2,w3,w4,w5,w6);
W = diag(w);

DCI = 0.5*d'*W*d;

end
