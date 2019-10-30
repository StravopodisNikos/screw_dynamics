function [DCI,E,P,D] = CalculateDynamicConditioningIndex2_3DoF(M,nDoF)
% Calculates Dynamic conditioning Index for serial manipulator
% A measure that quantifies how far is a given confguration from dynamic isotropy
%% Here definition 2 is approached!
% DCI is explained in paper: "The Concept of dynamic isotropy and its applications to inverse kinematics and trajectory planning"
% from Ou MA and Jorge Angeles, 1990
%% This approach is immune to matrix dimensions
% Matrix Diagonalization is used to visualize the physical meaning of DCI 

%% First the symmetric M matrix is diagonalized by finding the eigenvectors matrix P and the diagonal matrix of eigenvalues
[P,D] = eig(M);
% Now M is found by: M = P*D*P^(-1)
% and M^2 = P*D^2*P^(-1);
M2 = P*(D^2)*inv(P);
%% Sigma is a scalar defined such that the norm of difference matrix is minimum for fixed q
sigma = trace(M2)/trace(M); % 1x1
%% Get Difference Matrix between the generalized inertia matrix and its nearest isotropic matrix
E = (1/sigma)*(M - sigma.*eye(nDoF));
% mask = triu(true(size(D)));
e1 = diag(E); %3
e2 = E(1,2:3)'; %2
e3 = E(2,3)'; %1
e = vertcat(e1,e2,e3); % extrac n*(n+1)/2 vector
%% Define Weighting matrix depending on the user-desired relations between decoupling and isotropy
%% and the dimensional inhomogeneities of inertia matrix
w1 = [0 0 0 ]'; % Interested only for for dynamic decoupling
w2 = [0.1 0.1]';
w3 = 0.1;

scale_weights = 100; 
w = scale_weights*vertcat(w1,w2,w3);
W = diag(w);

DCI = 0.5*e'*W*e;

end