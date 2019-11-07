function [a] = McoresEvaluation(Ms, Mb ,Js ,Jb , qs, qdot)
%% Examination of the eigen-basis of the cores builts from Manipulator 
%% Generalized Inertia Matrix
% First the M matrix is calculated...

%% Dynamic Isotropy
%% Always working with respect to Body Frame for Dynamic Isotropy?
%% 1. Mass Matrix-Inertia Ellipsoid
% M is calculated at the links' body frames=>Should I transform to body
% frame?
[P1s,D1s] = eig(Ms);
[U1s,S1s,V1s] = svd(Ms);

[P1b,D1b] = eig(Mb);
[U1b,S1b,V1b] = svd(Mb);
% Now M is found by: M = P*D*P^(-1), P is the rotation matrix of inertia
% ellipsoid, D is the diagonal matrix with corresponding eigenvalues

%% 2. Generalized Inertia Matrix by Asada [GIE]
% First kinetic energy of the manipulator is found
GIE = (qdot'*Mb*qdot)/(qdot'*qdot);
%% 2. Acceleration Ellipsoid
trq_u = [44.7 25.3 25.3]'; % MAX for Dynamixel motors to be used
L = diag(trq_u);
qb = zeros(3,1); % for TCP in Tool Frame => Prepei na to allaksw!!!
[J33b_new] = CalculateSquareTCPJacobian_3DoF(Jb,qb);
[J33s_new] = CalculateSquareTCPJacobian_3DoF(Js,qs);
% Qb = inv(J33b_new')*Mb'*inv(L')*inv(L)*Mb*inv(J33b_new);
Qb = inv(J33b_new')*Mb'*L^2*Mb*inv(J33b_new);
Qb2 = inv(J33b_new')*Mb'*Mb*inv(J33b_new);
[P2b,D2b] = eig(Qb);
[U2b,S2b,V2b] = svd(Qb);
[P2b2,D2b2] = eig(Qb2);
[U2b2,S2b2,V2b2] = svd(Qb2);

% Qs = inv(J33s_new')*Ms'*inv(L')*inv(L)*Ms*inv(J33s_new);
Qs = inv(J33s_new')*Ms'*L^2*Ms*inv(J33s_new);
Qs2 = inv(J33s_new')*Ms'*Ms*inv(J33s_new);
%Q=                  *======D=======*
[P2s,D2s] = eig(Qs);
[U2s,S2s,V2s] = svd(Qs);
[P2s2,D2s2] = eig(Qs2);
[U2s2,S2s2,V2s2] = svd(Qs2);
%% Kinematic Isotropy
Ks = J33s_new*J33s_new';
[P3s,D3s] = eig(Ks);
[U3s,S3s,V3s] = svd(Ks);

Kb = J33b_new*J33b_new';
[P3b,D3b] = eig(Kb);
[U3b,S3b,V3b] = svd(Kb);

wk = 0.5;
wd = 0.5;

f1s = wk*(S3s(1,1)/S3s(3,3)) + wd*(S1s(1,1)/S1s(3,3)); % Ks & Ms
f1b = wk*(S3b(1,1)/S3b(3,3)) + wd*(S1b(1,1)/S1b(3,3)); % Kb & Mb

f2s = wk*(S3s(1,1)/S3s(3,3)) + wd*(S2s(1,1)/S2s(3,3)); % Ks & Qs
f2b = wk*(S3b(1,1)/S3b(3,3)) + wd*(S2b(1,1)/S2b(3,3)); % Kb & Qb

f3s2 = wk*(S3s(1,1)/S3s(3,3)) + wd*(S2s2(1,1)/S2s2(3,3)); % Ks & Qs2
f3b2 = wk*(S3b(1,1)/S3b(3,3)) + wd*(S2b2(1,1)/S2b2(3,3)); % Kb & Qb2
end