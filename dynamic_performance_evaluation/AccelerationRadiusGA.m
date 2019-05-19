function [x,fval,exitflag,output] = AccelerationRadiusGA(tp1, xi_ai, xi_pi, Pi_test, gst0, gsli0, M0b_CoM)
% Workspace Specification for single anatomy
% For given tpi, we find the area inside configuration space that, a (high)
% minimum number of TCP acceleration is achieved
% tpi is global in order to be passed inside Objective, Constraint functions

%% Uncomment desired call for solver

%% Optimization functions & options for single anatomy
% FitnessFunction = @(x)MinAccelerationRadius1(x,tpi1, xi_ai, xi_pi, Pi_test, gst0, gsli0, M0b_CoM);
% ConstraintFunction = @(x)ConAccelerationRadius11(x,tpi1, xi_ai, xi_pi, Pi_test, gst0, gsli0, M0b_CoM);
% nvars = 4; % q2,3 qd2,3
% LB = [-1.57 -1.57 -3.3 -3.3];
% UB = [1.57 1.57 3.3 3.3];

%% Optimization functions for anatomy determination
%% 1 for Acceleration Radius
% FitnessFunction = @(x)MinAccelerationRadius12(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);
% ConstraintFunction = @(x)ConAccelerationRadius112(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);
% nvars = 10; % q2,3 qd2,3
% LB = [-1.57 -1.57 -3.3 -3.3 1 1 1 1 1 1];
% UB = [1.57 1.57 3.3 3.3 5 5 5 5 5 5];
%% 2 for DCI
% FitnessFunction = @(x)MinDCI(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);
% nvars = 8; % q2,3 tpi
% LB = [-1.57 -1.57 1 1 1 1 1 1];
% UB = [1.57 1.57 5 5 5 5 5 5];
%% 3 for Wd
FitnessFunction = @(x)MaxWd(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);
nvars = 8; % q2,3 tpi
LB = [-1.57 -1.57 1 1 1 1 1 1];
UB = [1.57 1.57 5 5 5 5 5 5];
%% GA calls for single anatomy
% tic
% options = optimoptions('ga','Generations',200,'PopulationSize',100,'Display','iter','MutationFcn',{'mutationadaptfeasible'},'StallGenLimit',20);
% options = optimoptions('ga','Generations',400,'PopulationSize',10,'Display','iter','MutationFcn',{'mutationadaptfeasible'},'StallGenLimit',50,'TolCol', 1e-6'UseParallel', true, 'UseVectorized', false);
%
% [x,fval,exitflag,output] = ga(FitnessFunction,nvars,[],[],[],[],LB,UB,ConstraintFunction,options);
% [x,fval,exitflag,output] = ga(FitnessFunction,nvars,[],[],[],[],LB,UB,[],[],options);
% toc
%% GA calls for anatomy determination
%% 1
% IntCon = [5,6,7,8,9,10];
% options = optimoptions('ga','Generations',400,'PopulationSize',100,'Display','iter','StallGenLimit',50,'UseParallel', true, 'UseVectorized', false);
% tic
% [x,fval,exitflag,output] = ga(FitnessFunction,nvars,[],[],[],[],LB,UB,ConstraintFunction,IntCon,options);
% toc
%% 2+3
IntCon = [3,4,5,6,7,8];
options = optimoptions('ga','Generations',400,'PopulationSize',400,'Display','iter','StallGenLimit',50,'UseParallel', true, 'UseVectorized', false);
tic
[x,fval,exitflag,output] = ga(FitnessFunction,nvars,[],[],[],[],LB,UB,[],IntCon,options);
toc
%% fmincon - local minimum danger
% x0 = [0.5 -0.5 -0.5 -0.5];
% options = optimoptions('fmincon','UseParallel',true);
% tic
% [x,fval,exitflag,output] = fmincon(FitnessFunction,x0,[],[],[],[],LB,UB,ConstraintFunction,options);
% toc
%% fmincon with global search - needs HUGE time
% rng default % For reproducibility
% gs = GlobalSearch;
% options = optimoptions('fmincon','UseParallel',true);
% problem = createOptimProblem('fmincon','objective',FitnessFunction,'x0',x0,'lb',LB,'ub',UB,'nonlcon',ConstraintFunction,'options',options);
% tic
% [x,fval,exitflag,output] = run(gs,problem)
% toc