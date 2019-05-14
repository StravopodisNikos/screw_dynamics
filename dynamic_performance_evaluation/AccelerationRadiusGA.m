function [x,fval,exitflag,output] = AccelerationRadiusGA(a)
% Workspace Specification for single anatomy
% For given tpi, we find the area inside configuration space that, a (high)
% minimum number of TCP acceleration is achieved
% tpi is global in order to be passed inside Objective, Constraint functions

% Optimization functions
FitnessFunction = @MinAccelerationRadius1;
ConstraintFunction = @ConAccelerationRadius11;
% nvars = 12; % [q; qd]
% LB = [0 -1.57 -1.57 0 0 0 -1 -1 -1 -1 -1 -1];
% UB = [0.001 1.57 1.57 0.001 0.001 0.001 1 1 1 1 1 1];
nvars = 4; % q2,3 qd2,3
LB = [-1.57 -1.57 -3.3 -3.3];
UB = [1.57 1.57 3.3 3.3];
% options = optimoptions('ga','Generations',400,'PopulationSize',10,'Display','iter','MutationFcn',{'mutationadaptfeasible'},'StallGenLimit',50,'TolCol', 1e-6'UseParallel', true, 'UseVectorized', false);
%% GA
% options = optimoptions('ga','Generations',200,'PopulationSize',100,'Display','iter','MutationFcn',{'mutationadaptfeasible'},'StallGenLimit',20);
% [x,fval,exitflag,output] = ga(FitnessFunction,nvars,[],[],[],[],LB,UB,ConstraintFunction,options);
% [x,fval,exitflag,output] = ga(FitnessFunction,nvars,[],[],[],[],LB,UB,[],[],options)
%% fmincon: 10000000x faster
x0 = [0.5 -0.5 -0.5 -0.5];
[x,fval,exitflag,output] = fmincon(FitnessFunction,x0,[],[],[],[],LB,UB,ConstraintFunction);