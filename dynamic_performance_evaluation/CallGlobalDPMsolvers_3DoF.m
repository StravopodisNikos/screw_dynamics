function [x,fval,exitflag,output] = CallGlobalDPMsolvers_3DoF(xi_ai, xi_pi, gst0, gsli0, M0b_CoM)


%% Uncomment desired call for solver

%% Optimization functions & options for single anatomy
% FitnessFunction = @(x)MinAccelerationRadius1(x,tpi1, xi_ai, xi_pi, Pi_test, gst0, gsli0, M0b_CoM);
% ConstraintFunction = @(x)ConAccelerationRadius11(x,tpi1, xi_ai, xi_pi, Pi_test, gst0, gsli0, M0b_CoM);
% nvars = 4; % q2,3 qd2,3
% LB = [-1.57 -1.57 -3.3 -3.3];
% UB = [1.57 1.57 3.3 3.3];

%% Optimization functions for anatomy determination
%% 1 for Acceleration Radius
FitnessFunction1 = @(x)MinAccelerationRadius12_3DoF(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);
ConstraintFunction1 = @(x)ConAccelerationRadius112_3DoF(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);
nvars1 = 10; 
LB1 = [-1.57 -1.57 -1.57 -3.3 -3.3 -3.3 1 1 1 1 ];
UB1 = [1.57 1.57 1.57 3.3 3.3 3.3 5 5 5 5];
%% 2 for DCI
% FitnessFunction2a = @(x)MinDCI_3DoF(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);
FitnessFunction2b = @(x)GlobalMinDCI1_3DoF(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);
nvars2 = 7; % q1,2,3 tpi
% LB2a = [-1.57 -1.57 -1.57 1 1 1 1];
% UB2a = [1.57 1.57 1.57 5 5 5 5];
LB2b = [0.2 0.2 0.2 1 1 1 1];
UB2b = [0.3 0.3 0.3 5 5 5 5];
%% 3 for Wd
% FitnessFunction3 = @(x)MaxWd_3DoF(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);
% nvars3 = 7; % q2,3 tpi
% LB3 = [-1.57 -1.57 -1.57 1 1 1 1];
% UB3 = [1.57 1.57 1.57 5 5 5 5];
%% 4 for GDII
FitnessFunction4 = @(x)GlobalDynamicIsotropyIndex_3DoF(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);
ConstraintFunction4 = @(x)ConGlobalDynamicIsotropyIndex_3DoF(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);

nvars4 = 7; % q2,3 tpi
LB4 = [-1.57 -1.57 -1.57 1 1 1 1 ];
UB4 = [1.57 1.57 1.57 5 5 5 5 ];
%% 5 for GlobalMaxWd
FitnessFunction5 = @(x)GlobalMaxWd_3DoF(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);
ConstraintFunction5 = @(x)ConGlobalMaxWd_3DoF(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM);

nvars5 = 6; % step_a2,3 - tpi
LB5 = [0.2 0.2 1 1 1 1];
UB5 = [0.3 0.3 5 5 5 5];
%% 6 for GlobalMinDCI - gamultiobj doesn't accept integer constraints!!!!
FitnessFunction6 = @(x)[GlobalMinDCI1_3DoF(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM),GlobalMinDCI2_3DoF(x, xi_ai, xi_pi, gst0, gsli0, M0b_CoM)];
ConstraintFunction6 = @(x)ConGlobalMinDCI_3DoF(x, xi_ai, xi_pi, gst0,gsli0,M0b_CoM);  

nvars6 = 9; % step_a1,2,3 - tpi
LB6 = [ 0.2 0.2 1 1 1 1 0 0 0];
UB6 = [ 0.3 0.3 5.999 5.999 5.999 5.999 10 10 10];
%% GA calls for single anatomy
% tic
% options = optimoptions('ga','Generations',200,'PopulationSize',100,'Display','iter','MutationFcn',{'mutationadaptfeasible'},'StallGenLimit',20);
% options = optimoptions('ga','Generations',400,'PopulationSize',10,'Display','iter','MutationFcn',{'mutationadaptfeasible'},'StallGenLimit',50,'TolCol', 1e-6'UseParallel', true, 'UseVectorized', false);
%
% [x,fval,exitflag,output] = ga(FitnessFunction,nvars,[],[],[],[],LB,UB,ConstraintFunction,options);
% [x,fval,exitflag,output] = ga(FitnessFunction,nvars,[],[],[],[],LB,UB,[],[],options);
% toc
%% GA calls for anatomy determination
%% 1 is global measure
% IntCon1 = [7,8,9,10];
% options = optimoptions('ga','Generations',500,'PopulationSize',30,'Display','iter','StallGenLimit',50,'UseParallel', true, 'UseVectorized', false);
% tic
% [x,fval,exitflag,output] = ga(FitnessFunction1,nvars1,[],[],[],[],LB1,UB1,ConstraintFunction1,IntCon1,options);
% toc
%% 2+3 are local measures
% IntCon23 = [4,5,6,7];
% options = optimoptions('ga','Generations',100,'PopulationSize',100,'Display','iter','StallGenLimit',50,'UseParallel', true, 'UseVectorized', false);
% tic
% [x,fval,exitflag,output] = ga(FitnessFunction2b,nvars2,[],[],[],[],LB2b,UB2b,[],IntCon23,options);
% toc
%% 4 is global measures
% IntCon4 = [4,5,6,7];
% options = optimoptions('ga','Generations',200,'PopulationSize',500,'Display','iter','StallGenLimit',100,'UseParallel', true, 'UseVectorized', false);
% tic
% [x,fval,exitflag,output] = ga(FitnessFunction4,nvars4,[],[],[],[],LB4,UB4,ConstraintFunction4,IntCon4,options);
% toc
%% 5 is global measure
% IntCon = [3,4,5,6];
% options = optimoptions('ga','Generations',200,'PopulationSize',100,'Display','iter','FunctionTolerance',1e-6,'StallGenLimit',50,'UseParallel', true, 'UseVectorized', false);
% tic
% [x,fval,exitflag,output] = ga(FitnessFunction5,nvars5,[],[],[],[],LB5,UB5,ConstraintFunction5,IntCon,options);
% toc
%% 6 is global measure
IntCon6 = [4,5,6,7];
options = optimoptions('gamultiobj','Generations',100,'PopulationSize',100,'Display','iter','StallGenLimit',25,'UseParallel', true, 'UseVectorized', false);
tic
[x,fval,exitflag,output] = gamultiobj(FitnessFunction6,nvars6,[],[],[],[],LB6,UB6,ConstraintFunction6,options);
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