function dotJb5 = dotJb5(qdot,Jb)
% Calculates the first time derivative of Body Jacobian for 6 DoF serial
% manipulator

xib1 = Jb(:,1);
xib2 = Jb(:,2);
xib3 = Jb(:,3);
xib4 = Jb(:,4);
xib5 = Jb(:,5);
xib6 = Jb(:,6);

dxib1 = dotxib1(qdot,xib1,xib2,xib3,xib4,xib5,xib6);
dxib2 = dotxib2(qdot,xib2,xib3,xib4,xib5,xib6);
dxib3 = dotxib3(qdot,xib3,xib4,xib5,xib6);
dxib4 = dotxib4(qdot,xib4,xib5,xib6);
dxib5 = dotxib5(qdot,xib5,xib6);
dxib6 = [0 0 0 0 0 0]';

dotJb5 = [dxib1 dxib2 dxib3 dxib4 dxib5 dxib6];
end