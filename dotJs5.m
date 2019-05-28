function dotJs5 = dotJs5(qdot,Js)
% Calculates the first time derivative of Spatial Jacobian for 6 DoF serial
% manipulator

xi1 = Js(1,:);
xi2 = Js(2,:);
xi3 = Js(3,:);
xi4 = Js(4,:);
xi5 = Js(5,:);
xi6 = Js(6,:);

dxi1 = [0 0 0 0 0 0]';
dxi2 = dotxi2(qdot,xi1,xi2);
dxi3 = dotxi3(qdot,xi1,xi2,xi3);
dxi4 = dotxi4(qdot,xi1,xi2,xi3,xi4);
dxi5 = dotxi5(qdot,xi1,xi2,xi3,xi4,xi5);
dxi6 = dotxi6(qdot,xi1,xi2,xi3,xi4,xi5,xi6);

dotJs5 = [dxi1 dxi2 dxi3 dxi4 dxi5 dxi6];
end