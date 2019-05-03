function [r,MR] = CalculateAccelerationRadius(J0,M0,J,M,TorqueLim)
% Calculates AccelerationRadiusIndex(r) and Maneuverability Ratio(MR)
% as described in Evaluation of the dynamic performance variation of a
% serial manipulator after eliminating the self-weight influence, Pi-Ying Chen &
% Kuei-Jen Chen
% works for non-redundant manipulators

% TorqueLim is a nx1 vector of upper joint torque limits
% J0,M0 for reference anatomy for specified point
% J,M for current configuration in new anatomy
L = diag(TorqueLim);
r0 = abs(det(J0*L)/det(M0));
r = abs(det(J*L)/det(M));

% test
Q0 = J0*inv(M0)*L*L'*inv(M0')*J0';
r02 = sqrt(abs(det(Q0)));
Q = J*inv(M)*L*L'*inv(M')*J';
r2 = sqrt(abs(det(Q)));

MR = (r - r0)/r0;


end