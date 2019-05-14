function [maxVm] = CalculateMaximumAnatomyWsVolume(minLm)
% minLm is minimum sum of link lengths of current anatomy
% calculated in CalculateMinimumAnatomyLinkLength.m
maxVm = (4*pi/3)*(2*pi*minLm)^3;
end