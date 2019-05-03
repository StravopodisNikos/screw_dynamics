function [Wd] = calculateDynamicManipulabilityIndex(J,M)
    % Calculates dynamic manipulablity Indec for non-redundant manipulators
    % at the given posture(configuration)
    Wd = abs(det(J)/det(M));
end