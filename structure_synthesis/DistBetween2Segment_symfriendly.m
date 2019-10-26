function [distance varargout] = DistBetween2Segment_symfriendly(p1, p2, p3, p4)

    u = p1 - p2;
    v = p3 - p4;
    w = p2 - p4;
    
    a = dot(u,u);
    b = dot(u,v);
    c = dot(v,v);
    d = dot(u,w);
    e = dot(v,w);
    D = a*c - b*b;
    sD = D;
    tD = D;
        
    % compute the line parameters of the two closest points
    % get the closest points on the infinite lines
    sN = (b*e - c*d);
    tN = (a*e - b*d);

%     (sN > sD)% sc > 1 => the s=1 edge is visible
    sN = sD;
    tN = e + b;
    tD = c;

   
%     (tN > tD)       % tc > 1 => the t=1 edge is visible
        tN = tD;
        % recompute sc for this edge
 
        sN = (-d + b);
        sD = a;

    
    % finally do the division to get sc and tc

        sc = sN / sD;

    

        tc = tN / tD;

    
    % get the difference of the two closest points
    dP = w + (sc * u) - (tc * v);  % = S1(sc) - S2(tc)

    distance = norm(dP);
    outV = dP;
    
    varargout(1) = {outV};      % vector connecting the closest points
    varargout(2) = {p2+sc*u};   % Closest point on object 1 
    varargout(3) = {p4+tc*v};   % Closest point on object 2
    
end