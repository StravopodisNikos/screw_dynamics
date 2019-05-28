function dotxi = dotxi(xi,v_qi)
    dot_xi(1:3,:) = cross(-xi(4:6),v_qi(1:3)); 
    dot_xi(4:6,:) = [0 0 0]';
    
    
    dotxi = dot_xi;
    
end