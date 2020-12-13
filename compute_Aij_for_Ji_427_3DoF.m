function [Aij_427] = compute_Aij_for_Ji_427_3DoF(exp_ai, Pi, i, j)

Aij_427 = zeros(6);

if i==1
    if j==1 % i=j
        Aij_427 = eye(6);
    elseif j==2 % i<j
        Aij_427 = zeros(6);
    elseif j==3 % i<j
        Aij_427 = zeros(6);      
    end
elseif i==2
    if j==1 % j<i
        Aij_427 = inv(ad(Pi(:,:,1)*exp_ai(:,:,i)));
    elseif j==2 % i=j
        Aij_427 = eye(6);
    elseif j==3 % i<j
        Aij_427 = zeros(6);
    end
elseif i==3
    if j==1 % j<i
        Aij_427 = inv(ad(Pi(:,:,1)*exp_ai(:,:,j+1)*Pi(:,:,2)*exp_ai(:,:,i)));
    elseif j==2 % i=j
        Aij_427 = inv(ad(Pi(:,:,2)*exp_ai(:,:,j+1)));
    elseif j==3 % i<j
        Aij_427 = eye(6);
    end

else
  no_3DoF = 1;  
end

end