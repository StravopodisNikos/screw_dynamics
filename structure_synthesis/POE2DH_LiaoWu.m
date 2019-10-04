function [DH_i_i1_eq] = POE2DH_LiaoWu(im1XIi,XIi)
% XI = [v w]' = [v1 v2 v3 w1 w2 w3]'
% XIi is the active joint twist mapped to the previous joint(i-1)
% im1XIi is the relative twist due to metamorphosis between two adjacent

% active joints i --> i+1
% Based on paper "An analytic Approach to converting POE parameters into D-H Parameters for serial link robots"
% Oct 2017
% Modified DH notation is used! NOT the standard model
% Modified DH model: im1Tx(Lim1)*im1Rx(aim1)*iTz(di)*iRz(thi) = im1Mi
%% Step 1. Create symbolic Tf Matrix im1Mi based on Aspragathos p.64
syms aim1 Lim1 di thi
im1Mi = [cos(thi) -sin(thi) 0 Lim1;...
         sin(thi)*cos(aim1) cos(thi)*cos(aim1) -sin(aim1) -sin(aim1)*di;...
         sin(thi)*sin(aim1) cos(thi)*sin(aim1)  cos(aim1)  cos(aim1)*di;...
         0                  0                   0                    1];

Ad_im1Mi = ad(im1Mi);
%% Step 2. Form equation of Lemma 2.1 that must be satisfied
% im1XIi = Ad(im1Mi)*xi_Rz
A1 = Ad_im1Mi*XIi;

%% Step 3. System solution
A2 = im1XIi - A1;
DH_i_i1_eq(1) = A2(1)==0;
DH_i_i1_eq(2) = A2(2)==0;
DH_i_i1_eq(3) = A2(3)==0;
DH_i_i1_eq(4) = A2(4)==0;
DH_i_i1_eq(5) = A2(5)==0;
DH_i_i1_eq(6) = A2(6)==0;
end