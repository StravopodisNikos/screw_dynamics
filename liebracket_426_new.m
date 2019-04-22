function [liebracket] = liebracket_426_new(XIi, XIj)
w_i = XIi(4:6);
wi_hat = [0 -w_i(3) w_i(2); w_i(3) 0 -w_i(1); -w_i(2) w_i(1) 0];
XIi_hat = [wi_hat, [0 0 0]'; 0 0 0 0 ];

w_j = XIj(4:6);
wj_hat = [0 -w_j(3) w_j(2); w_j(3) 0 -w_j(1); -w_j(2) w_j(1) 0];
XIj_hat = [wj_hat, [0 0 0]'; 0 0 0 0 ];

cross_se3 = XIi_hat*XIj_hat - XIj_hat*XIi_hat;

liebracket(1:3) = cross_se3(1:3,4);
liebracket(4:6) = [cross_se3(3,2) cross_se3(1,3) cross_se3(2,1)];
liebracket = liebracket';
end