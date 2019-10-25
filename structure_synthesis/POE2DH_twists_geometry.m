function [DH] = POE2DH_twists_geometry(gai,grel,Js)
% works for 3 DOF metamorphic manipulators given from
% structure_assembly_GA.m
% OUTPUT: DH 3x4 matrix

%% Draw POE frames
DH_figure = figure;
figure(DH_figure);
drawframe(gai(:,:,1), 0.05); hold on; % ga1
text(gai(1,4,1), gai(2,4,1), gai(3,4,1),'\leftarrow g_{sl1}'), hold on;
drawframe(gai(:,:,2), 0.05); hold on; % ga2
text(gai(1,4,2), gai(2,4,2), gai(3,4,2),'\leftarrow g_{sl2}'), hold on;
drawframe(gai(:,:,3), 0.05); hold on; % ga3
text(gai(1,4,3), gai(2,4,3), gai(3,4,3),'\leftarrow g_{sl3}'), hold on;
drawframe(gai(:,:,4), 0.05); hold on; % ga3
text(gai(1,4,4), gai(2,4,4), gai(3,4,4),'\leftarrow g_{sTOOL}'), hold on;
%% Draw Twists
x1_graph = drawtwist(Js(:,1)); hold on;
x2_graph = drawtwist(Js(:,2)); hold on;
x3_graph = drawtwist(Js(:,3)); hold on;
% [xTOOLabs, thTOOL_abs] = homtotwist(gai(:,:,4));
% x4_graph = drawtwist(xTOOLabs*thTOOL_abs); hold on;
p1_1 = [x1_graph.XData(1) x1_graph.YData(1) x1_graph.ZData(1) ]';
p1_2 = [x1_graph.XData(2) x1_graph.YData(2) x1_graph.ZData(2) ]';
text(p1_2(1), p1_2(2), p1_2(3),'\leftarrow ξ_1'), hold on;
p2_1 = [x2_graph.XData(1) x2_graph.YData(1) x2_graph.ZData(1) ]';
p2_2 = [x2_graph.XData(2) x2_graph.YData(2) x2_graph.ZData(2) ]';
text(p2_2(1), p2_2(2), p2_2(3),'\leftarrow ξ_2'), hold on;
p3_1 = [x3_graph.XData(1) x3_graph.YData(1) x3_graph.ZData(1) ]';
p3_2 = [x3_graph.XData(2) x3_graph.YData(2) x3_graph.ZData(2) ]';
text(p3_2(1), p3_2(2), p3_2(3),'\leftarrow ξ_3'), hold on;
% p4_1 = [x4_graph.XData(1) x4_graph.YData(1) x4_graph.ZData(1) ]';
% p4_2 = [x4_graph.XData(2) x4_graph.YData(2) x4_graph.ZData(2) ]';
% text(p4_2(1), p4_2(2), p4_2(3),'\leftarrow ξ_{TOOL}'), hold on;

%% Find common normals
[L12, d12, S1n, S2] = DistBetween2Segment(p1_1, p1_2, p2_1, p2_2); 
plot3([S1n(1) S2(1)],[S1n(2) S2(2)],[S1n(3) S2(3)]), hold on
text(S1n(1), S1n(2), S1n(3),'\leftarrow S_{1n}'), hold on;
text(S2(1), S2(2), S2(3),'\leftarrow S_{2}'), hold on;
[L23, d23, S2n, S3] = DistBetween2Segment(p2_1, p2_2, p3_1, p3_2); 
plot3([S2n(1) S3(1)],[S2n(2) S3(2)],[S2n(3) S3(3)]), hold on
text(S2n(1), S2n(2), S2n(3),'\leftarrow S_{2n}'), hold on;
text(S3(1), S3(2), S3(3),'\leftarrow S_{3}'), hold on;


%% Build DH frames
x12 = -d12/norm(d12);
z12 = Js(4:6,1);
y12 = cross(z12,x12); % k^ x i^ = j^
T12 = [x12 y12 z12 S1n; 0 0 0 1];
figure(DH_figure);
drawframe(T12, 0.05, true); hold on; % DH12

x23 = -d23/norm(d23);
z23 = Js(4:6,2);
y23 = cross(z23,x23); % k^ x i^ = j^
T23 = [x23 y23 z23 S2n; 0 0 0 1];
figure(DH_figure);
drawframe(T23, 0.05, true); hold on; % DH23

x3TOOL = gai(1:3,3,4);
z3TOOL = Js(4:6,3);
y3TOOL = cross(z3TOOL,x3TOOL); % k^ x i^ = j^
T3TOOL = [x3TOOL y3TOOL z3TOOL S3; 0 0 0 1];
figure(DH_figure);
drawframe(T3TOOL, 0.05, true); hold on; % DH23

%% Find Correction matrices
C1 = inv(T12)*gai(:,:,1);
C2 = inv(T23)*gai(:,:,2);
C3 = inv(T3TOOL)*gai(:,:,3);

%% Calculate DH tfs
M01 = eye(4)*grel(:,:,1)*inv(C1); %ok
M12 = C1*grel(:,:,2)*inv(C2); %ok
M23 = C2*grel(:,:,3)*inv(C3); %ok
% DH = gai(:,:,3) - M01*M12*M23*C3; % for evaluation

[L01,a01,d01,th_star01] = CalculateModifiedDHparams(M01);
[L12,a12,d12,th_star12] = CalculateModifiedDHparams(M12);
[L23,a23,d23,th_star23] = CalculateModifiedDHparams(M23);

DH(1,:) = [L01,a01,d01,th_star01];
DH(2,:) = [L12,a12,d12,th_star12];
DH(3,:) = [L23,a23,d23,th_star23];
end