function [pi,gst] = ExtractGeometryPoints(robot,config)
%% works only for 6DoF MMD!

scale = 0.025; % for drawframe axes only

%% Get tfs of bodies of manipulator structure
base_link = char(robot.BodyNames(1));
Ts1 = getTransform(robot,config,base_link);
% drawframe(Ts1, scale) 
% hold on;
% text(Ts1(1,4),Ts1(2,4),Ts1(3,4),'\leftarrow base_link');
adaptor = char(robot.BodyNames(2));
Ts2 = getTransform(robot,config,adaptor);
frame = char(robot.BodyNames(3));
Ts3 = getTransform(robot,config,frame);
PseudoConnector1a = char(robot.BodyNames(4));
Ts4 = getTransform(robot,config,PseudoConnector1a);
PseudoConnector1b = char(robot.BodyNames(5));
Ts5 = getTransform(robot,config,PseudoConnector1b);
PseudoConnector2a = char(robot.BodyNames(6));
Ts6 = getTransform(robot,config,PseudoConnector2a);
PseudoConnector2b = char(robot.BodyNames(7));
Ts7 = getTransform(robot,config,PseudoConnector2b);
adaptor1 = char(robot.BodyNames(8));
Ts8 = getTransform(robot,config,adaptor1);
AM1 = char(robot.BodyNames(9));
Ts9 = getTransform(robot,config,AM1);
frame1 = char(robot.BodyNames(10));
Ts10 = getTransform(robot,config,frame1);
idler1 = char(robot.BodyNames(11));
Ts11 = getTransform(robot,config,idler1);
PseudoConnector3a = char(robot.BodyNames(12));
Ts12 = getTransform(robot,config,PseudoConnector3a);
PseudoConnector3b = char(robot.BodyNames(13));
Ts13 = getTransform(robot,config,PseudoConnector3b);
PseudoConnector4a = char(robot.BodyNames(14));
Ts14 = getTransform(robot,config,PseudoConnector4a);
PseudoConnector4b = char(robot.BodyNames(15));
Ts15 = getTransform(robot,config,PseudoConnector4b);
adaptor2 = char(robot.BodyNames(16));
Ts16 = getTransform(robot,config,adaptor2);
AM2 = char(robot.BodyNames(17));
Ts17 = getTransform(robot,config,AM2);
frame2 = char(robot.BodyNames(18));
Ts18 = getTransform(robot,config,frame2);
idler2 = char(robot.BodyNames(19));
Ts19 = getTransform(robot,config,idler2);
PseudoConnector5a = char(robot.BodyNames(20));
Ts20 = getTransform(robot,config,PseudoConnector5a);
PseudoConnector5b = char(robot.BodyNames(21));
Ts21 = getTransform(robot,config,PseudoConnector5b);
PseudoConnector6a = char(robot.BodyNames(22));
Ts22 = getTransform(robot,config,PseudoConnector6a);
PseudoConnector6b = char(robot.BodyNames(23));
Ts23 = getTransform(robot,config,PseudoConnector6b);
casebody4 = char(robot.BodyNames(24));
Ts24 = getTransform(robot,config,casebody4);
wrist_link1 = char(robot.BodyNames(25));
Ts25 = getTransform(robot,config,wrist_link1);
shaft45 = char(robot.BodyNames(26));
Ts26 = getTransform(robot,config,shaft45);
case45 = char(robot.BodyNames(27));
Ts27 = getTransform(robot,config,case45);
wrist_link2 = char(robot.BodyNames(28));
Ts28 = getTransform(robot,config,wrist_link2);
case56a = char(robot.BodyNames(29));
Ts29 = getTransform(robot,config,case56a);
case56b = char(robot.BodyNames(30));
Ts30 = getTransform(robot,config,case56b);
case56c = char(robot.BodyNames(31));
Ts31 = getTransform(robot,config,case56c);
wrist_link3 = char(robot.BodyNames(32));
Ts32 = getTransform(robot,config,wrist_link3);
TOOL = char(robot.BodyNames(33));
Ts33 = getTransform(robot,config,TOOL);

%% Extract geometry points of structure
pi(:,1)= Ts5(1:3,4); % p1
pi(:,2)= Ts7(1:3,4); % p2
pi(:,3)= Ts10(1:3,4); % p3
pi(:,4)= Ts13(1:3,4); % p4
pi(:,5)= Ts15(1:3,4); % p5
pi(:,6)= Ts18(1:3,4); % p6
pi(:,7)= Ts21(1:3,4); % p7
pi(:,8)= Ts23(1:3,4); % p8
pi(:,9)= Ts24(1:3,4); % TCP
pi(:,10)= Ts26(1:3,4); % wp1
pi(:,11)= Ts30(1:3,4); % wp2
pi(:,12)= Ts33(1:3,4); % wp3

gst = Ts33;
end