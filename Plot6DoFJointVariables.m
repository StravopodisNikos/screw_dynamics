function [status] = Plot6DoFJointVariables(figures,tvec,dy,DCI,Torques)

JointAngleFigure = figures(1);
JointVelocityFigure = figures(2);
JointTorqueFigure = figures(3);
DCIFigure = figures(4);

figure(JointAngleFigure);
% sgtitle('Joint Position')
axJA1 = subplot(6,1,1);
plot(axJA1,tvec,dy(:,1));
title(axJA1,'Joint1');
hold on;

axJA2 = subplot(6,1,2);
plot(axJA2,tvec,dy(:,2));
title(axJA2,'Joint2');
hold on;

axJA3 = subplot(6,1,3);
plot(axJA3,tvec,dy(:,3));
title(axJA3,'Joint3');
hold on;

axJA4 = subplot(6,1,4);
plot(axJA4,tvec,dy(:,4));
title(axJA4,'Joint4');
hold on;

axJA5 = subplot(6,1,5);
plot(axJA5,tvec,dy(:,5));
title(axJA5,'Joint5');
hold on;

axJA6 = subplot(6,1,6);
plot(axJA6,tvec,dy(:,6));
title(axJA6,'Joint6');
hold on;

figure(JointVelocityFigure);
% sgtitle('Joint Velocity')
axJV1 = subplot(6,1,1);
plot(axJV1,tvec,dy(:,7));
title(axJV1,'Joint1');
hold on;

axJV2 = subplot(6,1,2);
plot(axJV2,tvec,dy(:,8));
title(axJV2,'Joint2');
hold on;

axJV3 = subplot(6,1,3);
plot(axJV3,tvec,dy(:,9));
title(axJV3,'Joint3');
hold on;

axJV4 = subplot(6,1,4);
plot(axJV4,tvec,dy(:,10));
title(axJV4,'Joint4');
hold on;

axJV5 = subplot(6,1,5);
plot(axJV5,tvec,dy(:,11));
title(axJV5,'Joint5');
hold on;

axJV6 = subplot(6,1,6);
plot(axJV6,tvec,dy(:,12));
title(axJV6,'Joint6');
hold on;

figure(JointTorqueFigure);
% sgtitle('Joint Torque')
axJT1 = subplot(6,1,1);
plot(axJT1,tvec,Torques(1,:));
title(axJT1,'Joint1');
hold on;

axJT2 = subplot(6,1,2);
plot(axJT2,tvec,Torques(2,:));
title(axJT2,'Joint2');
hold on;

axJT3 = subplot(6,1,3);
plot(axJT3,tvec,Torques(3,:));
title(axJT3,'Joint3');
hold on;

axJT4 = subplot(6,1,4);
plot(axJT4,tvec,Torques(4,:));
title(axJT4,'Joint4');
hold on;

axJT5 = subplot(6,1,5);
plot(axJT5,tvec,Torques(5,:));
title(axJT5,'Joint5');
hold on;

axJT6 = subplot(6,1,6);
plot(axJT6,tvec,Torques(6,:));
title(axJT6,'Joint6');
hold on;

figure(DCIFigure);
plot(tvec,DCI);
title('DCI');
hold on;

status='plots ok';
end