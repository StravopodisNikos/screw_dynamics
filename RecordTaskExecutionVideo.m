function [video] = RecordTaskExecutionVideo(point2fig,robot,config)
% Saves to working directory a video file of P2P task execution

% figure(point2fig);
set(gca,'nextplot','replacechildren');
TOOL = char(robot.BodyNames(33));

v = VideoWriter('taskExecution.avi');
v.Quality = 100;
v.FrameRate = 20;
open(v)

for i=1:length(config)
    hold off;
    figure(point2fig);
    show(robot,config(1:6,i),'PreservePlot',false);
    hold on;
    
    view(45,45);
    box on;

% Visualize intermidiate points
%     gd = getTransform(robot,config(1:6,i),TOOL);
%     figure(point2fig);
%     scatter3(gd(1,4),gd(2,4),gd(3,4),'k*');
%     hold on;    
    
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);
video = 'taskExecution.avi';

end