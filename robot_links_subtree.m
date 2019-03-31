function [robot_links,CoM_robot_links] = robot_links_subtree(robot,config,nDoF)
% Input 1: a robot tree model imported from urdf file
% Input 2: robot configuration
% Input 3: robot DoF
% Output 1: links of robot as Matlab robot models
% Output 2: CoM of each link in spatial frame

% Construct the robot objects
for j = 1:nDoF+1 % nLinks = nDoF + 1
    robot_links(j) = robotics.RigidBodyTree; % number of links is equal to Number of DoF
    robot_links(j).DataFormat = 'column';
end

bodyNames = robot.BodyNames; % save existing names of Bodies

i = 1; % counter of Bodies in initial robot object
k = 1; % counter of Bodies in each sub robot model
j = 1; % counter of sub robot models, start for first robot link - must end in 3
robot_links(j).BaseName = robot.BaseName;
for i=1:robot.NumBodies % Runs for all Bodies of the robot object
    name = char(robot.BodyNames(i));
    Joint_Type_check = string(robot.getBody(name).Joint.Type);
    
    if Joint_Type_check == "revolute"
        % stop buliding current subtree
        robot_links(j+1).BaseName = robot.getBody(name).Name; % last body of i-1 is base for i
        if j == 1 % for first sub model don't change basename 
            addBody(robot_links(j),robot.getBody(name),robot_links(j).BaseName); % adds last body with first base name
            body_counter(1) = k;% final number of bodies in link
        else   
            addBody(robot_links(j),robot.getBody(name),char(robot_links(j).BodyNames(k-1))); % adds last body
            body_counter(j) = k;
%             showdetails(robot_links(j))
        end
        k=1;
        j=j+1; % in next for addBody will add Body to new tree
    else
        % constracts subtrees by adding bodies
        % Here we add bodies-use
        if k ==1 % first body must have basename defined in line23
            addBody(robot_links(j),robot.getBody(name),robot_links(j).BaseName);
        else
            addBody(robot_links(j),robot.getBody(name),char(robot_links(j).BodyNames(k-1)));
%             showdetails(robot_links(j));
        end
        k=k+1; % k is counter of bodies in each subtree constructed
    end
    i=i+1;
end

%% CoM of each link in spatial frame
CoM_robot_links(:,1) = centerOfMass(robot_links(1)); % for link1 the CoM is calculated directly for spatial frame
for j = 2:nDoF+1 % nLinks = nDoF + 1
    showdetails(robot_links(j))
%     config(:,j) = homeConfiguration(robot_links(j)); 
    CoM_robot_links(:,j) = centerOfMass(robot_links(j)); % CoM of links in link frame(body)
    TFs_com(:,:,j) = getTransform(robot,config,char(robot_links(j-1).BodyNames(body_counter(j-1)))); % gets tf from last body of previous link to spatial frame of first robot model
    CoM_robot_links4(:,j) = TFs_com(:,:,j)*[CoM_robot_links(:,j);1]; % q|A = g|A/B * q|B
    CoM_robot_links(:,j) = CoM_robot_links4(1:3,j); % reference to spatial frame
end

%% Generalized Inertia Matrix of each link in link frame
% for j=1:nDoF % for all links of robot
%     for k=1:body_counter(j) % for all bodies in each link
        I_bj(:,:,j,k) = diag(robot_links(1,j).Bodies{1,k}.Inertia(1:3);
        M_bj(:,:,j,k)
%% Generalized Inertia Matrix of each link in spatial frame

end