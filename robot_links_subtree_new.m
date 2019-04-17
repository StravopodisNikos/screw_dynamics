function [robot_links,CoM_robot_links,gsli0,M_CoM,M_s_CoM] = robot_links_subtree_new(robot,config,nDoF)
% Input 1: a robot tree model imported from urdf file
% Input 2: robot configuration
% Input 3: robot's DoF
% Output 1: links of robot as Matlab robot models
% Output 2: CoM of each link in spatial frame 3x(No_of_links)
% Output 3: Generalized Inertia Matrix of each link in link's CoM frame 6x6x(No_of_links)
% Output 4: Generalized Inertia Matrix of each link in spatial frame 6x6x(No_of_links)

% Construct the robot objects
for j = 1:nDoF % nLinks = nDoF
    robot_links(j) = robotics.RigidBodyTree; % number of links is equal to Number of DoF
    robot_links(j).DataFormat = 'column';
end

bodyNames = robot.BodyNames; % save existing names of Bodies

count_rev = 0; % counter of revolute joints
% i = 1; % counter of Bodies in initial robot object
k = 0; % counter of Bodies in each sub robot model
j = 1; % counter of sub robot models, start for first robot link - must end in 3
robot_links(j).BaseName = robot.BaseName;
for i=1:robot.NumBodies % Runs for all Bodies of the robot object
    name = char(robot.BodyNames(i)); % takes name of current body of initial robot
    Joint_Type_check = string(robot.getBody(name).Joint.Type);
    
    if Joint_Type_check =="fixed"
        % adds current body to current tree
        if count_rev == 0 && k==0
            addBody(robot_links(j),robot.getBody(name),robot_links(j).BaseName);
        else
            addBody(robot_links(j),robot.getBody(name),char(robot_links(j).BodyNames(k)));
        end
        k = k+1;
    else
        % this is first body of new tree
        count_rev = count_rev+1;
        if count_rev ==1
            addBody(robot_links(j),robot.getBody(name),robot_links(j).BaseName);
            k = k+1;
        else
            body_counter(j) = k;
            k=1;
            % must set new base frame
            % new base frame is last of previous tree
            robot_links(j+1).BaseName = robot.getBody(char(robot_links(j).BodyNames(body_counter(j)))).Name;
            addBody(robot_links(j+1),robot.getBody(name),robot_links(j+1).BaseName);
            j= j+1;
        end    
    end
end

% Specify number of bodies of final link
 body_counter(j) = robot.NumBodies - sum(body_counter);
 
%% CoM of each link in spatial frame
% CoM_robot_links(:,1) = [0 0 0.057]; % for link1 the CoM is calculated directly for spatial frame
for j = 1:nDoF % nLinks = nDoF + 1
%     showdetails(robot_links(j));
%     config(:,j) = homeConfiguration(robot_links(j)); 
    CoM_robot_links(:,j) = centerOfMass(robot_links(j),config(j)); % CoM of links reletive to link base frame
%     TFs_com1(:,:,j) = getTransform(robot,config,char(robot_links(j).BodyNames(1)),char(robot.BaseName));
    TFs_com2(:,:,j) = getTransform(robot,config,char(robot_links(j).BaseName),char(robot.BaseName));
%     TFs_com3(:,:,j) = getTransform(robot,config,char(robot_links(j-1).BodyNames(body_counter(j-1)))); % gets tf from last body of previous link to spatial frame of first robot model
%     CoM_robot_links1(:,j) = TFs_com1(:,:,j)*[CoM_robot_links(:,j);1]; % q|A = g|A/B * q|B
    CoM_robot_links2(:,j) = TFs_com2(:,:,j)*[CoM_robot_links(:,j);1]; % q|A = g|A/B * q|B
%     CoM_robot_links3(:,j) = TFs_com3(:,:,j)*[CoM_robot_links(:,j);1]; % q|A = g|A/B * q|B
    
    CoM_robot_links(:,j) = CoM_robot_links2(1:3,j); % reference to spatial frame
end

%% Generalized Inertia Matrix of each body in com_link frame
for j=1:nDoF % for all links of robot
    showdetails(robot_links(j));
    
    M_CoM(:,:,j) = zeros(6); % Initialiazation of final generalized inertia matrix of each link expressed in CoM frame of link
    
        gsli0(:,:,j) = [1     0     0     CoM_robot_links(1,j);... % tf of CoM of each link to spatial frame 
                        0     1     0     CoM_robot_links(2,j);...
                        0     0     1     CoM_robot_links(3,j);...
                        0     0     0     1];
                
    for k=1:body_counter(j) % for all bodies in each link
        I_bj(:,:) = diag(robot_links(1,j).Bodies{1,k}.Inertia(1:3)); % puts Ixx Iyy Izz in main diagonal of 3x3 matrix
        I_bj(1,2) = robot_links(1,j).Bodies{1,k}.Inertia(6); % Ixy
        I_bj(1,3) = robot_links(1,j).Bodies{1,k}.Inertia(5); % Ixz
        I_bj(2,3) = robot_links(1,j).Bodies{1,k}.Inertia(4); % Iyz
        
        I_bj(2,1) = I_bj(1,2);
        I_bj(3,1) = I_bj(1,3);
        I_bj(3,2) = I_bj(2,3);
        
        mass_bj = diag(robot_links(1,j).Bodies{1,k}.Mass); %[kg]
        Mass_bj = mass_bj.*eye(3);
        M_bj = [Mass_bj zeros(3); zeros(3) I_bj]; % Generalized Inertia Matrix in blue frame-reference frame of every object defined in urdf

%         g_sbj = getTransform(robot,config,char(robot_links(j).BodyNames(body_counter(j)))); % Get tf of body object in spatial frame
%         g_bjli0 = inv(g_sbj)*gsli0(:,:,j); % get tf of CoM of link to body object frame
        
%         [M_CoM_bj] = transformed_inertia_matrix(M_bj,g_bjli0); % Generalized Inertia Matrix in CoM frame of the link, for each body inside the link
%         [M_CoM_bj] = transformed_inertia_matrix(M_bj,g_sbj); % Generalized Inertia Matrix of the link, in spatial
        
        M_CoM(:,:,j) = M_CoM(:,:,j) + M_bj;
%         M_CoM(:,:,j) = M_CoM(:,:,j) + M_CoM_bj; % Sum of Gen. In. Matrices of bodies of each link in CoM frame of each link
    end

    [M_s_CoM(:,:,j)] = transformed_inertia_matrix(M_CoM(:,:,j),gsli0(:,:,j)); % Generalized Inertia Matrix of CoM frame of each link expressed in spatial frame of manipulator          

end     

end