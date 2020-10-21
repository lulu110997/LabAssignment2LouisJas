function boxes_h = PickAndPlaceWithCollisionDetecAndAvoid(robot, obstaclePatch, obstacleObject)

if nargin <3
    clear
    close
    clc
    robot = CreateUR10eModel;
    obstaclePatch = PlaceObject('bigLego.ply', [-0.15, 0.75, 0.49]); % z = 0.09 for box on pallet. originally 0.06807
    obstacleObject = get(obstaclePatch);
end

boxes_h = cell(1,17); % Preallocate for speed

% Variables for pick and place applications
homeLocation = [0 -pi/2 pi/2 -pi/2 -pi/2 0];
louisBoxPickUpPoint = [0.75, 0.68, 0.63];
jasBoxPickUpPoint = [0.75 0.68, 0.58]; 

% Vertically stack the array so that you have an equal number of pickup
% points for each louisBox and jasBox
pickUpPoints = [repmat(louisBoxPickUpPoint,9,1); repmat(jasBoxPickUpPoint,8,1)]; 

% Goal locations from layout
goalLocations = [ % Box 1 2 3 4
    -0.8673 0.3598 0.356
    -0.6173	0.3598 0.356
    -0.3673	0.3598 0.356
    -0.8673	0.1347 0.356
    % 5 6 7 8 9
    -0.6173	0.1347	0.356
    -0.3673	0.1347	0.356
    -0.8673	-0.0901	0.356
    -0.6173	-0.0901	0.356
    -0.3673	-0.0901	0.356
    % 10 11 12 13
    -0.9496	-0.2768	0.306;
    -0.7496	-0.2768	0.306;
    -0.5496	-0.2768	0.306;
    -0.3496	-0.2768	0.306;
    % 14 15 16 17
    -0.9496	-0.4268	0.306;
    -0.7496	-0.4268	0.306;
    -0.5496	-0.4268	0.306;
    -0.3496	-0.4268	0.306;
    ];


[k2, ~] =  convhull(obstacleObject.XData, obstacleObject.YData, obstacleObject.ZData, 'Simplify', true);
simpleObstacle = trisurf(k2,obstacleObject.XData,obstacleObject.YData,obstacleObject.ZData,'FaceAlpha',0.1,'EdgeAlpha',0);
obstacleVertices = simpleObstacle.Vertices;
obstacleFaces = simpleObstacle.Faces; pause(0.1)
obstacleFaceNormals = simpleObstacle.FaceNormals;
robot.delay = 0.1;
counter = 0; % Used to change box size

q = robot.getpos;
q1 = homeLocation;

if max(abs(mean(q-q1))) > 0.1745
    robot.animate(jtraj(q,q1,50))
end

%%%%%%%%%%%%%%%%%%%%%%%%%% Collision avoidance %%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:size(goalLocations, 1)
    robot.animate(jtraj(robot.getpos, homeLocation, 25));
    counter = counter + 1;
    if counter < 10
        box = 'louisBox.ply';
    else; box = 'jasBox.ply';
    end
    box_h = PlaceObject(box, pickUpPoints(i,:));
    boxes_h{i} = box_h;
    % Go to table FROM HOME LOCATION!!!!
    q1 = robot.getpos;
    joinStates = DetectAndAvoidObstacle(robot, q1, [pickUpPoints(i,1:2), 1.2*pickUpPoints(i,3)], ... 
        obstacleVertices, obstacleFaces, obstacleFaceNormals, 'pickup');
    robot.animate(joinStates)
    abovePickUpPose = robot.getpos;
    
    % Pick up box
    pickUpPose = robot.ikcon(transl(pickUpPoints(i,:))*troty(pi), robot.getpos);
    pickUpPose(6) = abovePickUpPose(6);
    pickUpJointStates = jtraj(robot.getpos, pickUpPose, 25);
    robot.animate(pickUpJointStates);
    
    % Lift box
    jointStates = jtraj(robot.getpos, abovePickUpPose, 10);
    MoveBoxWithRobot(robot, jointStates, box_h)  
    
    % Move box to home
    q1 = robot.getpos;
    tr = robot.fkine(homeLocation);
    jointStates = DetectAndAvoidObstacle(robot, q1, tr(1:3,4)', ...
        obstacleVertices, obstacleFaces, obstacleFaceNormals, 'dropoff');
    MoveBoxWithRobot(robot, jointStates, box_h)
    
    % Move box to above goal 
    q1 = robot.getpos;
    jointStates = DetectAndAvoidObstacle(robot, q1, [goalLocations(i,1:2), 1.2*goalLocations(i,3)], ...
        obstacleVertices, obstacleFaces, obstacleFaceNormals, 'dropoff');
    MoveBoxWithRobot(robot, jointStates, box_h)
    aboveGoalPose = robot.getpos;
    
    % Move box to goal 
    goalPose = robot.ikcon(transl(goalLocations(i,:))*troty(pi), robot.getpos);
    goalPose(6) = aboveGoalPose(6);
    jointStates = jtraj(robot.getpos, goalPose, 15);
    MoveBoxWithRobot(robot, jointStates, box_h)  
    
    % Move above goal location
    jointStates = jtraj(robot.getpos, aboveGoalPose, 10);
    robot.animate(jointStates)
    keyboard
end

end

