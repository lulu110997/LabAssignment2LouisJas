function MoveBoxWithRobot(robot, jointStates, box_h)
% Function to move the box with the robot during pick and place operation

% Get vertex count
boxVertexCount = size(box_h.Vertices,1);

midPoint = (sum(box_h.Vertices)/boxVertexCount);
boxVerts = box_h.Vertices - repmat(midPoint,boxVertexCount,1);

% Get number of robot joints to step through
steps = size(jointStates,1);
robot.delay = 0;
for i = 1:steps
    boxPose = robot.fkine(robot.getpos)*troty(pi);
    boxPose(3,4) = boxPose(3,4);
    
    % Transform the box vertices
    updatedPoints = [boxPose * [boxVerts,ones(boxVertexCount,1)]']';
    
    % Update robot joint state and box location
    robot.animate(jointStates(i,:));
    box_h.Vertices = updatedPoints(:,1:3);
    pause(0.075)
end

