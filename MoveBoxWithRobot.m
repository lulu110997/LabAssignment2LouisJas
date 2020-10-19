function MoveBoxWithRobot(robot, jointStates, box_h)
% Function to move the box with the robot during pick and place operation

% Get vertex count
boxVertexCount = size(get(box_h, 'Vertices'),1);

midPoint = (sum(get(box_h, 'Vertices'))/boxVertexCount);
boxVerts = (get(box_h, 'Vertices') - repmat(midPoint,boxVertexCount,1))*rotx(pi);

% Get number of robot joints to step through
steps = size(jointStates,1);
robot.delay = 0.001;
for i = 1:steps
    
    boxPose = robot.fkine(robot.getpos);
    
    % Transform the box vertices
    updatedPoints = [boxPose * [boxVerts,ones(boxVertexCount,1)]']';
    
    % Update robot joint state and box location
    set(box_h, 'Vertices', updatedPoints(:,1:3));
    robot.animate(jointStates(i,:));
end

