function boxes_h = PickAndPlace(robot)

if nargin < 1
    robot = createUR10eModel;
end

boxes_h = cell(1,17); % Preallocate for speed
robot.delay = 0.001; % Robot delay duration during animation

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

robot.animate(jtraj(robot.getpos, homeLocation, 75)) % Call the robot back to home

% The idea of pick/place is to go from home location --> pick up location
% --> waypoint --> just above the of goal location --> goal location 
% --> home location --> repeat
count = 0; % Used to change box size
for i = 1:size(goalLocations, 1)
    count = count + 1;
    if count < 10
        box = 'louisBox.ply';
    else; box = 'jasBox.ply'; 
    end
    
    box_h = PlaceObject(box, pickUpPoints(i,:));
    boxes_h{i} = box_h;
    
    %%% Create waypoints %%%
    currentPose = robot.fkine(robot.getpos);
    currentxyzLocation = currentPose(1:3,4);
    abovePickUp = [0.75, 0.68, 0.9320];
    wayPointGoal = transl(-0.25, 0.45,0.8)*troty(pi);
    aboveGoal = [0.95*goalLocations(i,1), 0.95*goalLocations(i,2), 1.6*goalLocations(i,3)];

    %%% Create curve then move the robot %%%
    % Do not use all the joint trajectories in one go. Move the robot
    % before creating the trajectory.
    pointsBeforePickUp = horzcat( ...
        currentxyzLocation, ...
        [0 0.5 0.8320]', ...
        [0.75, 0.68, 0.9320]', ...
        pickUpPoints(i,:)' ...
        );
    curveBeforePickUp = fnplt(cscvn(pointsBeforePickUp));
    curveBeforePickUp_h = plot3(curveBeforePickUp(1,:), curveBeforePickUp(2,:), curveBeforePickUp(3,:), 'r');
    jointStatesBeforePickUp = ResolvedMotionRateControl(robot, curveBeforePickUp);
    robot.animate(jointStatesBeforePickUp) % Move to pick up position
    
    pointsDuringPickUp = horzcat( ...
        pickUpPoints(i,:)', ...
        abovePickUp', ...
        wayPointGoal(1:3,4), ...
        aboveGoal', ...
        goalLocations(i,:)' ...
        );
    curveDuringPickUp = fnplt(cscvn(pointsDuringPickUp));
    curveDuringPickUp_h = plot3(curveDuringPickUp(1,:), curveDuringPickUp(2,:), curveDuringPickUp(3,:), 'b');
    jointStatesDuringPickUp = ResolvedMotionRateControl(robot, curveDuringPickUp);
    MoveBoxWithRobot(robot, jointStatesDuringPickUp, box_h) % Move to pick up position with box
    tr = robot.fkine(robot.getpos); % Obtain the current location of EE
    
    q2 = robot.ikcon(transl(aboveGoal)*troty(pi), robot.getpos); % joint state to move robot above goal
    robot.animate(jtraj(robot.getpos, q2, 25)) % Move above goal
    
    error = max(abs((tr(1:3,4)' - goalLocations(i,:))./goalLocations(i,:)))*100;
    display(['The x drop off location is ', num2str(tr(1,4)), newline...
        'The y drop off location is ', num2str(tr(2,4)), newline...
        'The z drop off location is ', num2str(tr(3,4)), newline])
    display(['The actual location of the box is', newline 'x = ', num2str(goalLocations(i,1)), newline...
        'y = ' num2str(goalLocations(i,2)), newline 'z = ' num2str(goalLocations(i,3)), newline])
    display(['Maximum error between actual location and goal location was ', num2str(round(error, 2)), '%', newline])
    
    % Remove path robot takes
    delete(curveBeforePickUp_h)
    delete(curveDuringPickUp_h)
end

robot.animate(jtraj(robot.getpos, homeLocation, 100)) % Call the robot back to home
end

% %% Testing phase %%
% robot = createUR10eModel;
% goalLocations = [ % Box 1 2 3 4
%     -0.8673 0.3598 0.356;
%     -0.6173	0.3598 0.356;
%     -0.3673	0.3598 0.356;
%     -0.8673	0.1347 0.356;
%     % 5 6 7 8 9
%     -0.6173	0.1347	0.356;
%     -0.3673	0.1347	0.356;
%     -0.8673	-0.0901	0.356;
%     -0.6173	-0.0901	0.356;
%     -0.3673	-0.0901	0.356;
%     % 10 11 12 13
%     -0.9496	-0.2768	0.306;
%     -0.7496	-0.2768	0.306;
%     -0.5496	-0.2768	0.306;
%     -0.3496	-0.2768	0.306;
%     % 14 15 16 17
%     -0.9496	-0.4268	0.306;
%     -0.7496	-0.4268	0.306;
%     -0.5496	-0.4268	0.306;
%     -0.3496	-0.4268	0.306;
%     ];
% for i = 1:9
%     PlaceObject('louisBox.ply', goalLocations(i,:));
% end
% for i = 10:17
%     PlaceObject('jasBox.ply', goalLocations(i,:));
% end
% % PlaceObject('louisBox.ply', [-1.0086 0.4434 0.356]);
% % PlaceObject('louisBox.ply', [-0.7586 0.4434 0.356]);
% % PlaceObject('louisBox.ply', [-0.5086 0.4434 0.356]);
% % PlaceObject('louisBox.ply', [-0.2679 0.4434 0.356]);
% % PlaceObject('louisBox.ply', [-1.0086 0.2084 0.356]);
% % PlaceObject('louisBox.ply', [-0.7586 0.2084	0.356]);
% % PlaceObject('louisBox.ply', [-0.5086 0.2084	0.356]);
% % PlaceObject('louisBox.ply', [-0.2679 0.2084	0.356]);
% % PlaceObject('louisBox.ply', [-1.0086 -0.0165 0.356]);
% % PlaceObject('louisBox.ply', [-0.7586 -0.0165 0.356]);
% % PlaceObject('louisBox.ply', [-0.5086 -0.0165 0.356]);
% % PlaceObject('louisBox.ply', [-0.2679 -0.0165 0.356]);
% % 
% % 
% % PlaceObject('jasBox.ply', [-1.0409	-0.1838	0.306]);
% % PlaceObject('jasBox.ply', [-0.8409	-0.1838	0.306]);
% % PlaceObject('jasBox.ply', [-0.6409	-0.1838	0.306]);
% % PlaceObject('jasBox.ply', [-0.4409	-0.1838	0.306]);
% % PlaceObject('jasBox.ply', [-0.2409	-0.1838	0.306]);
% % PlaceObject('jasBox.ply', [-1.0409	-0.3338	0.306]);
% % PlaceObject('jasBox.ply', [-0.8409	-0.3338	0.306]);
% % PlaceObject('jasBox.ply', [-0.6409	-0.3338	0.306]);
% % PlaceObject('jasBox.ply', [-0.4409	-0.3338	0.306]);
% % PlaceObject('jasBox.ply', [-0.2409	-0.3338	0.306]);
% % PlaceObject('jasBox.ply', [-1.0409	-0.4838	0.306]);
% % PlaceObject('jasBox.ply', [-0.8409	-0.4838	0.306]);
% % PlaceObject('jasBox.ply', [-0.6409	-0.4838	0.306]);
% % PlaceObject('jasBox.ply', [-0.4409	-0.4838	0.306]);
% % PlaceObject('jasBox.ply', [-0.2409	-0.4838	0.306]);
% %%
% r = robot;
% initialGuess = [0 -0.5655 1.1938 -2.1991 -1.5708 0];
% for j = 1:size(goalLocations, 1)
%     
%     clc
%     jointStates(j,:) = r.ikcon(transl(goalLocations(j,:))*troty(pi), initialGuess);
%     tr{j} = r.fkine(jointStates(j,:));
%     asd = 100*(((tr{j}(1:3,4))' - goalLocations(j,:))./goalLocations(j,:));
%     if max(abs(asd)) > 1
%         disp('inaccurate position')
%         display(max(abs(asd)))
%     end
%     r.animate(jointStates(j,:));
%     J = r.jacob0(jointStates(j,:));
%     if sqrt(det(J*J')) < 0.1
%         display('singularity')
%     end
%     tr2rpy((tr{j}(1:3,1:3)))
%     pause
%     
% end
