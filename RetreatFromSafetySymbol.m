function RetreatFromSafetySymbol(robot,cam,stopSign_h)
% Use visual servoing to retreat from a safety symbol. It needs the robot
% object as an input to do the fkine and ikine calcs. Then the patch of the
% safety sign it will retreat from

% Used for testing. Moving the robot to the initial position should be
% outside of this function under normal operation
if nargin < 2
    % Move the robot to the initial pose where it can see the stop sign
    clc
    close
    clear
    cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
        'resolution', [1024 1024], 'centre', [512 512],'name', 'testCamera');
    robot = createUR10eModel;
    jointForCameraPos = [0 -2.1437 1.7866 -2.7846 -1.5706 0];
    robot.animate(jointForCameraPos);
    stopSign_h = PlaceObject('stopSign.ply', [-0.805, -0.1741, 1.4133]); % Original value is [-1.1, -0.1741, 1.2133]. Don't change y value
end

% Create image target (points in the image plane)
pStar = [
  638.0931  393.4378  723.1931  605.6851  387.8622;
  518.5031  580.2681  622.5805  697.6496  914.4936
  ];

% Choose specific points from the object that you want to use to dictate 
% how the UR10e will move.
v = get(stopSign_h, 'Vertices');
xGoal = [v(1,1), v(140,1), v(280,1), v(420,1), v(560,1)];
yGoal = [v(1,2), v(140,2), v(280,2), v(420,2), v(560,2)];
zGoal = [v(1,3), v(140,3), v(280,3), v(420,3), v(560,3)];
goals = [xGoal' yGoal' zGoal'];
P = goals'; % 3D points in Cartesian space that the UR10e will move in reference with

% Add the camera and plot it on the robot's EE
Tc0= robot.fkine(robot.getpos);
cam.T = Tc0; % Changes camera transform to the robot's EE location
cam.plot_camera('Tcam',Tc0, 'label','scale',0.15); % Plots camera

% frame rate
fps = 25;

%Project points to the image
p = cam.plot(P, 'Tcam', Tc0);

% Camera view and plotting
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(true);
cam.plot(P);    % show initial view

lambda = 1.05; % Gain of the controler
depth = abs(mean(P(1,:))); % Depth of the IBVS 

q0 = robot.getpos';
%% 1.4 Loop
% loop of the visual servoing
for i = 1:125
    
    % compute the view of the camera
    uv = cam.plot(P);
    
    % compute image plane error as a column
    e = (pStar - uv);   % feature error
    e = e(:);
    Zest = [];
    
    % compute the Jacobian
    if isempty(depth)
        % exact depth from simulation (not possible in practice)
        pt = homtrans(inv(Tcam), P);
        J = cam.visjac_p(uv, pt(3,:) );
    elseif ~isempty(Zest)
        J = cam.visjac_p(uv, Zest);
    else
        J = cam.visjac_p(uv, depth);
    end
    
    % compute the velocity of camera in camera frame
    try
        v = lambda * pinv(J) * e;
    catch
        status = -1;
        return
    end
%     fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
    
    % Find the robot's manipulability
    J = robot.jacob0(robot.getpos);
    m = sqrt(det(J*J'));
    if m < 0.1  % If manipulability is less than given threshold
        lambda2 = sqrt((1 - (m/0.1)^2)*0.2^2);
    else
        lambda2 = 0;
    end
    
    %compute robot's Jacobian and inverse
    J2 = robot.jacobn(q0);
%     Jinv = pinv(J2);
    Jinv = pinv(J2'*J2 + lambda2 *eye(6))*J2';
    % get joint velocities
    qp = Jinv*v;
    
    
    %Maximum angular velocity cannot exceed 180 degrees/s
    ind=find(qp>pi);
    if ~isempty(ind)
        qp(ind)=pi;
    end
    ind=find(qp<-pi);
    if ~isempty(ind)
        qp(ind)=-pi;
    end
    
    %Update joints
    q = q0 + (1/fps)*qp; % Note that q0 has to be a column vector!!!!!!!!!!!!!
    robot.animate(q');
    
    %Get camera location
    Tc = robot.fkine(q);
    cam.T = Tc;
    
    drawnow
    %update current joint position
    q0 = q;
    pause(1/fps)
    
    if abs(mean(e)) < 1.1
        break;
    end
    
    
end %loop finishes
end