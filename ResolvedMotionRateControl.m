function jointStates = ResolvedMotionRateControl(robot, pathToTake)
% Resolved motion rate control from lab 9
% initialLocation and finalLocation needs to be a nx3 matrix where each xyz
% location is stored as a row vector in the form of [x y z]

% RMRC variables 
t = 1; %Total time
steps = size(pathToTake,2); % No. of steps depending on the number of points in space the cscvn func gave
deltaT = t/steps; % Discrete time step. Use 0.02 cause it's same as lab
rpy = zeros(3,steps); % Array for roll-pitch-yaw angles
xyz = zeros(3,steps); % Array for x-y-z trajectory
W = diag([1 1 1 0.5 0.5 0.5]); % Weighting matrix for the velocity vector
maxDamping = 0.1; % Damping coefficient
mLim = 0.15; % Minimum value for manipulability
qMatrix = zeros(steps,6); % Array for joint angles
qMatrix(1,:) = robot.getpos; % Get robot's current joint state and save it in the matrix

% Obtain xyz and rpy values
for j=1:steps
    xyz(1,j) = pathToTake(1,j); % Points in x
    xyz(2,j) = pathToTake(2,j); % Points in y
    xyz(3,j) = pathToTake(3,j); % Points in z
    rpy(1,j) = pi; % Roll angle %-0.4636; %
    rpy(2,j) = 0; % Pitch angle %-1.5708; %
    rpy(3,j) = -pi/2; % Yaw angle %1.1071; %
end

for i = 1:steps-1
    T = robot.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
    deltaX = xyz(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(rpy(1,i+1),rpy(2,i+1),rpy(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    
    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!

    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = robot.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
    m = sqrt(det(J*J'));
    if m < mLim  % If manipulability is less than given threshold
        lambda = sqrt((1 - (m/mLim)^2)*maxDamping^2);
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the vector)
    for j = 1:6                                                             % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
end

jointStates = qMatrix;

end

