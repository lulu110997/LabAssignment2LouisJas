function newLocation = changeEndEffectorLocation(robot, locationToChange, desiredLocation)
% Input the instance of a robot whose end effector position you want to change.
% Specify if it is the x, y or z position you want to change. Then specify
% the new desired location. This function will then extract specific
% details from the robot's current EE location to obtain a joint state for
% the new desired position
% Code for testing
if nargin ~= 3
    clear; close; clc;
    robot = createUR10eModel;
    locationToChange = 'z';
    desiredLocation = 1;
end
% End code used for testing

% Find out which axis needs to be changed
currentJointState = robot.getpos;
Tr = robot.fkine(currentJointState);
eulerAngles = tr2rpy(Tr(1:3, 1:3));

if locationToChange == 'x'
    xLocation = desiredLocation;
    yLocation = Tr(2,4);
    zLocation = Tr(3,4);
    
elseif locationToChange == 'y'
    xLocation = Tr(1,4);
    yLocation = desiredLocation;
    zLocation = Tr(3,4);

else 
    xLocation = Tr(1,4);
    yLocation = Tr(2,4);
    zLocation = desiredLocation;
    
end

t = 1;                                                                     % Total time
steps = 20;                                                               % No. of steps
deltaT = t/steps;                                                          % Discrete time step
W = diag([1 1 1 0.5 0.5 0.5]);                                             % Weighting matrix for the velocity vector
damping = 0.1;
m = 0.15;

qMatrix = zeros(steps,6);                                                  % Array for joint angles
qdot = zeros(steps,6);                                                     % Array for joint velocities
theta = zeros(3,steps);                                                    % Array for roll-pitch-yaw angles
x = zeros(3,steps);                                                        % Vector column for x-y-z trajectory

s = lspb(0,1,steps);                                                       % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = (1-s(i))*Tr(1,4) + s(i)*xLocation;                            % Points in x
    x(2,i) = (1-s(i))*Tr(2,4) + s(i)*yLocation;                            % Points in y
    x(3,i) = (1-s(i))*Tr(3,4) + s(i)*zLocation;                            % Points in z
    theta(1,i) = eulerAngles(1);                                           % Roll angle
    theta(2,i) = eulerAngles(2);                                           % Pitch angle
    theta(3,i) = eulerAngles(3);                                           % Yaw angle
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];         % Create transformation of first point and angle
q0 = currentJointState;                                                    % Initial guess for joint angles
qMatrix(1,:) = robot.ikcon(T,q0);                                          % Solve joint angles to achieve first waypoint

for i = 1:steps-1
    T = robot.fkine(qMatrix(i,:));                                         % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                          % Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1), theta(2,i+1), theta(3,i+1));                  % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                       % Current end-effector rotation matrix
    
    Rdot = (1/deltaT)*(Rd - Ra);                                           % Calculate rotation matrix error (see RMRC lectures)
    S = Rdot*Ra';                                                          % Skew symmetric! S(\omega)
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                             % Check the structure of Skew Symmetric matrix! Extract the angular velocities. (see RMRC lectures)

    xdot = W*[linear_velocity;angular_velocity];                           % Calculate end-effector velocity to reach next waypoint.
    J = robot.jacob0(qMatrix(i,:));                                        % Get Jacobian at current joint state
    
    if sqrt(det(J*J')) < m                                              % If manipulability is less than given threshold
        lambda = damping;                                                      % Damping coefficient (try scaling it)
        invJ = inv(J'*J + lambda *eye(6))*J';                              % Apply Damped Least Squares pseudoinverse
    else
        lambda = 0;
        invJ = inv(J'*J + lambda *eye(6))*J';                              % Don't use DLS
    end
    qdot(i,:) = (invJ*xdot)';                                              % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:6                                                            % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.qlim(j,1)               % If next joint angle is lower than joint limit...
            qdot(i,j) = 0;                                                 % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.qlim(j,2)           % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0;                                                 % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);
end
robot.animate(qMatrix(end,:))
Tr = robot.fkine(robot.getpos);
newLocation = Tr(1:3,4);
end
