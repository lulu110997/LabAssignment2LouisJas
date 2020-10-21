function jointStates = DetectAndAvoidObstacle(robot, q1,PickOrDropCoords, obstacleVertices, obstacleFaces, obstacleFaceNormals, action)
iteration = 1;
offset = 1;
while true
    if iteration == 1
        if strcmp(action,'pickup') % == "pickup" will not work. " and ' are different
            qInitialGuess = [-2.3876 -1.5708 1.5708 -1.5708 -1.5708 0];
        else
            qInitialGuess = [0.01 -pi/2 pi/2 -pi/2 -pi/2 0.01];
        end
    else 
        qInitialGuess = (robot.qlim(:,1) + (robot.qlim(:,2)-robot.qlim(:,1)))*rand(1);
%         robot.animate(offset*qMatrix);
%         q1 = 0.1*robot.getpos;
%         robot.animate(q1);
    end 
    
    q2 = robot.ikcon(transl(PickOrDropCoords(1), PickOrDropCoords(2), PickOrDropCoords(3))*troty(pi), qInitialGuess);
    qWaypoints = [q1;q2];
    isCollision = true;
    checkedTillWaypoint = 1;
    qMatrix = [];
    while (isCollision)
        startWaypoint = checkedTillWaypoint;
        disp(size(qMatrix,1))
%         if size(qMatrix,1) > 40
%             qMatrix = [0 0 0 0 0 0];
%             break
%         end
        for i = startWaypoint:size(qWaypoints,1)-1
            qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(17));
            if ~IsCollision(robot,qMatrixJoin,obstacleFaces,obstacleVertices,obstacleFaceNormals)
                qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
                %             robot.animate(qMatrixJoin);
                isCollision = false;
                checkedTillWaypoint = i+1;
                % Now try and join to the final goal (q2) % original q2
                % location is below here
%                 try
%                     q2 = robot.ikcon(transl(PickOrDropCoords(1), PickOrDropCoords(2), PickOrDropCoords(3))*troty(pi), qMatrix(i,:)); %
%                 catch
%                     q2 = robot.ikcon(transl(PickOrDropCoords(1), PickOrDropCoords(2), PickOrDropCoords(3))*troty(pi), qInitialGuess);
%                 end
                % original q2 locations is above here
                %             tr = robot.fkine(q2);
                %             err = max((abs(mean(tr(1:3,4))) - PickOrDropCoords'));
                %             errAcc = 0;
                %             while err > 0.2
                %                 qGuess = (errAcc + 0.01)*q2;
                %                 q2 = robot.ikcon(transl(PickOrDropCoords(1), PickOrDropCoords(2), PickOrDropCoords(3))*troty(pi), qGuess);
                %                 tr = robot.fkine(q2);
                %                 err = max((abs(mean(tr(1:3,4))) - PickOrDropCoords'))
                %                 errAcc = err + errAcc
                %
                %                 pause(0.1)
                % %                 display(['Error is ', err])
                %             end
                qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(17));
                if ~IsCollision(robot,qMatrixJoin,obstacleFaces,obstacleVertices,obstacleFaceNormals)
                    qMatrix = [qMatrix;qMatrixJoin];
                    % Reached goal without collision, so break out
                    break;
                end
                try
                    q2 = robot.ikcon(transl(PickOrDropCoords(1), PickOrDropCoords(2), PickOrDropCoords(3))*troty(pi), qMatrix(i,:)); %
                catch
                    q2 = robot.ikcon(transl(PickOrDropCoords(1), PickOrDropCoords(2), PickOrDropCoords(3))*troty(pi), qInitialGuess);
                end
            else
                % Randomly pick a pose that is not in collision
                randomXLocation = -0.85 + (1-(-0.85)).*rand(1);
                randomYLocation = -0.8 + (1-(-0.8)).*rand(1);
                randomZLocation = 0.5 + (1-(0.5)).*rand(1);
                qRand = robot.ikcon(transl(randomXLocation, randomYLocation, randomZLocation)*troty(pi), qWaypoints(i,:));
                qRand(4) = -pi/2;
                qRand(5) = -pi/2;
                while IsCollision(robot,qRand,obstacleFaces,obstacleVertices,obstacleFaceNormals) %& qRand < abs(robot.qlim(:,1)) & z < 0.4
                    randomXLocation = -0.85 + (1-(-0.85)).*rand(1);
                    randomYLocation = -0.6 + (1-(-0.6)).*rand(1);
                    randomZLocation = 0.5 + (1.5-(0.5)).*rand(1);
                    try
                        qRand = robot.ikcon(transl(randomXLocation, randomYLocation, randomZLocation)*troty(pi), qMatrix(i,:)); %
                    catch
                        qRand = robot.ikcon(transl(randomXLocation, randomYLocation, randomZLocation)*troty(pi), qInitialGuess);
                    end
                    qRand(4) = -pi/2;
                    qRand(5) = -pi/2;
                end
                qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
                isCollision = true;
                break;
            end
        end
    end
    
    tr = robot.fkine(qMatrix(end,:));
    err = round(abs(max((tr(1:3,4)) - PickOrDropCoords')),3);
    if abs(err) < 0.09
        disp(['Final joint state is acceptable, error is ',  num2str(abs(err)), ' m'])
        jointStates = qMatrix; %%%%%%%%%%%
        return
    end
    disp (['Final joint state is unacceptable, error is ', num2str(abs(err)), ' m'])
    offset = rand(1)*pi;
    iteration = iteration + 1;
    pause(1.2)
%     clc
end
end

%%% Test code %%%
% stepTest = size(qMatrix,1);
% xyzTest = zeros(stepTest,3);
% % dTest = zeros
% for i = 1:stepTest
%     trTest = robot.fkine(qMatrix(i,:));
%     xyzTest(i,:) = (trTest(1:3,4))';
% end 
% 
% dTest = pdist(xyzTest);
% pt = randperm(numel(dTest),30);
% dTest(pt) = 0;
% dTest = squareform(dTest)
% G = graph(dTest);
% G.Nodes = array2table(xyzTest,'VariableNames',{'X','Y','Z'});
% [P,L] = shortestpath(G,1,stepTest);
% hold on
% h = plot(G,'XData',G.Nodes.X,'YData',G.Nodes.Y,'ZData',G.Nodes.Z, 'EdgeAlpha', 1);
% highlight(h,P,'EdgeColor','g','LineWidth',2)
% % Display the path length
% disp(L)

