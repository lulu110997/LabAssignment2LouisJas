clear
robot = CreateUR10eModel;
robot.delay = 0.25;
q = robot.getpos;        

bigLego = PlaceObject('bigLego.ply', [-0.15, 0.75, 0.49]); % z = 0.09 for box on pallet. originally 0.06807
% bigLego = PlaceObject('bigLego.ply', [0.55, 0.58, 0.63]);
a = get(bigLego);
[k2, av2] =  convhull(a.XData, a.YData, a.ZData);
simpleObstacle = trisurf(k2,a.XData,a.YData,a.ZData,'FaceAlpha',0.1,'EdgeAlpha',0);
bigLegoVertices = simpleObstacle.Vertices;
bigLegoFaces = simpleObstacle.Faces;
pause(0.1)
bigLegoFaceNormals = simpleObstacle.FaceNormals;
% bigLegoFaceNormals2 = surfnorm(a.XData, a.YData, a.ZData)';
q1 = [0 -pi/2 pi/2 -pi/2 -pi/2 0];
q2 = robot.ikcon(transl(0.75, 0.68, 0.8)*troty(pi),[1.4767 -2.0735 -0.8727 -1.5708 1.5708 0]);

tr = zeros(4,4,robot.n+1);
tr(:,:,1) = robot.base;
L = robot.links;
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

robot.animate(q1)
pause
robot.animate(q2)
for i = 1 : size(tr,3)-1    
    for faceIndex = 1:size(bigLegoFaces,1)
        vertOnPlane = bigLegoVertices(bigLegoFaces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(bigLegoFaceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if check == 1 && IsIntersectionPointInsideTriangle(intersectP,bigLegoVertices(bigLegoFaces(faceIndex,:)',:))
                    plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                    disp(['link number ', num2str(i), ' results in ' num2str(check)])
                end
    end    
end

steps = 2;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),100))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);

 result = true(steps,1);
%  for i = 1: steps
%      result(i) = IsCollision(robot,qMatrix(i,:),bigLegoFaces,bigLegoVertices,bigLegoFaceNormals, false); 
%      if result(i)
%          break
%      end
%      robot.animate(qMatrix(i,:));
%  end

%%%%%% Collision avoidance %%%%%  
robot.animate(q1);
qWaypoints = [q1;q2];
isCollision = true;
checkedTillWaypoint = 1;
qMatrix = [];
while (isCollision)
    startWaypoint = checkedTillWaypoint;
    for i = startWaypoint:size(qWaypoints,1)-1
        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(60));
        if ~IsCollision(robot,qMatrixJoin,bigLegoFaces,bigLegoVertices,bigLegoFaceNormals)
            qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
%             robot.animate(qMatrixJoin);
            size(qMatrix)
            isCollision = false;
            checkedTillWaypoint = i+1;
            % Now try and join to the final goal (q2)
            q2 = robot.ikcon(transl(0.75, 0.68, 0.8)*troty(pi), qMatrix(i,:));
            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(60));
            if ~IsCollision(robot,qMatrixJoin,bigLegoFaces,bigLegoVertices,bigLegoFaceNormals)
                qMatrix = [qMatrix;qMatrixJoin];
                % Reached goal without collision, so break out
                break;
            end
        else
            % Randomly pick a pose that is not in collision
            randomXLocation = -0.85 + (1-(-0.85)).*rand(1);
            randomYLocation = -0.5 + (1-(-0.5)).*rand(1);
            randomZLocation = 0.5 + (1-(0.5)).*rand(1);
            qRand = robot.ikcon(transl(randomXLocation, randomYLocation, randomZLocation)*troty(pi), qWaypoints(i,:));
            qRand(4) = -pi/2;
            qRand(5) = -pi/2;
            while IsCollision(robot,qRand,bigLegoFaces,bigLegoVertices,bigLegoFaceNormals) %& qRand < abs(robot.qlim(:,1)) & z < 0.4
                randomXLocation = -0.85 + (1-(-0.85)).*rand(1);
                randomYLocation = -0.6 + (1-(-0.6)).*rand(1);
                randomZLocation = 0.5 + (1.5-(0.5)).*rand(1);
                qRand = robot.ikcon(transl(randomXLocation, randomYLocation, randomZLocation)*troty(pi), qMatrix(i,:));
                qRand(4) = -pi/2;
                qRand(5) = -pi/2;
            end
            qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
            isCollision = true;
            break;
        end
    end
end
robot.animate(qMatrix)    