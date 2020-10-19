function ur10e = CreateUR10eModel(workspace, scale)
tic
% nargin statement for testing
if nargin ~= 2
    close
    clear
    clc
%     workspace = [-3 3 -2.5 2 -0.25 2.5];
    workspace = [-2 1.45 -1.45 1.45 -0.25 2.75];
    scale = 0.5;
    disp('closed, cleared, clcd')
end

% DH parameters
L1 = Link('d', 0.1807, 'a', 0, 'alpha', pi/2, 'qlim', [-2*pi, 2*pi]);
L2 = Link('d', 0,'a', -0.6127, 'alpha', 0, 'qlim', [deg2rad(-360), deg2rad(360)]);
L3 = Link('d', 0, 'a', -0.57155, 'alpha', 0, 'qlim', [deg2rad(-360), deg2rad(360)]);
L4 = Link('d', 0.17415, 'a', 0, 'alpha', pi/2, 'qlim', [-2*pi, 2*pi]);
L5 = Link('d', 0.11985, 'a', 0, 'alpha', -pi/2, 'qlim', [-2*pi, 2*pi]);
L6 = Link('d', (0.11655+0.2), 'a', 0, 'alpha', 0, 'qlim', [-2*pi, 2*pi]); % Added 0.2 for the EE

ur10eModel = SerialLink([L1 L2 L3 L4 L5 L6],'name','ur10e');

%%%%%%%%%%%%%%%%%%%%%%%% SKIN ON %%%%%%%%%%%%%%%%%%%%%%%%
% ur10eModel.base = ur10eModel.base*transl(0.22, 0 ,0.3);
% ur10eModel.delay = 0;
% % Read ply files for robot model
% for linkIndex = 0:ur10eModel.n
%     [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR10eLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
%     ur10eModel.faces{linkIndex+1} = faceData;
%     ur10eModel.points{linkIndex+1} = vertexData;
%     disp(linkIndex)
% end
% 
% % Display robot
% ur10eModel.plotopt3d = {'wrist', 'xyz', 'arrow'};
% ur10eModel.plot3d(1.0*[0 -pi/2 pi/2 -pi/2 -pi/2 0], 'workspace', workspace, 'scale', scale);
% if isempty(findobj(get(gca,'Children'),'Type','Light'))
%     camlight
% end
% 
% % Try to correctly colour the arm (if colours are in ply file data)
% for linkIndex = 0:ur10eModel.n
%     handles = findobj('Tag', ur10eModel.name);
%     h = get(handles,'UserData');
%     try
%         h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
%             , plyData{linkIndex+1}.vertex.green ...
%             , plyData{linkIndex+1}.vertex.blue]/255;
%         h.link(linkIndex+1).Children.FaceColor = 'interp';
%     catch ME_1
%         disp(ME_1);
%         continue;
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%% SKIN ON %%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%% NO SKIN %%%%%%%%%%%%%%%%%%%%%%%%
ur10eModel.plotopt = {'notiles', 'nobase', 'noshadow', 'noname'};
ur10eModel.base = ur10eModel.base*transl(0.22, 0 ,0.3);
ur10eModel.plot([0 -pi/2 pi/2 -pi/2 -pi/2 0],'workspace',workspace, 'scale', scale);
ur10eModel.delay = 0;
%%%%%%%%%%%%%%%%%%%%%%%% NO SKIN %%%%%%%%%%%%%%%%%%%%%%%%

ur10e = ur10eModel(); hold on
PlaceObject('pallet.ply', [-0.6374, 0, 0.1]); % z = 0.1 for box on pallet. originally 0.06807
% PlaceObject('safetyfence.ply', [ 0 0 0.03]); % 0.03 so the ground touches the pallet when it's @ 0.1
PlaceObject('table.ply', [0.75, 0.7, 0.09]); % 0.09 when pallet @ 0.1
PlaceObject('base.ply', [0.22, 0 ,0.05]);
% view([0, 0])

%%%%% testing %%%%%%%%%%%%%%%%%%%
% PlaceObject('louisBox.ply', [0.75, 0.68, 0.64]);
% PlaceObject('jasBox.ply', [0.75 0.68 0.59]);
% ur10eModel.teach;

toc
end