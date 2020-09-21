function varargout = D1ExtensionQuestion(varargin)
% D1EXTENSIONQUESTION MATLAB code for D1ExtensionQuestion.fig
%      D1EXTENSIONQUESTION, by itself, creates a new D1EXTENSIONQUESTION or raises the existing
%      singleton*.
%
%      H = D1EXTENSIONQUESTION returns the handle to a new D1EXTENSIONQUESTION or the handle to
%      the existing singleton*.
%
%      D1EXTENSIONQUESTION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in D1EXTENSIONQUESTION.M with the given input arguments.
%
%      D1EXTENSIONQUESTION('Property','Value',...) creates a new D1EXTENSIONQUESTION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before D1ExtensionQuestion_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to D1ExtensionQuestion_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help D1ExtensionQuestion

% Last Modified by GUIDE v2.5 30-Aug-2020 23:34:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @D1ExtensionQuestion_OpeningFcn, ...
                   'gui_OutputFcn',  @D1ExtensionQuestion_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before D1ExtensionQuestion is made visible.
function D1ExtensionQuestion_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to D1ExtensionQuestion (see VARARGIN)

% Choose default command line output for D1ExtensionQuestion
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using D1ExtensionQuestion.
if strcmp(get(hObject,'Visible'),'off')
    plot(rand(5));
end

% UIWAIT makes D1ExtensionQuestion wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = D1ExtensionQuestion_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes1);
cla;
brickLocation = {1};
baseLocationInput = inputdlg('Enter xyz base location separated by spaces:',...
    'Sample', [1 50]);
baseLocation = [str2num(baseLocationInput{1}) 1]'
L1 = Link('d', 0.1519, 'a', 0, 'alpha', pi/2, 'qlim', [-2*pi, 2*pi]);
L2 = Link('d', 0,'a', -0.24365, 'alpha', 0, 'qlim', [deg2rad(-360), deg2rad(360)]);
L3 = Link('d', 0, 'a', -0.21325, 'alpha', 0, 'qlim', [deg2rad(-360), deg2rad(360)]);
L4 = Link('d', 0.11235, 'a', 0, 'alpha', pi/2, 'qlim', [-2*pi, 2*pi]);
L5 = Link('d', 0.08535, 'a', 0, 'alpha', -pi/2, 'qlim', [-2*pi, 2*pi]);
L6 = Link('d', 0.0819, 'a', 0, 'alpha', 0, 'qlim', [-2*pi, 2*pi]);

rotateZ = trotz(pi/2);
fixBase = [rotateZ(1:4, 1:3) baseLocation]; % Fix robot base rotation

ur3Robot = SerialLink([L1 L2 L3 L4 L5 L6],'name', 'ur3', 'base', fixBase);

for linkIndex = 0:ur3Robot.n
    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR3Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
    ur3Robot.faces{linkIndex+1} = faceData;
    ur3Robot.points{linkIndex+1} = vertexData;
end

% Display robot
workspace = [(baseLocation(1,1) - 1.5) (baseLocation(1,1) + 1.5) ...
    (baseLocation(2,1) - 1.5) (baseLocation(2,1) + 1.5) ...
    (baseLocation(3,1) - 0.5) (baseLocation(3,1) + 1)];
ur3Robot.plot3d(zeros(1,ur3Robot.n),'noarrow','workspace',workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
ur3Robot.delay = 0;
hold on

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:ur3Robot.n
    handles = findobj('Tag', ur3Robot.name);
    h = get(handles,'UserData');
    try
        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
            , plyData{linkIndex+1}.vertex.green ...
            , plyData{linkIndex+1}.vertex.blue]/255;
        h.link(linkIndex+1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end

% Brick placement code
for i = 1:9
    brickDropOffLocationInput = inputdlg('Enter xyz location for bricks separated by spaces:',...
        'Sample', [1 50]);
    brickLocation{i} = [str2num(brickDropOffLocationInput{1})];
    clc
    if i == 9
        disp('bricks have been placed in the environment')
        brick9_h = PlaceObject('brick.ply', brickLocation{i});
    elseif i == 8; brick8_h = PlaceObjectWithRotation(brickDropOffLocationInput{i});
    elseif i == 7; brick7_h = PlaceObjectWithRotation(brickDropOffLocationInput{i});
    elseif i == 6; brick6_h = PlaceObjectWithRotation(brickDropOffLocationInput{i});
    elseif i == 5; brick5_h = PlaceObjectWithRotation(brickDropOffLocationInput{i});
    elseif i == 4; brick4_h = PlaceObjectWithRotation(brickDropOffLocationInput{i});
    elseif i == 3; brick3_h = PlaceObjectWithRotation(brickDropOffLocationInput{i});
    elseif i == 2; brick2_h = PlaceObjectWithRotation(brickDropOffLocationInput{i});
    elseif i == 1; brick1_h = PlaceObjectWithRotation(brickDropOffLocationInput{i});
        
    end
end

% Drop off location code
for i = 1:9
    brickLocationInput = inputdlg('Enter xyz location for bricks separated by spaces:',...
        'Sample', [1 50]);
    brickLocation{i} = [str2num(brickLocationInput{1})];
    clc
    if i == 9
        disp('bricks have been placed in the environment')
        brick9_h = PlaceObject('brick.ply', brickLocation{i});
    elseif i == 8; brick8_h = PlaceObject('brick.ply', brickLocation{i});
    elseif i == 7; brick7_h = PlaceObject('brick.ply', brickLocation{i});
    elseif i == 6; brick6_h = PlaceObject('brick.ply', brickLocation{i});
    elseif i == 5; brick5_h = PlaceObject('brick.ply', brickLocation{i});
    elseif i == 4; brick4_h = PlaceObject('brick.ply', brickLocation{i});
    elseif i == 3; brick3_h = PlaceObject('brick.ply', brickLocation{i});
    elseif i == 2; brick2_h = PlaceObject('brick.ply', brickLocation{i});
    elseif i == 1; brick1_h = PlaceObject('brick.ply', brickLocation{i});
        
    end
end


data = guidata(hObject);
data.model = ur3Robot;
guidata(hObject,data);


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1
