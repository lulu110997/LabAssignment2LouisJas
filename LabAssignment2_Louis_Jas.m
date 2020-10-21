function varargout = LabAssignment2_Louis_Jas(varargin)
% LABASSIGNMENT2_LOUIS_JAS MATLAB code for LabAssignment2_Louis_Jas.fig
%      LABASSIGNMENT2_LOUIS_JAS, by itself, creates a new LABASSIGNMENT2_LOUIS_JAS or raises the existing
%      singleton*.
%
%      H = LABASSIGNMENT2_LOUIS_JAS returns the handle to a new LABASSIGNMENT2_LOUIS_JAS or the handle to
%      the existing singleton*.
%
%      LABASSIGNMENT2_LOUIS_JAS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LABASSIGNMENT2_LOUIS_JAS.M with the given input arguments.
%
%      LABASSIGNMENT2_LOUIS_JAS('Property','Value',...) creates a new LABASSIGNMENT2_LOUIS_JAS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LabAssignment2_Louis_Jas_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LabAssignment2_Louis_Jas_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES
% 1.570796326795
% 6.283185307180
% Edit the above text to modify the response to help LabAssignment2_Louis_Jas

% Last Modified by GUIDE v2.5 17-Oct-2020 02:37:51

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LabAssignment2_Louis_Jas_OpeningFcn, ...
                   'gui_OutputFcn',  @LabAssignment2_Louis_Jas_OutputFcn, ...
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


% --- Executes just before LabAssignment2_Louis_Jas is made visible.
function LabAssignment2_Louis_Jas_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LabAssignment2_Louis_Jas (see VARARGIN)
handles.ur10e = CreateUR10eModel([-3 3 -2.5 2 -0.25 2.5] ,0.5);
jointState = handles.ur10e.getpos;
Tr = handles.ur10e.fkine(jointState); %Change texts that displays the joint angles and xyz location of EE
set(handles.text8, 'String', num2str(jointState(1)));
set(handles.text10, 'String', num2str(jointState(2)));
set(handles.text11, 'String', num2str(jointState(3)));
set(handles.text12, 'String', num2str(jointState(4)));
set(handles.text13, 'String', num2str(jointState(5)));
set(handles.text14, 'String', num2str(jointState(6))); 
set(handles.text16, 'String', num2str(Tr(1,4)));
set(handles.text17, 'String', num2str(Tr(2,4)));
set(handles.text18, 'String', num2str(Tr(3,4)));

% Turn off all PB for VS
set(handles.pushbutton8, 'Enable', 'off') % DOWN PB
set(handles.pushbutton9, 'Enable', 'off') % LEFT PB
set(handles.pushbutton10, 'Enable', 'off') % RIGHT PB
set(handles.pushbutton11, 'Enable', 'off') % UP PB

handles.current_data = 'Select an action'; % Default selection for pop up menu

handles.estop = EmergencyStop; % Create an instance of the class for notifying when estop is active
lh = addlistener(handles.estop,'eStopActive',@HandleEmergencyStopState); % Add a listener

handles.dlt = 0; % For deleting obstacle
handles.humanEntering_h = PlaceObject('bigLego.ply', [-4.5, 0, 0.03]); % For simulated asynch sensor
handles.humanEnteringPose = transl(-3.5, 0, 0.03);
    
% Choose default command line output for LabAssignment2_Louis_Jas
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% UIWAIT makes LabAssignment2_Louis_Jas wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LabAssignment2_Louis_Jas_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
joint1Position = get(hObject, 'Value');
currentPosition = handles.ur10e.getpos;
currentPosition(1) = joint1Position;
handles.ur10e.animate(currentPosition);
set(handles.text8, 'String', num2str(currentPosition(1)));
eeLocation = handles.ur10e.fkine(handles.ur10e.getpos);
set(handles.text16, 'String', num2str(eeLocation(1,4)));
set(handles.text17, 'String', num2str(eeLocation(2,4)));
set(handles.text18, 'String', num2str(eeLocation(3,4)));
set(handles.slider19, 'Value', eeLocation(1,4))
set(handles.slider20, 'Value', eeLocation(2,4))
set(handles.slider21, 'Value', eeLocation(3,4))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
joint3Position = get(hObject, 'Value');
currentPosition = handles.ur10e.getpos;
currentPosition(3) = joint3Position;
handles.ur10e.animate(currentPosition);
set(handles.text11, 'String', num2str(currentPosition(3)));

eeLocation = handles.ur10e.fkine(handles.ur10e.getpos);
set(handles.text16, 'String', num2str(eeLocation(1,4)));
set(handles.text17, 'String', num2str(eeLocation(2,4)));
set(handles.text18, 'String', num2str(eeLocation(3,4)));
set(handles.slider19, 'Value', eeLocation(1,4))
set(handles.slider20, 'Value', eeLocation(2,4))
set(handles.slider21, 'Value', eeLocation(3,4))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
joint4Position = get(hObject, 'Value');
currentPosition = handles.ur10e.getpos;
currentPosition(4) = joint4Position;
handles.ur10e.animate(currentPosition);
set(handles.text12, 'String', num2str(currentPosition(4)));
eeLocation = handles.ur10e.fkine(handles.ur10e.getpos);
set(handles.text16, 'String', num2str(eeLocation(1,4)));
set(handles.text17, 'String', num2str(eeLocation(2,4)));
set(handles.text18, 'String', num2str(eeLocation(3,4)));
set(handles.slider19, 'Value', eeLocation(1,4))
set(handles.slider20, 'Value', eeLocation(2,4))
set(handles.slider21, 'Value', eeLocation(3,4))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
joint6Position = get(hObject, 'Value');
currentPosition = handles.ur10e.getpos;
currentPosition(6) = joint6Position;
handles.ur10e.animate(currentPosition);

set(handles.text14, 'String', num2str(currentPosition(6)));
eeLocation = handles.ur10e.fkine(handles.ur10e.getpos);
set(handles.text16, 'String', num2str(eeLocation(1,4)));
set(handles.text17, 'String', num2str(eeLocation(2,4)));
set(handles.text18, 'String', num2str(eeLocation(3,4)));
set(handles.slider19, 'Value', eeLocation(1,4))
set(handles.slider20, 'Value', eeLocation(2,4))
set(handles.slider21, 'Value', eeLocation(3,4))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider11_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
joint2Position = get(hObject, 'Value');
currentPosition = handles.ur10e.getpos;
currentPosition(2) = joint2Position;
set(handles.text10, 'String', num2str(currentPosition(2)));
handles.ur10e.animate(currentPosition);

eeLocation = handles.ur10e.fkine(handles.ur10e.getpos);
set(handles.text16, 'String', num2str(eeLocation(1,4)));
set(handles.text17, 'String', num2str(eeLocation(2,4)));
set(handles.text18, 'String', num2str(eeLocation(3,4)));
set(handles.slider19, 'Value', eeLocation(1,4))
set(handles.slider20, 'Value', eeLocation(2,4))
set(handles.slider21, 'Value', eeLocation(3,4))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider13_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
joint3Position = get(hObject, 'Value');
currentPosition = handles.ur10e.getpos;
currentPosition(3) = joint3Position;
handles.ur10e.animate(currentPosition);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider14_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
joint5Position = get(hObject, 'Value');
currentPosition = handles.ur10e.getpos;
currentPosition(5) = joint5Position;
handles.ur10e.animate(currentPosition);

set(handles.text13, 'String', num2str(currentPosition(5)));
eeLocation = handles.ur10e.fkine(handles.ur10e.getpos);
set(handles.text16, 'String', num2str(eeLocation(1,4)));
set(handles.text17, 'String', num2str(eeLocation(2,4)));
set(handles.text18, 'String', num2str(eeLocation(3,4)));
set(handles.slider19, 'Value', eeLocation(1,4))
set(handles.slider20, 'Value', eeLocation(2,4))
set(handles.slider21, 'Value', eeLocation(3,4))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.current_data == "pick and place"
    handles.boxes_h = PickAndPlace(handles.ur10e);
elseif handles.current_data == "obstacle detection and avoidance" 
    try 
        handles.boxesObs_h = PickAndPlaceWithCollisionDetecAndAvoid(...
        handles.ur10e, handles.humanObstacle_h, get(handles.humanObstacle_h));
    catch
        mydlg = errordlg(['No obstacle in place'],'Error101')
        waitfor(mydlg)
    end
elseif handles.current_data == "visual servoing"
    jointForCameraPos = [0 -2.1437 1.7866 -2.7846 -1.5706 0];
    handles.ur10e.delay = 0.01;
    handles.ur10e.animate(jtraj(handles.ur10e.getpos, jointForCameraPos,50));
    handles.stopSign_h = PlaceObject('stopSign.ply', [-1.1, -0.1741, 1.2133]);
    handles.stopSignPose = transl(-1.1, -0.1741, 1.2133);
    handles.cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
        'resolution', [1024 1024], 'centre', [512 512],'name', 'testCamera');
    Tc0 = handles.ur10e.fkine(jointForCameraPos);
    handles.cam.plot_camera('Tcam',Tc0, 'label','scale',0.15); % Plots camera

else 
    f = errordlg('You must select an action','Error 101');
end

% Save the handles structure.
guidata(hObject,handles)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Determine the selected data set.
str = get(hObject, 'String');
val = get(hObject,'Value');
% Set current data to the selected data set.
switch str{val}
    case 'pick and place'
        set(handles.pushbutton8, 'Enable', 'off') % DOWN PB
        set(handles.pushbutton9, 'Enable', 'off') % LEFT PB
        set(handles.pushbutton10, 'Enable', 'off') % RIGHT PB
        set(handles.pushbutton11, 'Enable', 'off') % UP PB
        handles.current_data = 'pick and place';
    case 'obstacle detection and avoidance'
        set(handles.pushbutton8, 'Enable', 'off') % DOWN PB
        set(handles.pushbutton9, 'Enable', 'off') % LEFT PB
        set(handles.pushbutton10, 'Enable', 'off') % RIGHT PB
        set(handles.pushbutton11, 'Enable', 'off') % UP PB
        handles.current_data = 'obstacle detection and avoidance';
    case 'visual servoing'
        set(handles.pushbutton8, 'Enable', 'on') % DOWN PB
        set(handles.pushbutton9, 'Enable', 'on') % LEFT PB
        set(handles.pushbutton10, 'Enable', 'on') % RIGHT PB
        set(handles.pushbutton11, 'Enable', 'on') % UP PB
        handles.current_data = 'visual servoing';
    otherwise
        set(handles.pushbutton8, 'Enable', 'off') % DOWN PB
        set(handles.pushbutton9, 'Enable', 'off') % LEFT PB
        set(handles.pushbutton10, 'Enable', 'off') % RIGHT PB
        set(handles.pushbutton11, 'Enable', 'off') % UP PB
        handles.current_data = 'Select an action';
end
% Save the handles structure.
guidata(hObject,handles)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
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


% --- Executes on button press in togglebutton1. E-stop code goes here
function togglebutton1_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value') == 1
    set(handles.togglebutton2, 'Value', 1); % Flag must be zero to resume operation
    % Disable all sliders
    set(handles.slider1, 'Enable', 'off')
    set(handles.slider2, 'Enable', 'off')
    set(handles.slider3, 'Enable', 'off')
    set(handles.slider4, 'Enable', 'off')
    set(handles.slider5, 'Enable', 'off')
    set(handles.slider6, 'Enable', 'off')
    set(handles.slider19, 'Enable', 'off')
    set(handles.slider20, 'Enable', 'off')
    set(handles.slider21, 'Enable', 'off')
    set(handles.slider26, 'Enable', 'off') % Slider for human entering
    % Disable all buttons except toggle button
    set(handles.pushbutton5, 'Enable', 'off')
    set(handles.pushbutton1, 'Enable', 'off')
    set(handles.pushbutton6, 'Enable', 'off')
    set(handles.pushbutton8, 'Enable', 'off') % DOWN PB
    set(handles.pushbutton9, 'Enable', 'off') % LEFT PB
    set(handles.pushbutton10, 'Enable', 'off') % RIGHT PB
    set(handles.pushbutton11, 'Enable', 'off') % UP PB
    set(handles.pushbutton12, 'Enable', 'off') % remove obstacle PB
    set(handles.togglebutton2, 'Enable', 'off') % Resume button 
    triggerEvent(handles.estop)
    waitfor(hObject, 'Value', 0);
    % Enable PB
    while(1)
        set(handles.togglebutton2, 'Enable', 'on') % Resume button 
        waitfor(handles.togglebutton2, 'Value', 0)
        break
    end
    % Enable everything
    set(handles.slider1, 'Enable', 'on')
    set(handles.slider2, 'Enable', 'on')
    set(handles.slider3, 'Enable', 'on')
    set(handles.slider4, 'Enable', 'on')
    set(handles.slider5, 'Enable', 'on')
    set(handles.slider6, 'Enable', 'on')
    set(handles.slider19, 'Enable', 'on')
    set(handles.slider20, 'Enable', 'on')
    set(handles.slider21, 'Enable', 'on')
    set(handles.pushbutton5, 'Enable', 'on')
    set(handles.pushbutton1, 'Enable', 'on')
    set(handles.pushbutton6, 'Enable', 'on')
    set(handles.pushbutton12, 'Enable', 'on') % Obstacle PB
    set(handles.slider26, 'Enable', 'on') % Slider for human entering
    
    
end
% Save the handles structure.
guidata(hObject,handles)
% Hint: get(hObject,'Value') returns toggle state of togglebutton1


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over slider1.
function slider1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider19_Callback(hObject, eventdata, handles)
% hObject    handle to slider19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x = get(hObject, 'Value');
xyz = ChangeEndEffectorLocation(handles.ur10e, 'x', x);
set(handles.text16, 'String', num2str(xyz(1)));
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider19_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider20_Callback(hObject, eventdata, handles)
% hObject    handle to slider20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
y = get(hObject, 'Value');
xyz = ChangeEndEffectorLocation(handles.ur10e, 'y', y);
set(handles.text17, 'String', num2str(xyz(2)));
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider21_Callback(hObject, eventdata, handles)
% hObject    handle to slider21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
z = get(hObject, 'Value');
xyz = ChangeEndEffectorLocation(handles.ur10e, 'z', z);
set(handles.text18, 'String', num2str(xyz(3)));
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider22_Callback(hObject, eventdata, handles)
% hObject    handle to slider22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try 
    delete(handles.humanObstacle_h)
catch
end 
x = get(handles.edit2, 'String');
y = get(handles.edit3, 'String');
z = get(handles.text28, 'String');
handles.humanObstacle_h = PlaceObject('bigLego.ply', [str2num(x) ...    
    str2num(y) str2num(z)]);
guidata(hObject, handles); % Needed to save the handles into the gui/update handles value (or smth like that)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.ur10e.delay = 0.01;
handles.ur10e.animate(jtraj(handles.ur10e.getpos, [0 -pi/2 pi/2 -pi/2 -pi/2 0], 100));
jointState = handles.ur10e.getpos;
Tr = handles.ur10e.fkine(jointState);

%Restart sliders
set(handles.slider1, 'Value', jointState(1));
set(handles.slider2, 'Value', jointState(2));
set(handles.slider3, 'Value', jointState(3));
set(handles.slider4, 'Value', jointState(4));
set(handles.slider5, 'Value', jointState(5));
set(handles.slider6, 'Value', jointState(6));
set(handles.slider19, 'Value', Tr(1,4));
set(handles.slider20, 'Value', Tr(2,4));
set(handles.slider21, 'Value', Tr(3,4));

% Restart text values
set(handles.text8, 'String', num2str(jointState(1)));
set(handles.text10, 'String', num2str(jointState(2)));
set(handles.text11, 'String', num2str(jointState(3)));
set(handles.text12, 'String', num2str(jointState(4)));
set(handles.text13, 'String', num2str(jointState(5)));
set(handles.text14, 'String', num2str(jointState(6))); 
set(handles.text16, 'String', num2str(Tr(1,4)));
set(handles.text17, 'String', num2str(Tr(2,4)));
set(handles.text18, 'String', num2str(Tr(3,4))); 


% --- Executes during object creation, after setting all properties.
function text17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
stopSignVertexCount = size(get(handles.stopSign_h,'Vertices'),1);

midPoint = (sum(get(handles.stopSign_h,'Vertices'))/stopSignVertexCount);
boxVerts = (get(handles.stopSign_h,'Vertices') - repmat(midPoint,stopSignVertexCount,1));

% Move the stop sign back (-z)
downwardsTR = makehgtform('translate',[0, 0, -0.2]);
handles.stopSignPose = handles.stopSignPose * downwardsTR;
updatedPoints = [handles.stopSignPose * [boxVerts,ones(stopSignVertexCount,1)]']';
set(handles.stopSign_h,'Vertices', updatedPoints(:,1:3)); 
drawnow;
set(handles.pushbutton8, 'Enable', 'off') % DOWN PB
set(handles.pushbutton9, 'Enable', 'off') % LEFT PB
set(handles.pushbutton10, 'Enable', 'off') % RIGHT PB
set(handles.pushbutton11, 'Enable', 'off') % UP PB
RetreatFromSafetySymbol(handles.ur10e, handles.cam, handles.stopSign_h);
set(handles.pushbutton8, 'Enable', 'on') % DOWN PB
set(handles.pushbutton9, 'Enable', 'on') % LEFT PB
set(handles.pushbutton10, 'Enable', 'on') % RIGHT PB
set(handles.pushbutton11, 'Enable', 'on') % UP PB
display('Finished moving -z')
% Save the handles structure.
guidata(hObject,handles)


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
stopSignVertexCount = size(get(handles.stopSign_h,'Vertices'),1);

midPoint = (sum(get(handles.stopSign_h,'Vertices'))/stopSignVertexCount);
boxVerts = (get(handles.stopSign_h,'Vertices') - repmat(midPoint,stopSignVertexCount,1));

% Move the stop sign back (-x)
upwardsTR = makehgtform('translate',[-0.15, 0, 0]);
handles.stopSignPose = handles.stopSignPose * upwardsTR;
updatedPoints = [handles.stopSignPose * [boxVerts,ones(stopSignVertexCount,1)]']';
set(handles.stopSign_h,'Vertices', updatedPoints(:,1:3)); 
drawnow;
set(handles.pushbutton8, 'Enable', 'off') % DOWN PB
set(handles.pushbutton9, 'Enable', 'off') % LEFT PB
set(handles.pushbutton10, 'Enable', 'off') % RIGHT PB
set(handles.pushbutton11, 'Enable', 'off') % UP PB
RetreatFromSafetySymbol(handles.ur10e, handles.cam, handles.stopSign_h);
set(handles.pushbutton8, 'Enable', 'on') % DOWN PB
set(handles.pushbutton9, 'Enable', 'on') % LEFT PB
set(handles.pushbutton10, 'Enable', 'on') % RIGHT PB
set(handles.pushbutton11, 'Enable', 'on') % UP PB
disp('done moving to -x')
% Save the handles structure.
guidata(hObject,handles)


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
stopSignVertexCount = size(get(handles.stopSign_h,'Vertices'),1);

midPoint = (sum(get(handles.stopSign_h,'Vertices'))/stopSignVertexCount);
boxVerts = (get(handles.stopSign_h,'Vertices') - repmat(midPoint,stopSignVertexCount,1));

% Move the stop sign forward (+x)
upwardsTR = makehgtform('translate', [0.15, 0, 0]);
handles.stopSignPose = handles.stopSignPose * upwardsTR;
updatedPoints = [handles.stopSignPose * [boxVerts,ones(stopSignVertexCount,1)]']';
set(handles.stopSign_h,'Vertices', updatedPoints(:,1:3)); 
drawnow;
set(handles.pushbutton8, 'Enable', 'off') % DOWN PB
set(handles.pushbutton9, 'Enable', 'off') % LEFT PB
set(handles.pushbutton10, 'Enable', 'off') % RIGHT PB
set(handles.pushbutton11, 'Enable', 'off') % UP PB
RetreatFromSafetySymbol(handles.ur10e, handles.cam, handles.stopSign_h);
set(handles.pushbutton8, 'Enable', 'on') % DOWN PB
set(handles.pushbutton9, 'Enable', 'on') % LEFT PB
set(handles.pushbutton10, 'Enable', 'on') % RIGHT PB
set(handles.pushbutton11, 'Enable', 'on') % UP PB
disp('done moving to +x')
% Save the handles structure.
guidata(hObject,handles)

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get vertex count
stopSignVertexCount = size(get(handles.stopSign_h,'Vertices'),1);

midPoint = (sum(get(handles.stopSign_h,'Vertices'))/stopSignVertexCount);
boxVerts = (get(handles.stopSign_h,'Vertices') - repmat(midPoint,stopSignVertexCount,1));

% Move the stop sign up (+z)
upwardsTR = makehgtform('translate',[0,0,0.2]);
handles.stopSignPose = handles.stopSignPose * upwardsTR;
updatedPoints = [handles.stopSignPose * [boxVerts,ones(stopSignVertexCount,1)]']';
set(handles.stopSign_h,'Vertices', updatedPoints(:,1:3));
drawnow;
    set(handles.pushbutton8, 'Enable', 'off') % DOWN PB
    set(handles.pushbutton9, 'Enable', 'off') % LEFT PB
    set(handles.pushbutton10, 'Enable', 'off') % RIGHT PB
    set(handles.pushbutton11, 'Enable', 'off') % UP PB
RetreatFromSafetySymbol(handles.ur10e, handles.cam, handles.stopSign_h);
    set(handles.pushbutton8, 'Enable', 'on') % DOWN PB
    set(handles.pushbutton9, 'Enable', 'on') % LEFT PB
    set(handles.pushbutton10, 'Enable', 'on') % RIGHT PB
    set(handles.pushbutton11, 'Enable', 'on') % UP PB
disp('finish moving +z')
% Save the handles structure.
guidata(hObject,handles)


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    delete(handles.humanObstacle_h)
catch 
end
try     
    delete(handles.boxes_h{1:size(handles.boxes_h,2)})
catch
end

% Save the handles structure.
guidata(hObject,handles)



% --- Executes on slider movement.
function slider26_Callback(hObject, eventdata, handles)
% hObject    handle to slider26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
humanEnteringVertexCount = size(get(handles.humanEntering_h,'Vertices'),1);

midPoint = (sum(get(handles.humanEntering_h,'Vertices')))/humanEnteringVertexCount;
humanEnteringVerts = (get(handles.humanEntering_h,'Vertices') - repmat(midPoint,humanEnteringVertexCount,1));

% Move the human in x axis direction
x = get(hObject,'Value');
moveInXtr = makehgtform('translate',[x, 0, 0.74]);
% handles.humanEnteringPose = handles.humanEnteringPose*moveInXtr; disp(handles.humanEnteringPose)
updatedPoints = [moveInXtr * [humanEnteringVerts,ones(humanEnteringVertexCount,1)]']';
set(handles.humanEntering_h,'Vertices', updatedPoints(:,1:3)); 
set(handles.text30, 'String', num2str(x)) 

if x > -2.4
	set(handles.slider1, 'Enable', 'off')
    set(handles.slider2, 'Enable', 'off')
    set(handles.slider3, 'Enable', 'off')
    set(handles.slider4, 'Enable', 'off')
    set(handles.slider5, 'Enable', 'off')
    set(handles.slider6, 'Enable', 'off')
    set(handles.slider19, 'Enable', 'off')
    set(handles.slider20, 'Enable', 'off')
    set(handles.slider21, 'Enable', 'off')
    % Disable all buttons except toggle button
    set(handles.pushbutton5, 'Enable', 'off')
    set(handles.pushbutton1, 'Enable', 'off')
    set(handles.pushbutton6, 'Enable', 'off')
    set(handles.pushbutton8, 'Enable', 'off') % DOWN PB
    set(handles.pushbutton9, 'Enable', 'off') % LEFT PB
    set(handles.pushbutton10, 'Enable', 'off') % RIGHT PB
    set(handles.pushbutton11, 'Enable', 'off') % UP PB
    set(handles.pushbutton12, 'Enable', 'off') % remove obstacle PB
    set(handles.togglebutton1, 'Enable', 'off') % TB for estop
    mydlg = warndlg('Safety sensor has been tripped. Please remove object in the vicinity.','Warning');
    waitfor(mydlg);
    
    while 1
        if x < -2.4
            break
        end
        x = get(hObject,'Value');
        moveInXtr = makehgtform('translate',[x, 0, 0.74]);
        % handles.humanEnteringPose = handles.humanEnteringPose*moveInXtr; disp(handles.humanEnteringPose)
        updatedPoints = [moveInXtr * [humanEnteringVerts,ones(humanEnteringVertexCount,1)]']';
        set(handles.humanEntering_h,'Vertices', updatedPoints(:,1:3));
        pause(1)
    end
    % Enable everything
    set(handles.slider1, 'Enable', 'on')
    set(handles.slider2, 'Enable', 'on')
    set(handles.slider3, 'Enable', 'on')
    set(handles.slider4, 'Enable', 'on')
    set(handles.slider5, 'Enable', 'on')
    set(handles.slider6, 'Enable', 'on')
    set(handles.slider19, 'Enable', 'on')
    set(handles.slider20, 'Enable', 'on')
    set(handles.slider21, 'Enable', 'on')
    set(handles.pushbutton5, 'Enable', 'on')
    set(handles.pushbutton1, 'Enable', 'on')
    set(handles.pushbutton6, 'Enable', 'on')
    set(handles.pushbutton12, 'Enable', 'on') % Obstacle PB
    set(handles.togglebutton1, 'Enable', 'on') % TB for estop
end

guidata(hObject, handles); 
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider26_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in resume.
function resume_Callback(hObject, eventdata, handles)
% hObject    handle to resume (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.eFlag = 0;
% Save the handles structure.
guidata(hObject,handles)


% --- Executes on button press in togglebutton2.
function togglebutton2_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton2
