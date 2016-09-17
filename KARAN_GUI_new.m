function varargout = KARAN_GUI_new(varargin)
% KARAN_GUI_new M-file for KARAN_GUI_new.fig
%      KARAN_GUI_new, by itself, creates a new KARAN_GUI_new or raises the existing
%      singleton*.
%
%      H = KARAN_GUI_new returns the handle to a new KARAN_GUI_new or the handle to
%      the existing singleton*.
%
%      KARAN_GUI_new('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in KARAN_GUI_new.M with the given input arguments.
%
%      KARAN_GUI_new('Property','Value',...) creates a new KARAN_GUI_new or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before KARAN_GUI_new_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to KARAN_GUI_new_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help KARAN_GUI_new

% Last Modified by GUIDE v2.5 05-Aug-2011 01:32:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @KARAN_GUI_new_OpeningFcn, ...
                   'gui_OutputFcn',  @KARAN_GUI_new_OutputFcn, ...
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


% --- Executes just before KARAN_GUI_new is made visible.
function KARAN_GUI_new_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to KARAN_GUI_new (see VARARGIN)

% Choose default command line output for KARAN_GUI_new
handles.output = hObject;
% data.number= 0;
% set(handles.videoselect,'UserData',data)
% setappdata(hObject,'Push Button',slider_data);
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes KARAN_GUI_new wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = KARAN_GUI_new_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in kalman.
function kalman_Callback(hObject, eventdata, handles)
% hObject    handle to kalman (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
R=[[0.0045,0.0045]',[0.0045,0.0455]'];
H=[[1,0]',[0,1]',[0,0]',[0,0]'];

num1 = str2double(get(handles.QNOISE,'String')); 

num2 = str2double(get(handles.PNOISE,'String')); 

Q = num1*eye(4);

P = num2*eye(4);
% Q=0.01*eye(4);
% P = 10*eye(4);
dt=.2;
A=[[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];
g = 1; % pixels^2/time step
Bu = [0,0,0,dt*g]';
kfinit=0;
%a=getframe(a);
global avi_file;
for i = 1 :size(avi_file,2)
imshow(avi_file(1,i).cdata,'Parent',handles.IMAGE1);
pause(.001);


    tic;
    frame=ind2rgb(avi_file(1,i).cdata,avi_file(1,i).colormap);
    %image(frame,'Parent',handles.axes1);
    axis off;
    frame_rgb = rgb2gray(frame);
    
    filtertype = get(handles.filter, 'String');   
    
    switch filtertype
        
        case 'Mean Filter'
    
    h = fspecial('average');
    frame_rgb= imfilter(frame_rgb,h);
     
    
     thresh=graythresh(frame_rgb);
   bw=~(im2bw(frame_rgb,thresh));
    labeled = bwlabel( bw,8);
  stats = regionprops(labeled,['basic']);%basic mohem nist
  [N,W] = size(stats);
  if N < 1
    return   
  end

  % do bubble sort (large to small) on regions in case there are more than 1
  id = zeros(N);
  for k = 1 : N
    id(k) = k;
  end
  for k = 1 : N-1
    for j = k+1 : N
      if stats(k).Area < stats(j).Area
        tmp = stats(k);
        stats(k) = stats(j);
        stats(j) = tmp;
        tmp = id(k);
        id(k) = id(j);
        id(j) = tmp;
      end
    end
  end
  selected = (labeled==id(2));

  % get center of mass and radius of largest
 k = stats(2).Centroid;
%     frame_rgb= imfilter(frame_rgb,h);
%     thresh=graythresh(frame_rgb);
%      bw=~(im2bw(frame_rgb,thresh));
%     bw = BWMORPH(bw,'clean')
%         gdata=regionprops(bw,'area');
%         idx = find([gdata.Area] > 100);
%              BW2 = ismember(bw, idx);
%              [l num]=bwlabel(BW2,4);
%              CC = bwconncomp(BW2);
%               L = labelmatrix(CC);
%              gdata1=regionprops(L==2,'centroid');
%     %imshow(l==3,'Parent',handles.axes3);
%     %set(handles.edit2,'String',num2str(num));
    imshow(bw,'Parent',handles.IMAGE2);
    %k=gdata1.Centroid;
    imshow(frame_rgb,'Parent',handles.IMAGE3);
    hold on;
    plot(k(1),k(2),'b*');
    hold on;
   
if kfinit==0
    xp = [0,0,0,0]';
  else
    xp=A*x(i-1,:)' + Bu; % predict phase
  end
  kfinit=1;
  PP = A*P*A' + Q;
  K = PP*H'*inv(H*PP*H'+R);
  x(i,:) = (xp + K*([k(1),k(2)]' - H*xp))';%update
     P = (eye(4)-K*H)*PP;
     measure=norm(P);
    tme=toc;
   plot(x(i,1),x(i,2),'r*');
   err1=k(1)-x(i,1);
   err2=k(2)-x(i,2);

   hold off;
   
   set(handles.centroidX,'String',num2str(k(1)));
    set(handles.centroidY,'String',num2str(k(2)));
    set(handles.TSTATUS,'String','KALMAN TRACKING');
    set(handles.TFRAME,'String',num2str(tme));
     set(handles.FRAME,'String',num2str(i));
     set(handles.ERRORCX,'String',num2str(measure));
     set(handles.ERRORCY,'String',num2str(measure));
     
    set(handles.ERRORX,'String',num2str(err1));
    set(handles.ERRORY,'String',num2str(err2));
    pause(.001);
    
    
    
        case 'Gaussian Filter'
            
             h = fspecial('gaussian');
    frame_rgb= imfilter(frame_rgb,h);
     imshow(frame_rgb,'Parent',handles.IMAGE3);
    
     thresh=graythresh(frame_rgb);
   bw=~(im2bw(frame_rgb,thresh));
    labeled = bwlabel( bw,8);
  stats = regionprops(labeled,['basic']);%basic mohem nist
  [N,W] = size(stats);
  if N < 1
    return   
  end

  % do bubble sort (large to small) on regions in case there are more than 1
  id = zeros(N);
  for k = 1 : N
    id(k) = k;
  end
  for k = 1 : N-1
    for j = k+1 : N
      if stats(k).Area < stats(j).Area
        tmp = stats(k);
        stats(k) = stats(j);
        stats(j) = tmp;
        tmp = id(k);
        id(k) = id(j);
        id(j) = tmp;
      end
    end
  end
  selected = (labeled==id(2));

  % get center of mass and radius of largest
 k = stats(2).Centroid;
%     frame_rgb= imfilter(frame_rgb,h);
%     thresh=graythresh(frame_rgb);
%      bw=~(im2bw(frame_rgb,thresh));
%     bw = BWMORPH(bw,'clean')
%         gdata=regionprops(bw,'area');
%         idx = find([gdata.Area] > 100);
%              BW2 = ismember(bw, idx);
%              [l num]=bwlabel(BW2,4);
%              CC = bwconncomp(BW2);
%               L = labelmatrix(CC);
%              gdata1=regionprops(L==2,'centroid');
%     %imshow(l==3,'Parent',handles.axes3);
%     %set(handles.edit2,'String',num2str(num));
    imshow(bw,'Parent',handles.IMAGE2);
    %k=gdata1.Centroid;
    hold on;
    plot(k(1),k(2),'b*');
    hold on;
   
if kfinit==0
    xp = [0,0,0,0]';
  else
    xp=A*x(i-1,:)' + Bu; % predict phase
  end
  kfinit=1;
  PP = A*P*A' + Q;
  K = PP*H'*inv(H*PP*H'+R);
  x(i,:) = (xp + K*([k(1),k(2)]' - H*xp))';%update
     P = (eye(4)-K*H)*PP;
    tme=toc;
   plot(x(i,1),x(i,2),'r*');
   err1=k(1)-x(i,1);
   err2=k(2)-x(i,2);
   hold on;
    set(handles.centroidX,'String',num2str(k(1)));
    set(handles.centroidY,'String',num2str(k(2)));
    set(handles.TSTATUS,'String','Object is being tracked through Kalman Filter');
    set(handles.TFRAME,'String',num2str(tme));
     set(handles.FRAME,'String',num2str(i));
    set(handles.ERRORX,'String',num2str(err1));
    set(handles.ERRORY,'String',num2str(err2));
    pause(.001);
            
            
            
            

    end
end

%set(handles.edit2,'String','File has been succesfully imported')
%msgbox('File has been succesfully imported','Status','help')
% --- Executes on button press in pushbutton2.


function QNOISE_Callback(hObject, eventdata, handles)
% hObject    handle to QNOISE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of QNOISE as text
%        str2double(get(hObject,'String')) returns contents of QNOISE as a double


% --- Executes during object creation, after setting all properties.
function QNOISE_CreateFcn(hObject, eventdata, handles)
% hObject    handle to QNOISE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PNOISE_Callback(hObject, eventdata, handles)
% hObject    handle to PNOISE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PNOISE as text
%        str2double(get(hObject,'String')) returns contents of PNOISE as a double


% --- Executes during object creation, after setting all properties.
function PNOISE_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PNOISE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ERRORCX_Callback(hObject, eventdata, handles)
% hObject    handle to ERRORCX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ERRORCX as text
%        str2double(get(hObject,'String')) returns contents of ERRORCX as a double


% --- Executes during object creation, after setting all properties.
function ERRORCX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ERRORCX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ERRORCY_Callback(hObject, eventdata, handles)
% hObject    handle to ERRORCY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ERRORCY as text
%        str2double(get(hObject,'String')) returns contents of ERRORCY as a double


% --- Executes during object creation, after setting all properties.
function ERRORCY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ERRORCY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ERRORX_Callback(hObject, eventdata, handles)
% hObject    handle to ERRORX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ERRORX as text
%        str2double(get(hObject,'String')) returns contents of ERRORX as a double


% --- Executes during object creation, after setting all properties.
function ERRORX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ERRORX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ERRORY_Callback(hObject, eventdata, handles)
% hObject    handle to ERRORY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ERRORY as text
%        str2double(get(hObject,'String')) returns contents of ERRORY as a double


% --- Executes during object creation, after setting all properties.
function ERRORY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ERRORY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in tracking.
function tracking_Callback(hObject, eventdata, handles)
% hObject    handle to tracking (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global str2;
global val2;
global avi_file;     
str2 = get(hObject, 'String');
val2 = get(hObject,'Value');
% set(handles.videoselect,'String',...
%    (get(hObject,'Value')));
% Set current data to the selected data set.
switch val2

    case 1
        [filename, pathname] = uigetfile('C:\Documents and Settings\karan\My Documents\My Pictures\*.avi','Input file selector');
%[filename, pathname] = uigetfile('C:\Documents and Settings\admin\Desktop\clipped video files\avi\*.avi','Input file selector');
a=strcat(pathname,filename);   %giving the path for the file bowsing
avi_file=aviread(a);           %reading the image by each frame
        set(handles.tracking,'String','Centroid Tracking');
    
    case 2
        
        [filename, pathname] = uigetfile('C:\Documents and Settings\karan\My Documents\My Pictures\*.avi','Input file selector');
    %[filename, pathname] = uigetfile('C:\Documents and Settings\admin\Desktop\clipped video files\avi\*.avi','Input file selector');
    a=strcat(pathname,filename);   %giving the path for the file bowsing
       avi_file=aviread(a);           %reading the image by each frame
        set(handles.tracking,'String','Thresholding');
    
    case 3
        [filename, pathname] = uigetfile('C:\Documents and Settings\karan\My Documents\My Pictures\*.avi','Input file selector');
%[filename, pathname] = uigetfile('C:\Documents and Settings\admin\Desktop\clipped video files\avi\*.avi','Input file selector');
a=strcat(pathname,filename);   %giving the path for the file bowsing
avi_file=aviread(a);           %reading the image by each frame
        set(handles.tracking,'String','Phase Correlation');
        
end


















% Hints: contents = get(hObject,'String') returns tracking contents as cell array
%        contents{get(hObject,'Value')} returns selected item from tracking


% --- Executes during object creation, after setting all properties.
function tracking_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tracking (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in track.
function track_Callback(hObject, eventdata, handles)
% hObject    handle to track (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global datatrack;
global avi_file;

  datatrack = get(handles.tracking,'String');

 
 switch datatrack
     
     case 'Centroid Tracking'
         
         global avi_file;
for i = 1 :size(avi_file,2)
imshow(avi_file(1,i).cdata,'Parent',handles.IMAGE1);
pause(.001);


    tic;
    frame=ind2rgb(avi_file(1,i).cdata,avi_file(1,i).colormap);
    %image(frame,'Parent',handles.axes1);
    axis off;
    frame_rgb = rgb2gray(frame);
    h = fspecial('average');
    frame_rgb= imfilter(frame_rgb,h);
    thresh=graythresh(frame_rgb);
   bw=~(im2bw(frame_rgb,thresh));
    labeled = bwlabel( bw,8);
  stats = regionprops(labeled,['basic']);%basic mohem nist
  [N,W] = size(stats);
  if N < 1
    return   
  end

  % do bubble sort (large to small) on regions in case there are more than 1
  id = zeros(N);
  for k = 1 : N
    id(k) = k;
  end
  for k = 1 : N-1
    for j = k+1 : N
      if stats(k).Area < stats(j).Area
        tmp = stats(k);
        stats(k) = stats(j);
        stats(j) = tmp;
        tmp = id(k);
        id(k) = id(j);
        id(j) = tmp;
      end
    end
  end
  selected = (labeled==id(2));

  % get center of mass and radius of largest
 k = stats(2).Centroid;
    
    imshow(bw,'Parent',handles.IMAGE3)
    %k=gdata1.Centroid;
    hold on;
    plot(k(1),k(2),'b*');
    hold on;
    tme=toc;
set(handles.centroidX,'String',num2str(k(1)));
    set(handles.centroidY,'String',num2str(k(2)));
    set(handles.TSTATUS,'String','CENTROID TRACKING');
    set(handles.TFRAME,'String',num2str(tme));
     set(handles.FRAME,'String',num2str(i));
     pause(.001);  
end
 
    case 'Thresholding'
        
         global avi_file;

         for i = 1 :size(avi_file,2)

             imshow(avi_file(1,i).cdata,'Parent',handles.IMAGE1);
pause(.001);


    tic;
    frame=ind2rgb(avi_file(1,i).cdata,avi_file(1,i).colormap);
    %image(frame,'Parent',handles.axes1);
    axis off;
    frame_rgb = rgb2gray(frame);
    h = fspecial('average');
    frame_rgb= imfilter(frame_rgb,h);
    thresh=graythresh(frame_rgb);
   bw=~(im2bw(frame_rgb,thresh));
   imshow(bw,'Parent',handles.IMAGE3);

         tme=toc;
    set(handles.TSTATUS,'String','KALMAN TRACKING');
    set(handles.TFRAME,'String',num2str(tme));
     set(handles.FRAME,'String',num2str(i));
    pause(.001);
         end
     

     case 'Phase Correlation'
         
 global avi_file;

 frame1=ind2rgb(avi_file(1,1).cdata,avi_file(1,1).colormap);
    %image(frame,'Parent',handles.axes1);
    axis off;
    frame_rgb1 = rgb2gray(frame1);
    h1 = fspecial('average');
    frame_rgb1= imfilter(frame_rgb1,h1);
    thresh1=graythresh(frame_rgb1);
   bw1=~(im2bw(frame_rgb1,thresh1));

   for i = 2 :size(avi_file,2)

    tic;
     frame2=ind2rgb(avi_file(1,i).cdata,avi_file(1,i).colormap);
    %image(frame,'Parent',handles.axes1);
    axis off;
    frame_rgb2 = rgb2gray(frame2);
    h2 = fspecial('average');
    frame_rgb2= imfilter(frame_rgb2,h2);
    thresh2=graythresh(frame_rgb2);
   bw2=~(im2bw(frame_rgb2,thresh2));
   imshow(bw2,'Parent',handles.IMAGE2);
    imshow(bw1,'Parent',handles.IMAGE1);
   t=zeros(240,320);
   L=fft2(bw1);
M=fft2(bw2);

c= real(ifft2(L .* fft2(rot90(bw2,2),240,320)));

imshow(c,'Parent',handles.IMAGE3);
   bw1=bw2;
 tme=toc;
    set(handles.TSTATUS,'String','KALMAN TRACKING');
    set(handles.TFRAME,'String',num2str(tme));
     set(handles.FRAME,'String',num2str(i));
     
    pause(.001);
   end
end
 


   
   
% --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles)
% hObject    handle to reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% cla(IMAGE1);
% cla(IMAGE2);
% cla(IMAGE2);
% cla(handles.IMAGE1);
function handles = ResetGUI(IMAGE1);




% --- Executes on selection change in videoselect.
function videoselect_Callback(hObject, eventdata, handles)
% hObject    handle to videoselect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Determine the selected data set.
global str;
global val;
global avi_file;     
str = get(hObject, 'String');
val = get(hObject,'Value');
% set(handles.videoselect,'String',...
%    (get(hObject,'Value')));
% Set current data to the selected data set.
switch val

    case 1            % if the user selected browsing videos
% User selected the first item

           %creating a global variable
[filename, pathname] = uigetfile('C:\Documents and Settings\karan\My Documents\My Pictures\*.avi','Input file selector');
%[filename, pathname] = uigetfile('C:\Documents and Settings\admin\Desktop\clipped video files\avi\*.avi','Input file selector');
a=strcat(pathname,filename);   %giving the path for the file bowsing
avi_file=aviread(a);           %reading the image by each frame

set(handles.videoselect,'String','BrowseVideos');

% for i = 1 :size(avi_file,2)    % running the loop till the number of frames of the video
% imshow(avi_file(1,i).cdata,'Parent',handles.IMAGE1); %displaying the image by each frame 
% pause(.001);  %pause each frame so can we can see each frame
% end


    case 2           %if the user selected video input from usb cam

global vid;

handles.vidobj=videoinput('winvideo',1,'RGB24_320x240');
vid=handles.vidobj;
%vidRes=get(handles.vidobj,'VideoResolution')
%nBands=get(handles.vidobj,'NumberOfBands')
hImage=image(zeros(240,320,3),'Parent',handles.axes1);
set(handles.videoselect,'String','FetchfromUSBCAM');
% preview(handles.vidobj,hImage);
end

% Save the handles structure.
guidata(hObject,handles);


% Hints: contents = get(hObject,'String') returns videoselect contents as cell array
%        contents{get(hObject,'Value')} returns selected item from videoselect


% --- Executes during object creation, after setting all properties.
function videoselect_CreateFcn(hObject, eventdata, handles)
% hObject    handle to videoselect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    


% --- Executes on button press in vidselect.
function vidselect_Callback(hObject, eventdata, handles)

global data;
global avi_file;
  data = get(handles.videoselect,'String');
 % var3= lower(data);
 
 switch data
%  strcmp(data,'BrowseVideos')==1  
     
     case 'BrowseVideos'
       
   for i = 1 :size(avi_file,2)    % running the loop till the number of frames of the video
imshow(avi_file(1,i).cdata,'Parent',handles.IMAGE1); %displaying the image by each frame 
pause(.001);  %pause each frame so can we can see each frame
   end
   
     case 'FetchfromUSBCAM'
         
   %  elseif strcmp(data,'FetchfromUSBCAM')
% 
preview(handles.vidobj,hImage);

 end
 
 % Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% hObject    handle to vidselect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in close.
function close_Callback(hObject, eventdata, handles)
% hObject    handle to close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(gcf);    % close the gui window



function TSTATUS_Callback(hObject, eventdata, handles)
% hObject    handle to TSTATUS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TSTATUS as text
%        str2double(get(hObject,'String')) returns contents of TSTATUS as a double


% --- Executes during object creation, after setting all properties.
function TSTATUS_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TSTATUS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function FRAME_Callback(hObject, eventdata, handles)
% hObject    handle to FRAME (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FRAME as text
%        str2double(get(hObject,'String')) returns contents of FRAME as a double


% --- Executes during object creation, after setting all properties.
function FRAME_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FRAME (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function TFRAME_Callback(hObject, eventdata, handles)
% hObject    handle to TFRAME (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TFRAME as text
%        str2double(get(hObject,'String')) returns contents of TFRAME as a double


% --- Executes during object creation, after setting all properties.
function TFRAME_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TFRAME (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in filter.
function filter_Callback(hObject, eventdata, handles)
% hObject    handle to filter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global val3;
global avi_file;     
str3 = get(hObject, 'String');
val3 = get(hObject,'Value');
% set(handles.videoselect,'String',...
%    (get(hObject,'Value')));
% Set current data to the selected data set.
switch val3

    case 1      
        
[filename, pathname] = uigetfile('C:\Documents and Settings\karan\My Documents\My Pictures\*.avi','Input file selector');
%[filename, pathname] = uigetfile('C:\Documents and Settings\admin\Desktop\clipped video files\avi\*.avi','Input file selector');
a=strcat(pathname,filename);   %giving the path for the file bowsing
avi_file=aviread(a);           %reading the image by each frame
set(handles.filter,'String','Mean Filter');

    case 2
        
        [filename, pathname] = uigetfile('C:\Documents and Settings\karan\My Documents\My Pictures\*.avi','Input file selector');
%[filename, pathname] = uigetfile('C:\Documents and Settings\admin\Desktop\clipped video files\avi\*.avi','Input file selector');
a=strcat(pathname,filename);   %giving the path for the file bowsing
avi_file=aviread(a);           %reading the image by each frame
set(handles.filter,'String','Gaussian Filter');
end




% Hints: contents = get(hObject,'String') returns filter contents as cell array
%        contents{get(hObject,'Value')} returns selected item from filter


% --- Executes during object creation, after setting all properties.
function filter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to filter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function centroidX_Callback(hObject, eventdata, handles)
% hObject    handle to centroidX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of centroidX as text
%        str2double(get(hObject,'String')) returns contents of centroidX as a double


% --- Executes during object creation, after setting all properties.
function centroidX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to centroidX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function centroidY_Callback(hObject, eventdata, handles)
% hObject    handle to centroidY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of centroidY as text
%        str2double(get(hObject,'String')) returns contents of centroidY as a double


% --- Executes during object creation, after setting all properties.
function centroidY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to centroidY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key press with focus on videoselect and none of its controls.
function videoselect_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to videoselect (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
