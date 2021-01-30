%% Kyle Molinari, May 10, 2020

%Takes quaternion data From IMU (BNO055) and displays real time 
%orientation in 3D  with .stl overlay

%last updated Dec 23, 2020

clear;
clc;

%set serial port and baud rate
s = serialport("COM18", 9600);
disp("Ready"); 

altitudeoffset = 4.5;

%number of samples to be plotted
N = 5000;
%current sample
i = 0;
%or set this to true for continuous plotting
keepPlotting = true;

%initialize adjustquat to the identity quaternion
adjustquat = [1 0 0 0];

%quaternion representing the default position set when button is pressed
defaultquat = [-0.7 0 0 -0.72];%[-0.71 0.03 0 -0.7]

%define local x y and z axes
axes = [0 0 -1;-1 0 0; 0 1 0];
%axes = [0 0 0;0 0 0;0 0 0];

rotatex90 = [1, 0, 0; 0, 0, -1;0, 1, 0];
rotatey90 = [0, 0, 1; 0, 1, 0;-1, 0, 0];
rotatez90 = [0, -1, 0; 1, 0, 0;0, 0, 1];

rotatex45 = [1, 0, 0; 0, sqrt(2)/2, -sqrt(2)/2;0, sqrt(2)/2, sqrt(2)/2];
rotatey45 = [sqrt(2)/2, 0,sqrt(2)/2; 0, 1, 0;-sqrt(2)/2, 0, sqrt(2)/2];
rotatez45 = [sqrt(2)/2, -sqrt(2)/2, 0; sqrt(2)/2, sqrt(2)/2, 0;0, 0, 1];

%optional - point cloud to be overlayed onto plot. point cloud can be
%imported as .xyz or .txt in the form X Y Z or X Y Z R G B.
%use point clouds with under 15k points to avoid latency
try
    stl = stlread("AssemblyV4.stl");
    
    pointcloud = stl.Points;
    meshlist = stl.ConnectivityList;
    xmean = mean(pointcloud(:,1));
    ymean = mean(pointcloud(:,2));
    zmean = mean(pointcloud(:,3));
    pointcloud(:,1) = pointcloud(:,1)-xmean;
    pointcloud(:,2) = pointcloud(:,2)-ymean;
    pointcloud(:,3) = pointcloud(:,3)-zmean;
    xscale = 1.5*max(pointcloud(:,3));
    yscale = 1.5*max(pointcloud(:,1));
    zscale = 1.5*max(pointcloud(:,2));
    
    %change initial orientation
    pointcloud = pointcloud*rotatex90*rotatex90*rotatez90*rotatez90*rotatez90;
    flag = 1;
    
    clear xmean ymean zmean stl rotatex90 rotatey90 rotatez90 rotatex45 rotatey45 rotatez45;

catch
    pointcloud = [0 0 0];
    xscale = 1;
    yscale = 1;
    zscale = 1;
    flag = 0;
end

dronecolour = [0 0 0];

colormap(dronecolour)

[X,Y] = meshgrid(-15:1:15,-15:15);
Z = zeros(length(X(:,1)),length(X(1,:)));

%define origin point to be used in plot with point cloud
%size = size(pointcloud);
%origin = zeros(size(1),1);

fig = figure(1);
fig.WindowState = 'maximized';

altitude = 0;

while(i<N || keepPlotting)

    tic
    
    %increment current sample
    i=i+1;

    %read orientation data from serial port
    data = str2double(split(readline(s), ",")');

    %Serial port data structure is of the form:
    %w,x,y,z,distance,reset
    %orientation is represented by quaternion w,x,y,z
    %distance is in mm
    %reset can be 0 (no button press) or 1 (button press)

    %save data into variables
    %take negative of w, x, and y to flip z axis rotation
    quatw = -data(1); 
    quatx = -data(2);
    quaty = -data(3);
    quatz = data(4);
    distance = data(5);
    reset = data(6);

    %if reset flag is set, zero out the rotation
    if reset == 1
        %find inverse quaternion of the current orientation
        [winv, xinv, yinv, zinv] = invquat([quatw quatx quaty quatz]);
        %multiply the current quaternion inverse by the default quaternion position
        [w, x, y, z] = quatmult([winv, xinv, yinv, zinv],defaultquat);
        %update the adjust quaternion which "pulls" the current
        %orientation back to the default quaternion
        adjustquat = [w, x, y, z]/norm([w, x, y, z]);
    end

    %create rotation matrix corresponding to current orientation
    rotationmatrix = [1-2*(quaty^2+quatz^2), 2*(quatx*quaty-quatw*quatz), 2*(quatw*quaty+quatx*quatz); ...
        2*(quatx*quaty+quatw*quatz), 1-2*(quatx^2+quatz^2), 2*(quaty*quatz-quatw*quatx); ...
        2*(quatx*quatz-quatw*quaty), 2*(quatw*quatx+quaty*quatz), 1-2*(quatx^2+quaty^2)];

    %create another rotation matrix to adjust the current orientation
    %based on the adjust quaternion
    adjustrotation = [1-2*(adjustquat(3)^2+adjustquat(4)^2), ...
        2*(adjustquat(2)*adjustquat(3)-adjustquat(1)*adjustquat(4)), ...
        2*(adjustquat(1)*adjustquat(3)+adjustquat(2)*adjustquat(4)); ...
        2*(adjustquat(2)*adjustquat(3)+adjustquat(1)*adjustquat(4)), ...
        1-2*(adjustquat(2)^2+adjustquat(4)^2), ...
        2*(adjustquat(3)*adjustquat(4)-adjustquat(1)*adjustquat(2)); ...
        2*(adjustquat(2)*adjustquat(4)-adjustquat(1)*adjustquat(3)), ...
        2*(adjustquat(1)*adjustquat(2)+adjustquat(3)*adjustquat(4)), ...
        1-2*(adjustquat(2)^2+adjustquat(3)^2)];

    %define local cartesian axes by rotating the defined global axes by
    %the rotation matrix and adjust rotation matrix
    cartaxes = axes*rotationmatrix*adjustrotation;
    cartaxes(1,:) = cartaxes(1,:)/norm(cartaxes(1,:));
    cartaxes(2,:) = cartaxes(2,:)/norm(cartaxes(2,:));
    cartaxes(3,:) = cartaxes(3,:)/norm(cartaxes(3,:));

    %rotate the point cloud based on the rotation matrix and adjust
    %rotation matrix
    cartcoords = pointcloud*rotationmatrix*adjustrotation;

    %convert distance measurement into altitude (need to take out
    %horizontal component)
    localdownvector = [-cartaxes(1,1) -cartaxes(1,2) cartaxes(1,3)]/norm([-cartaxes(1,1) -cartaxes(1,2) cartaxes(1,3)]);
    globaldownvector = [0 0 -1];
    
    if(acosd(dot(localdownvector, globaldownvector))<60)
        altitude = distance/10*dot(localdownvector, globaldownvector)+altitudeoffset;
    end
    
    
    %plot local coordinates in RGB
    quiver3(0,0,altitude, cartaxes(1,1), cartaxes(1,2), -cartaxes(1,3), xscale, 'Color', 'r');
    hold on
    quiver3(0,0,altitude, cartaxes(2,1), cartaxes(2,2), -cartaxes(2,3), yscale, 'Color', 'g');
    quiver3(0,0,altitude, cartaxes(3,1), cartaxes(3,2), -cartaxes(3,3), zscale, 'Color', 'b');

    axis equal

    if(flag~=0)
        %plot point cloud data in specified colour
        trimesh(meshlist,cartcoords(:,1),cartcoords(:,2),-cartcoords(:,3)+altitude, pointcloud(:,3), 'FaceColor', [0.4 0.4 0.4]);
    
        %plot ground      
        mesh(X,Y,Z,'FaceColor', [0.4660 0.6740 0.1880]);
    end
    
    set(gca,'color', [1 1 1])
    set(gca, 'visible', 'off')
    
    hold off

    %figure specifications
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
    word = "Sample " + i + " of " + N;
    title("Position and Orientation: " + word);

    view(-40,5)
    %view(10,10)


    drawnow


    %display time between plot updates. This should roughly be equal to
    %the reciprocal of the frequency. If it is larger, try reducing the
    %number of points in the point cloud
    toc



end

