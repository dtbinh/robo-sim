%{ 
/////////////
PROGRAM NOTES
/////////////

To Determine AntennaData
------------------------
1)Name of file
2)Name of spreadsheet
3)Data Range

To Determine RotAngle 
----------------
1)Rotate the Polar plot so the main lobe is centered at 0 degrees
*This will require verification using the polar plot
2)RotAngle = 0 - Main_Lobe_ANGLE 
3)This will effectively orient the main lobe at 0 degrees and prep the plot
for revolution about the axis

OR

1)Take Degree measurement for "Max dB" from Antenna measurement sheet.
%}

clc, clear all, close all;

% Read Spreadsheet
    AntennaData = xlsread('data2.xlsx','Trace Data','A5:C41'); %Read data from file  
    Angle_Position =(pi/180)*AntennaData(:,1); %0,5,10,....180.
    Lin_Mag = AntennaData(:,2); % radius

% Variables
    RotAngle = -95; 
    phi = -pi:.01:pi;  
    phi_slice = -pi:2*pi:pi;

% Rectangular Data Unmodified
    Xp = Lin_Mag.*cos(Angle_Position);      %Convert polar to Cartesian X value
    Yp = Lin_Mag.*sin(Angle_Position);      %Convert polar to Cartesian Y value

% Rectangular Data Modified
    Xr = Xp .* cosd(RotAngle) - Yp .* sind(RotAngle); %Modified X Coordinate
    Yr = Xp .* sind(RotAngle) + Yp .* cosd(RotAngle); %Modified Y Coordinate

%% POLAR PLOT
% Generate Subplot of Polar and Cartesian(Modified and Unmodified)   
    figure('name','Polar Plot','numbertitle','off')
    polar(Angle_Position, Lin_Mag); %2D polar plot
    
%% UNMODIFIED RADIATION PATTERN

% Plot of Rectangular Data Unmodified    
    figure('name','Radiation Unmodified','numbertitle','off')
    subplot(1,3,1)
    plot(Xp,Yp)                
    
% 3D Surface of Revolution Unmodified Slice  
    X = repmat(Xp,size(phi_slice));   %X coordinate
    Y = Yp * cos(phi_slice);          %Y coordinate
    Z = Yp * sin(phi_slice);          %Z coordinate
    subplot(1,3,2)
    mesh(X,Y,Z, 'FaceColor','interp','FaceLighting','phong'); %3D plot
    camlight right %Lighting Effect, not really necessary
    xlabel('X', 'Color', 'b','FontSize',20)
    ylabel('Y', 'Color', 'b','FontSize',20)
    zlabel('Z', 'Color', 'b','FontSize',20)
    
% 3D Surface of Revolution Unmodified           
    X = repmat(Xp,size(phi));       %X coordinate
    Y = Yp * cos(phi);              %Y coordinate
    Z = Yp * sin(phi);              %Z coordinate
    subplot(1,3,3)
    mesh(X,Y,Z, 'FaceColor','interp','FaceLighting','phong'); %3D Radiation plot
    camlight right %Lighting Effect, not really necessary
    xlabel('X', 'Color', 'r','FontSize',20)
    ylabel('Y', 'Color', 'r','FontSize',20)
    zlabel('Z', 'Color', 'r','FontSize',20)

%% MODIFIED RADIATION PATTERN    

% Plot of Rectangular Data Modified    
    figure('name','Radiation Rotated','numbertitle','off')
    subplot(1,3,1)
    plot(Xr,Yr)
    
% 3D Surface of Revolution Modified Slice   
    X = repmat(Xr,size(phi_slice));   %X coordinate
    Y = Yr * cos(phi_slice);          %Y coordinate
    Z = Yr * sin(phi_slice);          %Z coordinate
    subplot(1,3,2)
    mesh(X,Y,Z, 'FaceColor','interp','FaceLighting','phong'); %3D plot
    camlight right %Lighting Effect, not really necessary
    xlabel('X', 'Color', 'b','FontSize',20)
    ylabel('Y', 'Color', 'b','FontSize',20)
    zlabel('Z', 'Color', 'b','FontSize',20)
    
% 3D Surface of Revolution Modified 
    X = repmat(Xr,size(phi));   %X coordinate
    Y = Yr * cos(phi);          %Y coordinate
    Z = Yr * sin(phi);          %Z coordinate
    subplot(1,3,3)
    mesh(X,Y,Z, 'FaceColor','interp','FaceLighting','phong'); %3D plot
    camlight right %Lighting Effect, not really necessary
    xlabel('X', 'Color', 'r','FontSize',20)
    ylabel('Y', 'Color', 'r','FontSize',20)
    zlabel('Z', 'Color', 'r','FontSize',20)
    
%% RECTANGULAR COORDINATES ROTATED BACK TO ORIGINAL ORIENTATION
    Xoriginal = X*cosd(-RotAngle) - Y*sind(-RotAngle);
    Yoriginal = X*sind(-RotAngle) + Y*cosd(-RotAngle);
    Zoriginal = Z;
    figure('name','True Radiation Pattern','numbertitle','off')
    mesh(Xoriginal,Yoriginal,Zoriginal, 'FaceColor','interp','FaceLighting','phong'); %3D plot
    camlight right %Lighting Effect, not really necessary
    xlabel('X', 'Color', 'r','FontSize',20)
    ylabel('Y', 'Color', 'r','FontSize',20)
    zlabel('Z', 'Color', 'r','FontSize',20)

%% RECTANGULAR COORDINATES TO SPHERICAL   
    [AZIMUTH,ELEVATION,RADIUS] = cart2sph(Xoriginal,Yoriginal,Zoriginal);
