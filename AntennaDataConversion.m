clc, clear all;
% Polar Data
    AntennaData = xlsread('data2.xlsx','Trace Data','A5:C41'); %Read data from file  
    Angle_Position =(pi/180)*AntennaData(:,1); %0,5,10,....180.
    Lin_Mag = AntennaData(:,2); % radius
    Phase = AntennaData(:,3); % phase 
    subplot(1,2,1)%Figure1, Plot1
    polar(Angle_Position, Lin_Mag); %2D polar plot
    
% Rectangular Data
    Xp = Lin_Mag.*cosd(Phase);  %Convert polar to Cartesian X value
    Yp = Lin_Mag.*sind(Phase);  %Convert polar to Cartesian Y value
      
 % 3D points of the surface
    phi = -pi:.01:pi;           %Revolution of 2D to get 3D plot,increment => # of data
    X = repmat(Xp,size(phi));   %X coordinate
    Y = Yp * cos(phi);          %Y coordinate
    Z = Yp * sin(phi);          %Z coordinate
    
    subplot(1,2,2)%Figure1, Plot2
    mesh(X,Y,Z, 'FaceColor','interp','FaceLighting','phong'); %3D plot
    camlight right %Lighting Effect, not really necessary
