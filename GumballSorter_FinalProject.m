%% ECE 5330: Introduction to Robotics
% Final Project: Gumball Sorter 
% Description: This project will utilize the OWI Robot using 4 DOF from the
% end effector (M1), shoulder (M2), elbow (M3), and wrist (M4) and a camera
% that is mounted on top of the robot to execute a successful gumball
% sorting action. 
% CAVEAT: Camera became too heavy, and wore the motors 
% Names: Yulissa Rivas and Samyuktha Santhosh
% Date: 12/06/2023

%%%%%%%%%%%%%%%%%% LET'S BEGIN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Housing commands
clear; clc;
%clear('cam'); % clear the cam object so you can make a new one; no need
cam = webcam('HD Web Camera'); %open the camera

% Double check all joints have been cleared
clear s MotorElbow sm shield a;         
clear s MotorBase sm shield a;
clear s MotorShoulder sm shield a;
clear s MotorGripper sm shield a;
a = arduino('COM5', 'Mega2560', 'Libraries', {'Adafruit\MotorShieldV2'}); % Creates arduino object to control arduino board
shield = addon(a,'Adafruit\MotorShieldV2');      % addon to the arduino object for the motor shield

MotorBase = dcmotor(shield,4);        % Object for the waist joint
MotorShoulder = dcmotor(shield,2);    % object for the shoulder joint
MotorElbow = dcmotor(shield,3);       % Object for the elbow joint
MotorGripper = dcmotor(shield,1);     % Object for the gripper joint

% Declaring condition variables 
readyPos=0;         % condition for when end effector is ready to pickup gumball
initiate=1;         % while loop condition; allows to break from main
colorMask = 0;      % condition to determine specified mask for specified color
baseTime = 5.2;     % condition for base turn time after drop-off

while(initiate)
RGB = snapshot(cam);    

%%%%%%%%%%%%%%%%%%%%%%%%% FROM colorThresholder%%%%%%%%%%%%%%%%%%%%%%%%%%%
% I used the Matlab app colorThresholder to find the points for my mask, which is red 
% Convert RGB image to chosen color space
I = rgb2hsv(RGB);
if(colorMask == 0)  % use of color variable to set specific mask

%%%%%%%%%%%%%%%%%%%%%%%%% Red Color Threshold %%%%%%%%%%%%%%%%%%%%%%%%%
gumballArea=13000;    % area specifies gripper the range to grab the gumball

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.987;
channel1Max = 0.033;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.744;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.650;
channel3Max = 1.000;

dropOffTime = 2; % time after reaching checkpoint to get to red drop off zone

elseif(colorMask == 1)  % color check for green

%%%%%%%%%%%%%%%%%%%%%%%%% Green Color Threshold %%%%%%%%%%%%%%%%%%%%%%%%%

gumballArea = 8500;       % area specifies gripper the range to grab the gumball
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.447;
channel1Max = 0.505;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.789;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.528;
channel3Max = 0.692;

dropOffTime = 4; % time after reaching checkpoint to get to green drop off zone

elseif(colorMask == 2)  % color check for blue

%%%%%%%%%%%%%%%%%%%%%%%%% Blue Color Threshold %%%%%%%%%%%%%%%%%%%%%%%%%
gumballArea = 12000;  % area of blue object to pickup
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.540;
channel1Max = 0.614;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.784;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.667;
channel3Max = 1.000;

dropOffTime = 6; % time after reaching checkpoint to get to blue drop off zone

end



% Create mask based on chosen histogram thresholds
sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
(I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
(I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;
% Initialize output masked image based on input image.
maskedRGBImage = RGB;
% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
%%%%%%%%%%%%% END FROM colorThresholder %%%%%%%%%%%%%%%% 
%figure(2);clf;
imshow(maskedRGBImage) 

%https://www.mathworks.com/help/images/ref/bwconncomp.html is easier.
CC = bwconncomp(sliderBW);
s = regionprops(CC,'Centroid','Area');
centroids1 = cat(1,s.Centroid);     % gets centroid of object
areas = cat(1,s.Area);              % gets area of connected object
[m,ind] = max(areas);               % find the largest connected component


while(max(areas) >= 1000) % only pickup gumballs if it's within frame of specified color, otherwise move to next mask and check again
RGB = snapshot(cam);

I = rgb2hsv(RGB);
% Create mask based on chosen histogram thresholds
sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
(I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
(I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;
% Initialize output masked image based on input image.
maskedRGBImage = RGB;
% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
%%%%%%%%%%%%% END FROM colorThresholder %%%%%%%%%%%%%%%% 
%figure(2);clf;
%imshow(maskedRGBImage) 

%https://www.mathworks.com/help/images/ref/bwconncomp.html is easier.
CC = bwconncomp(sliderBW);
s = regionprops(CC,'Centroid','Area');
centroids1 = cat(1,s.Centroid);
areas = cat(1,s.Area);
[m,ind] = max(areas);       % find the largest connected component
disp("Inside Loop");
figure(3);clf;
imshow(double(sliderBW))    % display masked image
hold on
plot(centroids1(ind,1),centroids1(ind,2),'m*','markersize',32) % Plot centroid of the object
% Place a star on the largest connected component.
% This is where you should aim your robot.

[rows,columns,layers] = size(sliderBW); % size elements of image
% Determine the center of x and center of y of the frame-> create cross hair in the middle of screen
xCenter = columns/2;                    
yCenter = rows/2;
xline(xCenter, 'LineWidth', 2, 'Color', 'r');
yline(yCenter, 'LineWidth', 2, 'Color', 'r');

xline(centroids1(ind,1), 'LineWidth', 2, 'Color', 'r'); % Plot cross hair on the centroid of the object
yline(centroids1(ind,2), 'LineWidth', 2, 'Color', 'r');

disp(max(areas));                   % Display area of connected object // DEBUG
if(readyPos==0)                     % Check if in Ready Position, if not then find Ready Position
if((centroids1(ind,1)-xCenter)<-30) % If left of center, move left
   MotorBase.Speed = 0.3;
   start(MotorBase);
   pause(0.2);
   stop(MotorBase);
elseif((centroids1(ind,1)-xCenter)>30)  % If right of center-> move right
   MotorBase.Speed = -0.3;
   start(MotorBase);
   pause(0.2);
   stop(MotorBase);
end 

if((centroids1(ind,2)-yCenter)<-30)     % If above center-> move right
    MotorElbow.Speed = 0.35;
    start(MotorElbow);
    pause(0.2);
    stop(MotorElbow);
elseif((centroids1(ind,2)-yCenter)>30)  % If below center-> move below
    MotorElbow.Speed = -0.25;
    start(MotorElbow);
    pause(0.2);
    stop(MotorElbow);
end

% If grip is centered and far from gumball-> move closer
if(((centroids1(ind,1)-xCenter)>-50) && ((centroids1(ind,1)-xCenter)<50) ...
        && ((centroids1(ind,2)-yCenter)>-50) && ((centroids1(ind,2)-yCenter)<50))
    
    disp(max(areas)); % DEBUGGING
    if(max(areas)>2000 & max(areas)<gumballArea) % If area is >2000-> eliminate noise
        disp("Forward");
        MotorElbow.Speed=0.60;
        MotorShoulder.Speed=0.40;
        start(MotorElbow);
        start(MotorShoulder);
        pause(0.4);
        stop(MotorElbow);
        stop(MotorShoulder);
    
    end
end
end


% If gumball is centered and close-> initiate pickup
if(((centroids1(ind,1)-xCenter)>-80) && ((centroids1(ind,1)-xCenter)<80) ...
        && ((centroids1(ind,2)-yCenter)>-80) && ((centroids1(ind,2)-yCenter)<80)&&(max(areas>gumballArea)))

    readyPos=1; % Initiate pickup-> place in drop off zone
end


if(readyPos) % Initiating Pickup
    disp("Close Grip");
    MotorGripper.Speed = 0.5;
    start(MotorGripper);        % Close gripper
    pause(1.2);
    stop(MotorGripper);


    MotorShoulder.Speed=-0.50;
    start(MotorShoulder);        % Lift gumball up and away from table
    pause(1.5);
    stop(MotorShoulder);


    MotorElbow.Speed=-0.50;
    MotorShoulder.Speed=-0.60;
    start(MotorElbow);           % Use two joints to move linearly backward to original position
    start(MotorShoulder);
    pause(1.3);
    stop(MotorElbow);
    stop(MotorShoulder);

    RGB = snapshot(cam);         
    %figure(1)
    %imshow(RGB)

    I = rgb2hsv(RGB);

    sliderBW = ((I(:,:,1) >= channel1greenmin) | (I(:,:,1) <= channel1greenmax)) & ...
    (I(:,:,2) >= channel2greenmin) & (I(:,:,2) <= channel2greenmax) & ...
    (I(:,:,3) >= channel3greenmin) & (I(:,:,3) <= channel3greenmax);

    BW = sliderBW;

    maskedRGBImage = RGB;

    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

    CC = bwconncomp(sliderBW);

    s = regionprops(CC,'Centroid','Area');

    centroids3 = cat(1,s.Centroid);

    areas = cat(1,s.Area);



while(~(((centroids3(ind,1)-xCenter)>-80) && ((centroids3(ind,1)-xCenter)<80)))     
    disp("Searching Green Gumball...");
    disp(max(areas))
    RGB = snapshot(cam);
    %figure(1)
    %imshow(RGB)

    I = rgb2hsv(RGB);

    sliderBW = ((I(:,:,1) >= channel1greenmin) | (I(:,:,1) <= channel1greenmax)) & ...
    (I(:,:,2) >= channel2greenmin) & (I(:,:,2) <= channel2greenmax) & ...
    (I(:,:,3) >= channel3greenmin) & (I(:,:,3) <= channel3greenmax);

    BW = sliderBW;

    maskedRGBImage = RGB;

    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

   
   figure(3);clf;

    imshow(maskedRGBImage)

    CC = bwconncomp(sliderBW);

    s = regionprops(CC,'Centroid','Area');

    centroids3 = cat(1,s.Centroid);

    areas = cat(1,s.Area);

    MotorBase.Speed=-0.45;
    start(MotorBase);      
    pause(0.2);
    stop(MotorBase);

end

    disp("Green Gumball Found.");
    % Move to respective drop off bins
    MotorBase.Speed = -0.45; 
    start(MotorBase);        
    pause(dropOffTime);
    stop(MotorBase);

    disp("Open Gripper");
    MotorGripper.Speed = -0.5;
    start(MotorGripper);        % Gripper drops gumball in drop off zone
    pause(0.8);
    stop(MotorGripper);

    RGB = snapshot(cam);        % Initiate search
    %figure(1)
    %imshow(RGB)

    I = rgb2hsv(RGB);

    sliderBW = ((I(:,:,1) >= channel1greenmin) | (I(:,:,1) <= channel1greenmax)) & ...
    (I(:,:,2) >= channel2greenmin) & (I(:,:,2) <= channel2greenmax) & ...
    (I(:,:,3) >= channel3greenmin) & (I(:,:,3) <= channel3greenmax);

    BW = sliderBW;

    maskedRGBImage = RGB;

    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

    CC = bwconncomp(sliderBW);

    s = regionprops(CC,'Centroid','Area');

    centroids3 = cat(1,s.Centroid);

    areas = cat(1,s.Area);

while(~(((centroids3(ind,1)-xCenter)>-80) && ((centroids3(ind,1)-xCenter)<80)))

    disp("Searching for Green Gumball... going back...");
    disp(max(areas));
    RGB = snapshot(cam);
    %figure(1)
    %imshow(RGB)

    I = rgb2hsv(RGB);

    sliderBW = ((I(:,:,1) >= channel1greenmin) | (I(:,:,1) <= channel1greenmax)) & ...
    (I(:,:,2) >= channel2greenmin) & (I(:,:,2) <= channel2greenmax) & ...
    (I(:,:,3) >= channel3greenmin) & (I(:,:,3) <= channel3greenmax);

    BW = sliderBW;

    maskedRGBImage = RGB;

    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

    figure(3);clf;

    imshow(maskedRGBImage)

    CC = bwconncomp(sliderBW);

    s = regionprops(CC,'Centroid','Area');

    centroids3 = cat(1,s.Centroid);

    areas = cat(1,s.Area);

    MotorBase.Speed=0.40;
    start(MotorBase);
    pause(0.2);     
    stop(MotorBase);

end

    disp("Found Green Gumball Again!");  
    MotorBase.Speed = 0.45;
    start(MotorBase);
    pause(baseTime);
    stop(MotorBase);

    readyPos=0;    
end

RGB = snapshot(cam);        % Search for next Gumball

I = rgb2hsv(RGB);
% Create mask based on chosen histogram thresholds
sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
(I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
(I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;
% Initialize output masked image based on input image.
maskedRGBImage = RGB;
% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
%%%%%%%%%%%%% END FROM colorThresholder %%%%%%%%%%%%%%%% 
%https://www.mathworks.com/help/images/ref/bwconncomp.html is easier.
CC = bwconncomp(sliderBW);
s = regionprops(CC,'Centroid','Area');
centroids2 = cat(1,s.Centroid);
areas = cat(1,s.Area);
[m,ind] = max(areas);           % Locate the largest connected component

disp("Final max:"+max(areas));


end
colorMask = colorMask + 1;      % If specific color is not detected-> Switch to next mask and Search for a new color

if(colorMask==3)                % If all colors have been detected-> Switch to first mask to Search for new colors again
    colorMask=0;
end


end