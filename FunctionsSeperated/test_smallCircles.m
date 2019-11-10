%%
clear
clc
clear pi r cam; clc;
disp('_____________________________________________________________')
mm2pixel=200/300;
pixel2mm=300/200;
%%%%%%%%%%%%%%%%%%%%%%%%%Change directory pointing to your main folder%%%%%%%%%
Directory2Zip='C://Users/ryanl/Downloads/Senior_Project_kyle/Senior_Project';
%will try to pull image from pi if error is thrown will just load image
%from a directory specified
try
    r = raspi('10.100.114.59','pi','SeniorProject');
    cam = cameraboard(r,'Resolution','1280x720','Quality',100,...
        'Brightness',55,'Contrast', 0,'Saturation',0,'Sharpness',0);
    cam.HorizontalFlip = 1; %Flip upside down
    img = snapshot(cam);%capture image
catch
    img=imread([Directory2Zip  '/Photos/img3_3.png']);
end
%will try to undistort photo using cameraParams, if no cameraParams in
%workspace, will load using the function importCameraParams()
%commented out undistort casue I feel like it fucks it up more
try
    %img = undistortImage(img,cameraParams);
catch
    %this will calc cameraParams if not defined
    for i=1:40
        if (i<10)
            imageFileNames{i}=[Directory2Zip '/Photos/image_000'  num2str(i) '.png'];
        else
            imageFileNames{i}=[Directory2Zip '/Photos/image_00'  num2str(i) '.png'];
        end
    end
    % Detect checkerboards in images
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
    imageFileNames = imageFileNames(imagesUsed);
    % Read the first image to obtain image size
    originalImage = imread(imageFileNames{1});
    [mrows, ncols, ~] = size(originalImage);
    % Generate world coordinates of the corners of the squares
    squareSize = 22;  % in units of 'millimeters'
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    % Calibrate the camera
    [cameraParamsOut, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
        'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
        'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
        'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
        'ImageSize', [mrows, ncols]);    
    cameraParams = cameraParamsOut;
    img = undistortImage(img,cameraParams);
end
figure('Name','Original Image','NumberTitle','off')
imshow(img)
%%
%----------------------------------------------------
% Convert RGB image into L*a*b* color space.
X = rgb2lab(img);
% Create empty mask.
BW = false(size(X,1),size(X,2));
% Flood fill
row = 482;
column = 412;
tolerance = 5.000000e-02;
normX = sum((X - X(row,column,:)).^2,3);
normX = mat2gray(normX);
addedRegion = grayconnected(normX, row, column, tolerance);
BW = BW | addedRegion;
% Flood fill
row = 426;
column = 55;
tolerance = 5.000000e-02;
normX = sum((X - X(row,column,:)).^2,3);
normX = mat2gray(normX);
addedRegion = grayconnected(normX, row, column, tolerance);
BW = BW | addedRegion;
% Invert mask
BW = imcomplement(BW);
% Flood fill
row = 92;
column = 12;
tolerance = 5.000000e-02;
normX = sum((X - X(row,column,:)).^2,3);
normX = mat2gray(normX);
addedRegion = grayconnected(normX, row, column, tolerance);
BW = BW | addedRegion;
% Invert mask
BW = imcomplement(BW);
% Flood fill
row = 116;
column = 12;
tolerance = 5.000000e-02;
normX = sum((X - X(row,column,:)).^2,3);
normX = mat2gray(normX);
addedRegion = grayconnected(normX, row, column, tolerance);
BW = BW | addedRegion;
% Invert mask
BW = imcomplement(BW);
% Invert mask
BW = imcomplement(BW);
% Flood fill
row = 690;
column = 722;
tolerance = 5.000000e-02;
normX = sum((X - X(row,column,:)).^2,3);
normX = mat2gray(normX);
addedRegion = grayconnected(normX, row, column, tolerance);
BW = BW | addedRegion;
% Invert mask
BW = imcomplement(BW);
% Create masked image.
maskedImage = img;
maskedImage(repmat(~BW,[1 1 3])) = 0;
maskedImage_old=maskedImage;
% Convert RGB image into L*a*b* color space.
X = rgb2lab(maskedImage_old);
% Create empty mask.
BW = false(size(X,1),size(X,2));
% Local graph cut
xPos = [65.1154 1236.8077 1236.8077 65.1154 ];
yPos = [52.8077 52.8077 674.3462 674.3462 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [];
backgroundInd = [];
L = superpixels(X,8855,'IsInputLab',true);
% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);
% Invert mask
BW = imcomplement(BW);
% Create masked image.
maskedImage = maskedImage_old;
maskedImage(repmat(~BW,[1 1 3])) = 0;
RGB=maskedImage;
%another mask
I = rgb2lab(RGB);
% Define thresholds for channel 1 based on histogram settings
channel1Min = 64.347;
channel1Max = 93.285;
% Define thresholds for channel 2 based on histogram settings
channel2Min = -13.680;
channel2Max = 7.373;
% Define thresholds for channel 3 based on histogram settings
channel3Min = 3.912;
channel3Max = 53.067;
% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;
% Initialize output masked image based on input image.
maskedRGBImage = RGB;
% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
%%
%masked image to grey for edge detection
grey=rgb2gray(maskedRGBImage);
figure('Name', 'Masked out')
imshow(maskedRGBImage)
figure('Name','OG')
imshow(img);
hold on; axis on;
[c, r, m] = imfindcircles(grey,[2 6],'Sensitivity', 0.9, 'EdgeThreshold', 0.3339917);
%tries max dots at 18, catches at 3 dot incrament
try
    cSelect = c(1:18,:);
    rSelect = r(1:18);
    mSelect = m(1:18);
catch
    try
        cSelect = c(1:17,:);
        rSelect = r(1:17);
        mSelect = m(1:17);
    catch
        try
            cSelect = c(1:16,:);
            rSelect = r(1:16);
            mSelect = m(1:16);
        catch
            try
                cSelect = c(1:15,:);
                rSelect = r(1:15);
                mSelect = m(1:15);
            catch
                cSelect = c(1:14,:);
                rSelect = r(1:14);
                mSelect = m(1:14);
            end
        end
    end
end
%shows all circles detected
viscircles(c, r,'Color','b');
%shows circles used
viscircles(cSelect, rSelect,'Color','r','LineStyle','--');

%used to create averages
L=[];
R=[];
T=[];
B=[];
oppsx=[];
oppsy=[];

for i=1:length(c) 
    if c(i,1) <= 200
        %left side
        L=[L,;c(i,1)];
    elseif c(i,1) >= 1200 
        %Right side
        R=[R,;c(i,1)];
    else
        if c(i,2)<= 100
            %top
            T=[T,c(i,2)];
        elseif c(i,2)>= 600
            %Bottom
            B=[B,c(i,2)];
        else
            disp('outside parameters, random value')
            oppsx=[oppsx,c(i,1)];
            oppsy=[oppsy,c(i,1)];
        end
    end
end
TopLine=sum(T)/length(T);
BottomLine=sum(B)/length(B);
RightLine=sum(R)/length(R);
LeftLine=sum(L)/length(L);
%rectangle('Position',[TopLine,BottomLine,RightLine,LeftLine],'EdgeColor',[0.4940 0.1840 0.5560])

%use averages to plot corners and corners to plot lines

plot(LeftLine,TopLine,'om','LineWidth',1);    %top Left
plot(RightLine,TopLine,'om','LineWidth',1);    %top Right
plot(LeftLine,BottomLine,'om','LineWidth',1);    %Bottom Left
plot(RightLine,BottomLine,'om','LineWidth',1);    %Bottom Right
line([LeftLine,LeftLine],[TopLine ,BottomLine],'Color', 'm','LineWidth', 1);%leftline
line([RightLine,RightLine],[TopLine ,BottomLine],'Color', 'm','LineWidth', 1);%rightline
line([LeftLine,RightLine],[TopLine ,TopLine],'Color', 'm','LineWidth', 1);%topline
line([LeftLine,RightLine],[BottomLine ,BottomLine],'Color', 'm','LineWidth', 1);%bottomline
plot(((LeftLine+RightLine)/2),BottomLine,'oc','LineWidth',1);    %Bottom center
plot(((LeftLine+RightLine)/2),TopLine,'oc','LineWidth',1);    %Top center
%%
OS_Bumper=70*mm2pixel;%70mm from center white circles to edge bumper
OS_ball=OS_Bumper+((58/2)*mm2pixel);%58mm measured diamter, totally guessed
%Apply offset to get wall bumbper size
line([LeftLine+OS_Bumper,LeftLine+OS_Bumper],[TopLine+OS_Bumper ,BottomLine-OS_Bumper],'Color', 'r','LineWidth', 1);%leftline
line([RightLine-OS_Bumper,RightLine-OS_Bumper],[TopLine+OS_Bumper ,BottomLine-OS_Bumper],'Color', 'r','LineWidth', 1);%rightline
line([LeftLine+OS_Bumper,RightLine-OS_Bumper],[TopLine+OS_Bumper ,TopLine+OS_Bumper],'Color', 'r','LineWidth', 1);%topline
line([LeftLine+OS_Bumper,RightLine-OS_Bumper],[BottomLine-OS_Bumper ,BottomLine-OS_Bumper],'Color', 'r','LineWidth', 1);%bottomline
%Apply offset to get wall ball radius offset
line([LeftLine+OS_ball,LeftLine+OS_ball],[TopLine+OS_ball ,BottomLine-OS_ball],'Color', 'w','LineWidth', 1);%leftline
line([RightLine-OS_ball,RightLine-OS_ball],[TopLine+OS_ball ,BottomLine-OS_ball],'Color', 'w','LineWidth', 1);%rightline
line([LeftLine+OS_ball,RightLine-OS_ball],[TopLine+OS_ball ,TopLine+OS_ball],'Color', 'w','LineWidth', 1);%topline
line([LeftLine+OS_ball,RightLine-OS_ball],[BottomLine-OS_ball ,BottomLine-OS_ball],'Color', 'w','LineWidth', 1);%bottomline 

%%
%Shot Determine
%{
[c, r, m] = imfindcircles(img,[12 22],'Sensitivity', 0.9, 'EdgeThreshold', 0.3339917);
viscircles(c, r,'Color','b');

clear B1x B1y B2x B2y
B1x = c(1,1);
B1y = c(1,2);
B2x = c(2,1);
B2y = c(2,2);
balls = [B1x B1y;B2x B2y]; %stores x y coordinates for two balls
%starts at 0,0 and rotates cw
pocket = [RightLine-OS_ball,TopLine+OS_ball;            %1, Top Right
    RightLine-OS_ball,BottomLine-OS_ball;               %2, Bottom Right
    ((LeftLine+RightLine)/2),BottomLine-OS_ball;%3, Bottom middle
    LeftLine+OS_ball,BottomLine-OS_ball;                %4, Bottom Left
    LeftLine+OS_ball,TopLine+OS_ball;                   %5, Top left
    ((LeftLine+RightLine)/2),TopLine+OS_ball];   %6, TOP MIDDLE
clear pocketX pocketY ans distance
for i = 1:6
    pocketX = pocket(i,1);
    pocketY = pocket(i,2);
    ans = sqrt((pocketX-B2x)^2+(pocketY-B2y)^2);
    distance(i,1)= ans;

end
clear M I
disp(distance); %magnitude or distance from hole to pocket
[M I] = min(distance)

clear i 
clear angle angles pocket2TargetBall xi yi vectorX vectorY rDist targetY targetY
clear vectortoPocketX vectortoPocketY targetBall2Pocket andDis distanceCue2CueHit 
clear unitVectorCueX unitVectorCueY cueVect dotTop dotBot
for i = 1:6
    color = ['b','r','g','c','w','m']; %color matrix
    mag = distance(i); %distance from target ball to pocket
    xi = pocket(i,1); %pocket X coordinate
    yi = pocket(i,2); %pocket Y coordinate
    
    %unitvector from pocket 2 target ball
    vectorX = (B2x-xi)/mag; %X unit vector from pocket to target ball
    vectorY = (B2y- yi)/mag; %Y unit vector from pocket to target ball
    pocket2TargetBall(i,1:2) = [vectorX vectorY] %store unit vectors from pocket to target ball
    
    %find new location for cue ball
    rDist = (58*mm2pixel); %(2r)=distance to hit center
    targetX = B2x+rDist*vectorX; %location X for new cue center
    targetY = B2y+rDist*vectorY; %location Y for new cue center
    
    %used to find unit vector from target ball to pocket
    vectortoPocketX = (xi-B2x)/mag; %X unit vector from target ball to pocket
    vectortoPocketY = (yi-B2y)/mag; %Y unit vector from target ball to pocket
    targetBall2Pocket(i,1:2) = [vectortoPocketX vectortoPocketY] %packinto array
    
    %used to find unit vector for cue ball direction
    ansDis = sqrt((targetX-B1x)^2+(targetY-B1y)^2); %distance cue ball travel to new cue location
    distanceCue2CueHit(i,1) = ansDis; %distance cue ball travel
    unitVectorCueX = (targetX-B1x)/ansDis;  %cueBall unit vector X  
    unitVectorCueY = (targetY-B1y)/ansDis; %cueBall unit vector Y
    cueVect(i,1:2) = [unitVectorCueX unitVectorCueY]
    
    clear A B div angle
    %angle between cue vector and target direction vector
    A=cueVect(i,1:2) ;
    B= targetBall2Pocket(i,1:2);
    dotTop = dot(A,B);
    dotBot = 1;%
    div = dotTop/dotBot;
    angle = acosd(div);
    angles(i,1) = angle
    if angles(i)<85
        line([B2x,targetX],[B2y,targetY],'Color',color(i),'LineWidth',1,'linestyle','-'); %line add behind target direction
        line([xi,B2x], [yi,B2y],'Color', color(i),'LineWidth',1,'linestyle','-'); %draw pocket to target ball center line
        line([B1x,targetX],[B1y,targetY],'Color',color(i),'LineWidth',1,'linestyle','-'); %line from cue to target cue center
        possibleShot(i,1) = 1;
    else
        possibleShot(i,1) =0;
    end
             
end
clear cueArray cueCenter newUnitVector

%pack angles,distancesCue2Cue, distance target ball to Pocket into array
cueArray = [possibleShot distance angles cueVect] 
    %possibleShot = T/F
    %color = 1=B,2=R,3=G,4=C,5=W,6=M
    %distance = distance from target ball to pocket
    %angles = angle between cue vector and vector to pocket
    %cueVect= 2columns[x y] of unit vectors of cue ball
cueCenter = [B1x B1y];
%distanceCue2CueHit is distance from initial cue position to cue collision
%cueVect is the unit vector for cue ball travel4
R = [1 0 0;0 -1 0; 0 0 -1];%%%%%%%%%%%%%%%%%%%%%%ryan redeffined last 1 to -1
robotY = [0 1]
ballDirect = [cueArray(2,4) cueArray(2,5)]; %using white
%1=B,2=R,3=G,4=C,5=W,6=M
top = dot(robotY,ballDirect);
%bottom = 1;
%div = dotTop/dotBot;
FinalAngle = acosd(top)
%compare B1x B1y (cue Initial) to targetX and target Y(cue Hit)
a=15;
t= 90;
if B1x < targetX && B1y < targetY %+z less than 90, -y,+90
    o = -(90- FinalAngle)
    a=-a
    t = t
    disp('1st target')
    
elseif B1x < targetX && B1y > targetY %-z less than 90,-y,+90
    o = FinalAngle -90
    a=-a
    t= t
    disp('2nd target')
    
elseif B1x > targetX && B1y < targetY %-z less than 90,-y,90
    o = 90 - FinalAngle 
    a=a
    t=-t
    disp('3rd target')
    
elseif B1x > targetX && B1y > targetY %+z less than 90, +y, -90
    o =  90 - FinalAngle 
    a=a
    t=-t
    disp('4th target')
end
%}
%%      
function out = prepLab(in)

% Convert L*a*b* image to range [0,1]
out = in;
out(:,:,1) = in(:,:,1) / 100;  % L range is [0 100].
out(:,:,2) = (in(:,:,2) + 86.1827) / 184.4170;  % a* range is [-86.1827,98.2343].
out(:,:,3) = (in(:,:,3) + 107.8602) / 202.3382;  % b* range is [-107.8602,94.4780].
end