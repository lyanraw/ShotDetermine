%%
clear pi r cam; clc;

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
    img=imread([Directory2Zip  '/Photos/img1.png']);
end

%will try to undistort photo using cameraParams, if no cameraParams in
%workspace, will load using the function importCameraParams()
%commented out undistort casue I feel like it fucks it up more
try
    img = undistortImage(img,cameraParams);
catch
    %%
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
%%
%applys a mask to generate line detection
% Convert RGB image to chosen color space
I = rgb2hsv(img);
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.141;
channel1Max = 0.238;
% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.177;
channel2Max = 0.933;
% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.669;
channel3Max = 0.963;
% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;
% Invert mask if you want to
%BW = ~BW;
% Initialize output masked image based on input image.
maskedRGBImage = img;
% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
%%
%This creates the line detection
figure('Name','Original Image','NumberTitle','off');        %creates 1st figure,
imshow(img);
grey = rgb2gray(maskedRGBImage);                            %grey scale for edge function
BW = edge(grey,'canny');                                    %does its thing
[yMax_img,xMax_img]=size(grey);                             %max values of image for squaring image
[H,theta,rho] = hough(BW);
figure('Name','Masked Image for Line Detection','NumberTitle','off');   %second Image
imshow(maskedRGBImage);
P = houghpeaks(H,100,'threshold',ceil(0.5*max(H(:))));
lines = houghlines(BW,theta,rho,P,'FillGap',1000,'MinLength',100);
figure('Name','Image with Lines Drawn','NumberTitle','off');
imshow(img);
hold on; axis on;
for k = 1:length(lines)
    if lines(k).theta == 0
        xy = [lines(k).point1; lines(k).point2];
        xy(1,2)=0;
        xy(2,2)=yMax_img;
        plot(xy(:,1),xy(:,2),'LineWidth',1,'Color','b');
        try
            if xy(1,1)<=x_intersect
                x_intersect=xy(1,1);
            end
        catch
            x_intersect=xy(1,1);
        end
    elseif lines(k).theta == -90
        xy = [lines(k).point1; lines(k).point2];
        xy(1,1)=0;
        xy(2,1)=xMax_img;
        plot(xy(:,1),xy(:,2),'LineWidth',1,'Color','b');
        try
            if xy(1,2)<=y_intersect
                y_intersect=xy(1,2);
            end
        catch
            y_intersect=xy(1,2);
        end
    else
        disp("wrong angle use robot to adjust")
        %{
        %uncoment to display other lines that arent 100% right
        xy = [lines(k).point1; lines(k).point2];
        plot(xy(:,1),xy(:,2),'LineWidth',1,'Color','r');
        % Plot beginnings and ends of lines
        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','r');
        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','r');
        %}
    end
    
end
plot(x_intersect,y_intersect,'o','LineWidth',2,'Color','m');

%%
%Following will make line detection a square
%builds off of line detection forming a box
if (x_intersect >= (xMax_img/2))
    %this means x is on table right
    xPixelsFromEdge_min=xMax_img-x_intersect;
    if (y_intersect >= (yMax_img/2))
        %intersection is on bottom Right %%%%subject to change depending on
        %flips and rotations
        yPixelsFromEdge_min=yMax_img-y_intersect;
        yPixelsFromEdge_max=y_intersect;
        xPixelsFromEdge_max=x_intersect;
    else
        %intersection is on Top Right %%%%subject to change depending on
        %flips and rotations
        yPixelsFromEdge_min=y_intersect;
        yPixelsFromEdge_max=yMax_img-y_intersect;
        xPixelsFromEdge_max=x_intersect;
    end
else %this means x is on table LEFT
    xPixelsFromEdge_min=x_intersect;
    if (y_intersect >= (yMax_img/2))
        %intersection is on bottom LEFT %%%%subject to change depending on
        %flips and rotations
        yPixelsFromEdge_min=yMax_img-y_intersect;
        yPixelsFromEdge_max=y_intersect;
        xPixelsFromEdge_max=xMax_img-x_intersect;
    else
        %intersection is on Top LEFT %%%%subject to change depending on
        %flips and rotations
        yPixelsFromEdge_min=y_intersect;
        yPixelsFromEdge_max=yMax_img-y_intersect;
        xPixelsFromEdge_max=xMax_img-x_intersect;
    end
    
end
plot(xPixelsFromEdge_min,yPixelsFromEdge_min,'om','LineWidth',2);    %top Left
plot(xPixelsFromEdge_max,yPixelsFromEdge_min,'om','LineWidth',2);    %top Right
plot(xPixelsFromEdge_min,yPixelsFromEdge_max,'om','LineWidth',2);    %Bottom Left
plot(xPixelsFromEdge_max,yPixelsFromEdge_max,'om','LineWidth',2);    %Bottom Right
line([xPixelsFromEdge_min,xPixelsFromEdge_min],[yPixelsFromEdge_min ,yPixelsFromEdge_max],'Color', 'm','LineWidth', 2);%leftline
line([xPixelsFromEdge_max,xPixelsFromEdge_max],[yPixelsFromEdge_min ,yPixelsFromEdge_max],'Color', 'm','LineWidth', 2);%rightline
line([xPixelsFromEdge_min,xPixelsFromEdge_max],[yPixelsFromEdge_min ,yPixelsFromEdge_min],'Color', 'm','LineWidth', 2);%topline
line([xPixelsFromEdge_min,xPixelsFromEdge_max],[yPixelsFromEdge_max ,yPixelsFromEdge_max],'Color', 'm','LineWidth', 2);%bottomline

plot((xPixelsFromEdge_max/2),yPixelsFromEdge_max,'om','LineWidth',2);    %Bottom center
plot((xPixelsFromEdge_max/2),yPixelsFromEdge_min,'om','LineWidth',2);    %Top center
%%
%Once the above is calibrated we then appply this, my different setup is
%different as of now in lab nstant but for now
%build off last box and adds offsets for ball and bumper to create your
%original 2 boxes
offSetForBumper=28;
offSetForBall=15+offSetForBumper;
%Apply offset to get wall bumbper size
line([xPixelsFromEdge_min+offSetForBumper,xPixelsFromEdge_min+offSetForBumper],[yPixelsFromEdge_min+offSetForBumper ,yPixelsFromEdge_max-offSetForBumper],'Color', 'r','LineWidth', 2);%leftline
line([xPixelsFromEdge_max-offSetForBumper,xPixelsFromEdge_max-offSetForBumper],[yPixelsFromEdge_min+offSetForBumper ,yPixelsFromEdge_max-offSetForBumper],'Color', 'r','LineWidth', 2);%rightline
line([xPixelsFromEdge_min+offSetForBumper,xPixelsFromEdge_max-offSetForBumper],[yPixelsFromEdge_min+offSetForBumper ,yPixelsFromEdge_min+offSetForBumper],'Color', 'r','LineWidth', 2);%topline
line([xPixelsFromEdge_min+offSetForBumper,xPixelsFromEdge_max-offSetForBumper],[yPixelsFromEdge_max-offSetForBumper ,yPixelsFromEdge_max-offSetForBumper],'Color', 'r','LineWidth', 2);%bottomline
%Apply offset to get wall ball radius offset
line([xPixelsFromEdge_min+offSetForBall,xPixelsFromEdge_min+offSetForBall],[yPixelsFromEdge_min+offSetForBall ,yPixelsFromEdge_max-offSetForBall],'Color', 'w','LineWidth', 2);%leftline
line([xPixelsFromEdge_max-offSetForBall,xPixelsFromEdge_max-offSetForBall],[yPixelsFromEdge_min+offSetForBall ,yPixelsFromEdge_max-offSetForBall],'Color', 'w','LineWidth', 2);%rightline
line([xPixelsFromEdge_min+offSetForBall,xPixelsFromEdge_max-offSetForBall],[yPixelsFromEdge_min+offSetForBall ,yPixelsFromEdge_min+offSetForBall],'Color', 'w','LineWidth', 2);%topline
line([xPixelsFromEdge_min+offSetForBall,xPixelsFromEdge_max-offSetForBall],[yPixelsFromEdge_max-offSetForBall ,yPixelsFromEdge_max-offSetForBall],'Color', 'w','LineWidth', 2);%bottomline
%%
%{
%Find Circles want to be own function but in this WS will work for now so
%we dont have to call lines
imshow(img);
hold on; axis on;
[centersBright, radiiBright, metricBright] = imfindcircles(img,[12 22],'Sensitivity', 0.9, 'EdgeThreshold', 0.3339917,'ObjectPolarity','bright');
[centersDark, radiiDark, metricDark] = imfindcircles(img,[12 22],'Sensitivity', 0.9, 'EdgeThreshold', 0.3339917,'ObjectPolarity','dark');
viscircles(centersBright, radiiBright,'Color','b');
viscircles(centersDark, radiiDark,'LineStyle','--');
%}