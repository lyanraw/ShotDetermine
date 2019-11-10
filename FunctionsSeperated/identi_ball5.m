function [maskedRGBImage] = identi_ball5(RGB)
%ball 5     
    % Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.026;
channel1Max = 0.163;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.301;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.000;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
    [centers, radii, metric] = imfindcircles(maskedRGBImage_ball5,[12 22],'Sensitivity', 0.9, 'EdgeThreshold', 0.3339917);
    BallInfo{5,:}= {'5', centers(1,:), radii(1), metric(1)};
    BallInfo{5,1}= '5';
    BallInfo{5,2}=[0,0];
    BallInfo{5,3}=0;
    BallInfo{5,4}=0;