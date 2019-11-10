function [center_x,center_y] = identi_que(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Ball que
%------------------------------------------------------


%OR\L*A*B
I = rgb2lab(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 46.202;
channel1Max = 94.421;

% Define thresholds for channel 2 based on histogram settings
channel2Min = -17.862;
channel2Max = 6.157;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 1.236;
channel3Max = 51.835;

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
%ball que
%{
    % Convert RGB image to chosen color space
    I = rgb2hsv(img);
    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.991;
    channel1Max = 0.271;
    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.000;
    channel2Max = 0.836;
    % Define thresholds for channel 3 based on histogram settings
    channel3Min = 0.610;
    channel3Max = 0.979;
    % Create mask based on chosen histogram thresholds
    sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
    BW = sliderBW;
    % Initialize output masked image based on input image.
    maskedRGBImage= img;
    % Set background pixels where BW is false to zero.
    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
%}
[c, r, m] = imfindcircles(maskedRGBImage,[30 50],'Sensitivity', .9, 'EdgeThreshold', .2);
try
    cSelect = c(2,:);
    rSelect = r(2);
    mSelect = m(2);
catch
    disp('No Que_ball Detected')


end
center_x=cSelect(1);
center_y=cSelect(2);
end