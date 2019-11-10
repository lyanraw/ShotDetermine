function [maskedRGBImage] = identi_ball1(RGB)
%Ball 1
I = rgb2hsv(img);
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.079;
channel1Max = 0.227;
% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.516;
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
maskedRGBImage = img;
% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
%%
% ball 1 lab
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2lab(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 62.051;
channel1Max = 92.115;

% Define thresholds for channel 2 based on histogram settings
channel2Min = -24.169;
channel2Max = 19.178;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 28.906;
channel3Max = 84.796;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
end