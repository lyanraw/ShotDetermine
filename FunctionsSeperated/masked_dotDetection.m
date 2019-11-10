function [TopLine,BottomLine,RightLine]=dotDetection(img)
%A series of mask to be applied to the table to identify each white dot
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
maskedUp= RGB;
% Set background pixels where BW is false to zero.
maskedUp(repmat(~BW,[1 1 3])) = 0;
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



