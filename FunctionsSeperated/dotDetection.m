function [TopLine,BottomLine,RightLine,LeftLine]=dotDetection(img)
%A series of mask to be applied to the table to identify each white dot
%----------------------------------------------------
% Convert RGB image into L*a*b* color space.
X = rgb2lab(img);

% Create empty mask.
BW = false(size(X,1),size(X,2));
% Local graph cut
xPos = [146.7341 2673.1607 2673.1607 146.7341 ];
yPos = [107.4459 107.4459 1442.6651 1442.6651 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [];
backgroundInd = [];
L = superpixels(X,21382,'IsInputLab',true);
% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);
% Flood fill
row = 280;
column = 112;
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
%%
%masked image to grey for edge detection
figure('Name', 'Masked out')
imshow(maskedImage)
figure('Name','OG')
imshow(img);
hold on; axis on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%NEED to find pixel Ratio determind on img size%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%[c, r, m] = imfindcircles(grey,[2 6],'Sensitivity', 0.9, 'EdgeThreshold', 0.3339917);
[c, r, m] = imfindcircles(maskedImage,[5 12],'Sensitivity', 0.9, 'EdgeThreshold', 0.3339917);
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
    elseif c(i,1) >= 2500 
        %Right side
        R=[R,;c(i,1)];
    else
        if c(i,2)<= 100
            %top
            T=[T,c(i,2)];
        elseif c(i,2)>= 1400
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
end