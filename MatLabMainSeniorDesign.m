clear all
clc

%globals to other Directories
global ball_d bumper_w mm2pixel pixel2mm ballInfo  Directory2Zip
%globals from other directories
global pocket color X Y Z O A Ts balls cameraParams




Directory2Zip='C://Users/ryanl/Downloads/Senior_Project_kyle/Senior_Project';%directory to this  Zip

mm2pixel=1/1.4992;
pixel2mm=1.4992;
ball_d=58*mm2pixel;
bumper_w=70*mm2pixel;
%calls get image to obtain a img from pi camera or toload img
[img]=getImage();
%done to perform all the mask to identify dots around table, Returns an
%array of four elements representing their respective max values
[T,B,R,L]=dotDetection(img);
%Draws rectangle with respect to dot calbration GOLDEN COLOR
DrawTableFeatures(L,R,T,B);
PossibleShots(img);
disp('Right now it displays as 1X6 cell array... to view how its supposed to look click on variable and it will display correctly.');
ballInfo
%%
function [img]=getImage()
clc
clear pi c r circle
global cameraParams Directory2Zip
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
    img = undistortImage(img,cameraParams);
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
%figure('Name','Original Image','NumberTitle','off')
%imshow(img)
end

%%
function [img]=getImage()
clc
clear pi c r circle
global cameraParams Directory2Zip
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
    img = undistortImage(img,cameraParams);
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
%figure('Name','Original Image','NumberTitle','off')
%imshow(img)
end
%%
function PossibleShots(img)
%%
global ball_d bumper_w mm2pixel pixel2mm ballInfo pocket X Y Z O A Ts balls
%Shot Determine
%d = imdistline;
[c, r, m] = imfindcircles(img,[15 23],'Sensitivity', .9, 'EdgeThreshold', .2);
viscircles(c, r,'Color','b');
color = {[0 0 0];[1 0 0];[0.4660 0.6740 0.1880];[0 0 1];[0.8500 0.3250 0.0980];[1 0 1]}; %color matrix
img_size=size(rgb2gray(img));
for i=1:length(c)
    balls{i,16}=0;
    if i==1
        balls{1,1}=i;
        %set que at this position
        [balls{1,2},balls{1,3}]=identi_que(img);
        balls{i,4} = [0 0 0 0 0 0];
        balls{i,5} = [0 0 0 0 0 0];
        balls{i,6} = [0 0 0 0 0 0];
        balls{i,7} = [0 0 0 0 0 0];
        balls{i,8} = [0 0 0 0 0 0];
        balls{i,9} = [0 0 0 0 0 0];
        balls{i,10} = [0 0 0 0 0 0];
        %used to find unit vector for cue ball directio
        balls{i,11} = [0 0 0 0 0 0];
        balls{i,12} = [0 0 0 0 0 0];
        balls{i,13} = [0 0 0 0 0 0];
        balls{i,14} = [0 0 0 0 0 0];
        balls{i,15} = [0 0 0 0 0 0];
    else
        balls{i,1}=i;           %Ball number
        balls{i,2}=c(i,1);      %Ball [x y]
        balls{i,3}=c(i,2);      %Ball y
        for j = 1:6
            
            %mag
            balls{i,4} = [balls{i,4} sqrt((pocket(j,1)-balls{i,2})^2+(pocket(j,2)-balls{i,3})^2)];
            %unitvector from pocket 2 target ball
            balls{i,5} = [balls{i,5}  (balls{i,2}-pocket(j,1))/balls{i,4}(j)];%vectx
            balls{i,6} = [balls{i,6}  (balls{i,3}-pocket(j,2))/balls{i,4}(j)];%vecty
            %find new location for cue ball
            balls{i,7} = [balls{i,7}  balls{i,2}+ball_d*balls{i,5}(j)];%TargetX
            balls{i,8} = [balls{i,8}  balls{i,3}+ball_d*balls{i,6}(j)];%target Y
            %used to find unit vector from target ball to pocket
            balls{i,9} = [balls{i,9}  (pocket(j,1)-balls{i,2}) / (balls{i,4}(j))];%vectortoPocketX
            balls{i,10} = [balls{i,10}  (pocket(j,2)-balls{i,3}) / (balls{i,4}(j))];%   vectortoPocketY
            %used to find unit vector for cue ball directio
            balls{i,11} = [balls{i,11}  sqrt((balls{i,7}(j)-balls{1,2})^2+(balls{i,8}(j)-balls{1,3})^2)]; %distance cue ball travel
            balls{i,12} = [balls{i,12}  (balls{i,7}(j)-balls{1,2})/(balls{i,11}(j))]; %distance cue ball travelx
            balls{i,13} = [balls{i,13}  (balls{i,8}(j)-balls{1,3})/(balls{i,11}(j))]; %distance cue ball travely
            %angle
            balls{i,14} = [balls{i,14}  acosd(dot([balls{i,12}(j) balls{i,13}(j)],[balls{i,9}(j) balls{i,10}(j)]))];
%%
            if balls{i,14}(j)<85%test if each shot can make contact at less than 85 deg
                %next will test if Ghost ball will exceed boundries since item is abritary
                if ((([balls{i,7}(j),balls{i,8}(j)]) >=(img_size./2))==[1 1])%bottom right
                    %balls{i,16}=[balls{i,16}  4];
                    %target ball center is in bottom right of img so tb_x or tb_y cant be
                    %greater than bottom right pockets x,y
                    if (balls{i,7}(j)>pocket(2,1)|balls{i,8}(j)>pocket(2,2))%bottom right
                        balls{i,15}=[balls{i,15} 0];%shotpossible==FALSE
                    else
                        balls{i,15}=[balls{i,15} 1];%not eliminated yet
                    end
                elseif((([balls{i,7}(j),balls{i,8}(j)])>=(img_size./2))==[1 0])% top right
                    %balls{i,16}=[balls{i,16} 1];
                    if (balls{i,7}(j)>pocket(1,1)|balls{i,8}(j)<pocket(1,2))%check if GB exceeds boundries
                        balls{i,15}=[balls{i,15}  0];%shotpossible==FALSE
                    else
                        balls{i,15}=[balls{i,15}  1];%not eliminated yet
                    end
                elseif((([balls{i,7}(j),balls{i,8}(j)])>=(img_size./2))==[0 0])%top left
                    %balls{i,16}=[balls{i,16}  2];
                    if (balls{i,7}(j)<pocket(5,1)|balls{i,8}(j)<pocket(5,2))%check if GB exceeds boundries
                        balls{i,15}=[balls{i,15}  0];%shotpossible==FALSE
                    else
                        balls{i,15}=[balls{i,15}  1];%not eliminated yet
                    end
                else%bottom left [0 1]
                    %balls{i,16}=[balls{i,16}  1];
                    if (balls{i,7}(j)<pocket(4,1)|balls{i,8}(j)>pocket(4,2))%check if GB exceeds boundries
                        balls{i,15}=[balls{i,15} ; 0];%shotpossible==FALSE
                    else
                        balls{i,15}=[balls{i,15}  1];%not eliminated yet
                    end
                end
            else
                balls{i,15}=[balls{i,15}  0];%
            end
            %%
            if balls{i,15}(j)==1
                
                line([balls{1,2},balls{i,7}(j)],[balls{1,3},balls{i,8}(j)],'Color',color{j},'LineWidth',2); %line from cue to target cue center
                viscircles([balls{i,7}(j) balls{i,8}(j)] , ball_d/2, 'Color', color{j}, 'LineStyle', '--');
                %waitkeyin=input('enter to go to next step:');
                line([pocket(j,1),c(i,1)], [pocket(j,2),c(i,2)],'Color', color{j},'LineWidth',2); %draw pocket to target ball center line
                line([balls{i,2},balls{i,7}(j)],[balls{i,3},balls{i,8}(j)],'Color',color{j},'LineWidth',2); %line add behind target direction
                
                O=-(acosd(dot([1 0],[balls{i,12}(j) balls{i,13}(j)])));
                disp('X,    Y,    Z,    O,      A,      T')
                disp(['angle between vectors cue2ghost and ghost2pocket: ' num2str(balls{i,14}(j)) ', magnitude of cue ball: ' num2str(balls{i,11}(j))])
                A=20;
                Ts=-90;
                [xx,yy]=size(rgb2gray(img));
                
                X=(balls{1,2})*pixel2mm;
                Y=balls{1,3}*pixel2mm;
                Z=-50;
                disp([num2str(X) ', ' num2str(Y) ', ' num2str(Z) ', ' num2str(O) ', ' num2str(A) ', ' num2str(Ts)]) 
                waitkeyin=input('enter to go to next step:');
                %PossibleShot{count,1}=balls{i,9} balls{i,10}
                %r = centerCropWindow2d(size(img),[1200 600]);
                %imshow(imcrop(img,r));
            end
        end
    end
end
ballInfo=cell2table(balls);
ballInfo.Properties.VariableNames={'num','x','y','p2gb_mag','p2gb_vectX','p2gb_vect_Y','ghost_x','ghost_y','gb2p_x','gb2p_y','cue_mag','cue_magX','cue_magY','angle','Possible','Bullshit Col'};


    
   
    
    
  
end
%%
function [TopLine,BottomLine,RightLine,LeftLine]=dotDetection(img)
%A series of mask to be applied to the table to identify each white dot
%----------------------------------------------------
% Convert img image into L*a*b* color space.
I = rgb2lab(img);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 61.065;
channel1Max = 99.901;

% Define thresholds for channel 2 based on histogram settings
channel2Min = -17.299;
channel2Max = 61.238;

% Define thresholds for channel 3 based on histogram settings
channel3Min = -22.512;
channel3Max = 59.994;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = img;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;


% Convert RGB image into L*a*b* color space.
X = rgb2lab(maskedRGBImage);

% Create empty mask.
BW = false(size(X,1),size(X,2));

% Local graph cut
xPos = [56.9303 1247.8350 1247.8350 56.9303 ];
yPos = [45.8301 45.8301 669.8888 669.8888 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [44524 44534 44537 45960 46463 47266 47994 47995 48331 48336 48356 49081 50025 50029 50889 51427 52968 52972 52975 52980 52983 52985 52988 53681 53716 53784 54048 54391 54392 54395 54397 54990 55233 56022 56032 56036 56053 56225 56323 56327 56361 56373 56385 56404 56407 56604 56679 58387 58392 58401 58412 58422 58435 58448 58844 58851 59178 59581 59918 59923 60311 60323 60329 60330 62491 63407 66289 72774 74806 75526 79976 82138 83446 90063 93526 97266 100869 101451 103030 105773 107935 108790 112256 117297 119595 120177 124170 133117 133118 135278 137913 144638 148398 153659 156878 157383 159546 161350 164950 177404 180283 182225 182958 205256 211015 230478 252074 258551 259847 262004 265031 307010 307015 319603 337895 352282 354881 365102 365444 403253 406133 426309 427458 434083 436256 436389 441296 444298 448496 452938 457843 459881 462163 474549 480883 482469 490243 494565 496869 497446 501769 504651 508971 513167 518472 532166 540085 545125 546287 561685 573925 575087 588665 589045 596964 598127 603442 610642 626482 643042 656002 673282 677254 683362 688401 723673 728713 740240 782720 787040 792800 820738 821457 832979 837439 838897 841060 845220 846079 848820 848990 851840 853141 853280 855301 857463 861237 861924 862677 867685 868405 870599 872098 872116 872120 873449 873636 873648 873658 873674 873699 873715 873732 873748 873761 873775 873790 873808 873825 874197 874349 874356 875061 875289 875637 876015 876789 877050 877212 877847 877860 877926 878532 878626 878638 879252 879953 880777 881483 881489 882094 882837 883288 883290 883297 883307 883318 883329 883342 883343 883362 883556 884294 884987 884988 885015 885744 885749 888652 888658 888662 888668 888672 891976 894859 894866 ];
backgroundInd = [];
L = superpixels(X,4708,'IsInputLab',true);

% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);

% Local graph cut
xPos = [107.1015 1164.2164 1164.2164 107.1015 ];
yPos = [96.0012 96.0012 583.6296 583.6296 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [];
backgroundInd = [];
L = superpixels(X,4708,'IsInputLab',true);

% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);

% Local graph cut
xPos = [1040.1088 1174.7787 1174.7787 1040.1088 ];
yPos = [266.7592 266.7592 373.2628 373.2628 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [];
backgroundInd = [];
L = superpixels(X,4708,'IsInputLab',true);

% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);

% Flood fill
row = 13;
column = 34;
tolerance = 5.000000e-02;
normX = sum((X - X(row,column,:)).^2,3);
normX = mat2gray(normX);
addedRegion = grayconnected(normX, row, column, tolerance);
BW = BW | addedRegion;

% Flood fill
row = 157;
column = 24;
tolerance = 5.000000e-02;
normX = sum((X - X(row,column,:)).^2,3);
normX = mat2gray(normX);
addedRegion = grayconnected(normX, row, column, tolerance);
BW = BW | addedRegion;

% Local graph cut
xPos = [6.7592 162.5538 162.5538 6.7592 ];
yPos = [567.7861 567.7861 720.5000 720.5000 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [16508 17245 17925 17937 20132 20139 20785 35984 38776 39487 39489 40924 43081 43908 46788 51829 59029 63350 69830 71990 77030 81349 84946 88542 92140 94293 96446 97124 97126 97129 97136 97142 97149 97153 97157 102946 108710 109431 112311 ];
backgroundInd = [];
L = superpixels(X,4708,'IsInputLab',true);

% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);

% Local graph cut
xPos = [0.5000 49.8888 49.8888 0.5000 ];
yPos = [587.1504 587.1504 719.1797 719.1797 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [];
backgroundInd = [];
L = superpixels(X,4708,'IsInputLab',true);

% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);

% Local graph cut
xPos = [0.5000 99.1797 99.1797 0.5000 ];
yPos = [0.5000 0.5000 128.5685 128.5685 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [];
backgroundInd = [];
L = superpixels(X,4708,'IsInputLab',true);

% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);

% Local graph cut
xPos = [1129.8888 1275.1210 1275.1210 1129.8888 ];
yPos = [600.3533 600.3533 720.5000 720.5000 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [];
backgroundInd = [];
L = superpixels(X,4708,'IsInputLab',true);

% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);

% Local graph cut
xPos = [79.8154 824.4609 824.4609 79.8154 ];
yPos = [692.7738 692.7738 720.5000 720.5000 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [113743 116623 117343 123823 126703 128863 130304 134627 142549 149032 153353 159838 163438 170639 176400 192957 200155 223187 228223 229667 231822 243347 248387 255589 264232 270715 280078 291599 295200 300960 307440 314640 318240 318960 320399 321839 332639 339837 340557 341277 ];
backgroundInd = [];
L = superpixels(X,4708,'IsInputLab',true);

% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);

% Local graph cut
xPos = [63.9719 98.2995 98.2995 63.9719 ];
yPos = [195.4633 195.4633 625.8790 625.8790 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [];
backgroundInd = [];
L = superpixels(X,4708,'IsInputLab',true);

% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);

% Local graph cut
xPos = [549.8399 585.0477 585.0477 549.8399 ];
yPos = [29.9866 29.9866 75.7567 75.7567 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = 411177 ;
backgroundInd = [];
L = superpixels(X,4708,'IsInputLab',true);

% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);

% Local graph cut
xPos = [24.3631 164.3142 164.3142 24.3631 ];
yPos = [561.6247 561.6247 672.5293 672.5293 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [];
backgroundInd = [];
L = superpixels(X,4708,'IsInputLab',true);

% Convert L*a*b* range to [0 1]
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);

% Invert mask
BW = imcomplement(BW);

% Create masked image.
maskedImage = maskedRGBImage;
maskedImage(repmat(~BW,[1 1 3])) = 0;

%%
%masked image to grey for edge detection
%figure('Name', 'Masked out')
%imshow(maskedImage)
grey=rgb2gray(maskedImage);
figure('Name','OG')
imshow(img);
hold on; axis on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%NEED to find pixel Ratio determind on img size%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[c, r, m] = imfindcircles(grey,[2 6],'Sensitivity', 0.9, 'EdgeThreshold', 0.5);
%viscircles(c, r,'Color','b');
%[c, r, m] = imfindcircles(maskedImage,[5 12],'Sensitivity', 0.9, 'EdgeThreshold', 0.3339917);
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
    if c(i,1) <= 100
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
end
%%
function DrawTableFeatures(L,R,T,B)
%draws both with respect to lines detected

%globals to other Directories
global pocket
%globals from other directories
global ball_d bumper_w 

bOS=bumper_w+(ball_d/2);%ball offset
%draw this to check params
line([L,L],[T ,B],'Color', 'm','LineWidth', 1);%L
line([R,R],[T ,B],'Color', 'm','LineWidth', 1);%R
line([L,R],[T ,T],'Color', 'm','LineWidth', 1);%T
line([L,R],[B ,B],'Color', 'm','LineWidth', 1);%B

%%
%commented out to save computing time
plot(L,T,'om','LineWidth',1);                              %top Left
plot(R,T,'om','LineWidth',1);    %top Right
plot(L,B,'om','LineWidth',1);    %Bottom Left
plot(R,B,'om','LineWidth',1);    %Bottom Right Magenta

plot(((L+R)/2),B,'oc','LineWidth',1);    %Bottom center
plot(((L+R)/2),T,'oc','LineWidth',1);    %Top center
%%
%Apply offset to get wall bumbper size
line([L+bumper_w,L+bumper_w],[T+bumper_w ,B-bumper_w],'Color', 'r','LineWidth', 1);%L
line([R-bumper_w,R-bumper_w],[T+bumper_w ,B-bumper_w],'Color', 'r','LineWidth', 1);%R
line([L+bumper_w,R-bumper_w],[T+bumper_w ,T+bumper_w],'Color', 'r','LineWidth', 1);%T
line([L+bumper_w,R-bumper_w],[B-bumper_w ,B-bumper_w],'Color', 'r','LineWidth', 1);%B
%%
line([L+bOS,L+bOS],[T+bOS ,B-bOS],'Color', 'w','LineWidth', 1);%L
line([R-bOS,R-bOS],[T+bOS ,B-bOS],'Color', 'w','LineWidth', 1);%R
line([L+bOS,R-bOS],[T+bOS ,T+bOS],'Color', 'w','LineWidth', 1);%T
line([L+bOS,R-bOS],[B-bOS ,B-bOS],'Color', 'w','LineWidth', 1);%B 

%%
%Pocket Locations
          %Top Right  %Bottom Right %Bottom middle %Bottom Left %Top Left    %Top Middle
pocket = [R-bOS T+bOS; R-bOS B-bOS; (L+R)/2 B-bOS; L+bOS B-bOS; L+bOS T+bOS; (L+R)/2 T+bOS];  
Cpocket = [R-bumper_w T+bumper_w; R-bumper_w B-bumper_w; (L+R)/2 B-bumper_w; L+bumper_w B-bumper_w; L+bumper_w T+bumper_w; (L+R)/2 T+bumper_w];  
color = {[0 0 0];[1 0 0];[0.4660 0.6740 0.1880];[0 0 1];[0.8500 0.3250 0.0980];[1 0 1]}; %color matrix
for i=1:6
    viscircles(Cpocket(i,:), ball_d, 'Color', color{i}, 'LineStyle', '-.');
end
end

try
    r = raspi('10.100.114.59','pi','SeniorProject');
    cam = cameraboard(r,'Resolution','1280x720','Quality',100,...
        'Brightness',55,'Contrast', 0,'Saturation',0,'Sharpness',0);
    cam.HorizontalFlip = 1; %Flip upside down
    img = snapshot(cam);%capture image
catch
    img=imread([Directory2Zip  '/Photos/img3_3.png']);
end
try
    %img = undistortImage(img,cameraParams);
catch
    for i=1:40
        if (i<10)
            imageFileNames{i}=[Directory2Zip '/Photos/image_000'  num2str(i) '.png'];
        else
            imageFileNames{i}=[Directory2Zip '/Photos/image_00'  num2str(i) '.png'];
        end
    end
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
    imageFileNames = imageFileNames(imagesUsed);
    originalImage = imread(imageFileNames{1});
    [mrows, ncols, ~] = size(originalImage);
    squareSize = 22;  % in units of 'millimeters'
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
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
end
%%
function DrawTableFeatures(L,R,T,B)
%globals to other Directories
global pocket
%globals from other directories
global ball_d bumper_w 
bOS=bumper_w+(ball_d/2);%ball offset
line([L,L],[T ,B],'Color', 'm','LineWidth', 1);%L
line([R,R],[T ,B],'Color', 'm','LineWidth', 1);%R
line([L,R],[T ,T],'Color', 'm','LineWidth', 1);%T
line([L,R],[B ,B],'Color', 'm','LineWidth', 1);%B
plot(L,T,'om','LineWidth',1);    %top Left
plot(R,T,'om','LineWidth',1);    %top Right
plot(L,B,'om','LineWidth',1);    %Bottom Left
plot(R,B,'om','LineWidth',1);    %Bottom Right Magenta
plot(((L+R)/2),B,'oc','LineWidth',1);    %Bottom center
plot(((L+R)/2),T,'oc','LineWidth',1);    %Top center
%%
line([L+bumper_w,L+bumper_w],[T+bumper_w ,B-bumper_w],'Color', 'r','LineWidth', 1);%L
line([R-bumper_w,R-bumper_w],[T+bumper_w ,B-bumper_w],'Color', 'r','LineWidth', 1);%R
line([L+bumper_w,R-bumper_w],[T+bumper_w ,T+bumper_w],'Color', 'r','LineWidth', 1);%T
line([L+bumper_w,R-bumper_w],[B-bumper_w ,B-bumper_w],'Color', 'r','LineWidth', 1);%B
%%
line([L+bOS,L+bOS],[T+bOS ,B-bOS],'Color', 'w','LineWidth', 1);%L
line([R-bOS,R-bOS],[T+bOS ,B-bOS],'Color', 'w','LineWidth', 1);%R
line([L+bOS,R-bOS],[T+bOS ,T+bOS],'Color', 'w','LineWidth', 1);%T
line([L+bOS,R-bOS],[B-bOS ,B-bOS],'Color', 'w','LineWidth', 1);%B 
%%
%Pocket Locations
          %Top Right  %Bottom Right %Bottom middle %Bottom Left %Top Left    %Top Middle
pocket = [R-bOS T+bOS; R-bOS B-bOS; (L+R)/2 B-bOS; L+bOS B-bOS; L+bOS T+bOS; (L+R)/2 T+bOS];  
Cpocket = [R-bumper_w T+bumper_w; R-bumper_w B-bumper_w; (L+R)/2 B-bumper_w; L+bumper_w B-bumper_w; L+bumper_w T+bumper_w; (L+R)/2 T+bumper_w];  
color = {[0 0 0];[1 0 0];[0.4660 0.6740 0.1880];[0 0 1];[0.8500 0.3250 0.0980];[1 0 1]}; %color matrix
for i=1:6
    viscircles(Cpocket(i,:), ball_d, 'Color', color{i}, 'LineStyle', '-.');
end
end
%%
function out = prepLab(in)
out = in;
out(:,:,1) = in(:,:,1) / 100;  % L range is [0 100].
out(:,:,2) = (in(:,:,2) + 86.1827) / 184.4170;  % a* range is [-86.1827,98.2343].
out(:,:,3) = (in(:,:,3) + 107.8602) / 202.3382;  % b* range is [-107.8602,94.4780].
end
%%
function [TopLine,BottomLine,RightLine,LeftLine]=dotDetection(img)
X = rgb2lab(img);
BW = false(size(X,1),size(X,2));
xPos = [146.7341 2673.1607 2673.1607 146.7341 ];
yPos = [107.4459 107.4459 1442.6651 1442.6651 ];
m = size(BW, 1);
n = size(BW, 2);
ROI = poly2mask(xPos,yPos,m,n);
foregroundInd = [];
backgroundInd = [];
L = superpixels(X,21382,'IsInputLab',true);
scaledX = prepLab(X);
BW = BW | grabcut(scaledX,L,ROI,foregroundInd,backgroundInd);
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
viscircles(c, r,'Color','b');
viscircles(cSelect, rSelect,'Color','r','LineStyle','--');
L=[];
R=[];
T=[];
B=[];
oppsx=[];
oppsy=[];
for i=1:length(c) 
    if c(i,1) <= 200
        L=[L,;c(i,1)];
    elseif c(i,1) >= 2500 
        R=[R,;c(i,1)];
    else
        if c(i,2)<= 100
            T=[T,c(i,2)];
        elseif c(i,2)>= 1400
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
%%
function PossibleShots(img)
global ball_d bumper_w mm2pixel pixel2mm ballInfo pocket O A Ts
[c, r, m] = imfindcircles(img,[30 50],'Sensitivity', .9, 'EdgeThreshold', .2);
viscircles(c, r,'Color','b');
color = {[0 0 0];[1 0 0];[0.4660 0.6740 0.1880];[0 0 1];[0.8500 0.3250 0.0980];[1 0 1]}; %color matrix
img_size=size(rgb2gray(img));
count=1;
for i=1:length(c)
    balls{i,16}=0;
    if i==1
        balls{1,i}=i;
        [balls{i,2},balls{i,3}]=identi_que(img);
        balls{i,4} = [0 0 0 0 0 0];
        balls{i,5} = [0 0 0 0 0 0];
        balls{i,6} = [0 0 0 0 0 0];
        balls{i,7} = [0 0 0 0 0 0];
        balls{i,8} = [0 0 0 0 0 0];
        balls{i,9} = [0 0 0 0 0 0];
        balls{i,10} = [0 0 0 0 0 0];
        balls{i,11} = [0 0 0 0 0 0];
        balls{i,12} = [0 0 0 0 0 0];
        balls{i,13} = [0 0 0 0 0 0];
        balls{i,14} = [0 0 0 0 0 0];
        balls{i,15} = [0 0 0 0 0 0];
    else
        balls{i,1}=i;           %Ball number
        balls{i,2}=c(i,1);      %Ball [x y]
        balls{i,3}=c(i,2);      %Ball y
        for j = 1:6
            %mag
            balls{i,4} = [balls{i,4} sqrt((pocket(j,1)-balls{i,2})^2+(pocket(j,2)-balls{i,3})^2)];
            %unitvector from pocket 2 target ball
            balls{i,5} = [balls{i,5}  (balls{i,2}-pocket(j,1))/balls{i,4}(j)];%vectx
            balls{i,6} = [balls{i,6}  (balls{i,3}-pocket(j,2))/balls{i,4}(j)];%vecty
            %find new location for cue ball
            balls{i,7} = [balls{i,7}  balls{i,2}+ball_d*balls{i,5}(j)];%TargetX
            balls{i,8} = [balls{i,8}  balls{i,3}+ball_d*balls{i,6}(j)];%target Y
            %used to find unit vector from target ball to pocket
            balls{i,9} = [balls{i,9}  (pocket(j,1)-balls{i,2}) / (balls{i,4}(j))];%vectortoPocketX
            balls{i,10} = [balls{i,10}  (pocket(j,2)-balls{i,3}) / (balls{i,4}(j))];%   vectortoPocketY
            %used to find unit vector for cue ball directio
            balls{i,11} = [balls{i,11}  sqrt((balls{i,7}(j)-balls{1,2})^2+(balls{i,8}(j)-balls{1,3})^2)]; %distance cue ball travel
            balls{i,12} = [balls{i,12}  (balls{i,7}(j)-balls{1,2})/(balls{i,11}(j))]; %distance cue ball travelx
            balls{i,13} = [balls{i,13}  (balls{i,8}(j)-balls{1,3})/(balls{i,11}(j))]; %distance cue ball travely
            %angle
            balls{i,14} = [balls{i,14}  acosd(dot([balls{i,12}(j) balls{i,13}(j)],[balls{i,9}(j) balls{i,10}(j)]))];
%%
            if balls{i,14}(j)<85%test if each shot can make contact at less than 85 deg
                if ((([balls{i,7}(j),balls{i,8}(j)]) >=(img_size./2))==[1 1])%bottom right
                    if (balls{i,7}(j)>pocket(2,1)|balls{i,8}(j)>pocket(2,2))%bottom right
                        balls{i,15}=[balls{i,15} 0];%shotpossible==FALSE
                    else
                        balls{i,15}=[balls{i,15} 1];%not eliminated yet
                    end
                elseif((([balls{i,7}(j),balls{i,8}(j)])>=(img_size./2))==[1 0])% top right
                    if (balls{i,7}(j)>pocket(1,1)|balls{i,8}(j)<pocket(1,2))%check if GB exceeds boundries
                        balls{i,15}=[balls{i,15}  0];%shotpossible==FALSE
                    else
                        balls{i,15}=[balls{i,15}  1];%not eliminated yet
                    end
                elseif((([balls{i,7}(j),balls{i,8}(j)])>=(img_size./2))==[0 0])%top left
                    if (balls{i,7}(j)<pocket(5,1)|balls{i,8}(j)<pocket(5,2))%check if GB exceeds boundries
                        balls{i,15}=[balls{i,15}  0];%shotpossible==FALSE
                    else
                        balls{i,15}=[balls{i,15}  1];%not eliminated yet
                    end
                else
                    if (balls{i,7}(j)<pocket(4,1)|balls{i,8}(j)>pocket(4,2))%check if GB exceeds boundries
                        balls{i,15}=[balls{i,15} ; 0];%shotpossible==FALSE
                    else
                        balls{i,15}=[balls{i,15}  1];%not eliminated yet
                    end
                end
            else
                balls{i,15}=[balls{i,15}  0];%
            end
            %%
            if balls{i,15}(j)==1
                
                line([c(1,1),balls{i,7}(j)],[c(1,2),balls{i,8}(j)],'Color',color{j},'LineWidth',2); %line from cue to target cue center
                viscircles([balls{i,7}(j) balls{i,8}(j)] , ball_d/2, 'Color', color{j}, 'LineStyle', '--');
                waitkeyin=input('enter to go to next step:');
                line([pocket(j,1),c(i,1)], [pocket(j,2),c(i,2)],'Color', color{j},'LineWidth',2); %draw pocket to target ball center line
                line([c(i,1),balls{i,7}(j)],[c(i,2),balls{i,8}(j)],'Color',color{j},'LineWidth',2); %line add behind target direction
                O=acosd(dot([0 1],[balls{i,12}(j) balls{i,13}(j)]));
                disp(['angle between vectors cue2ghost and ghost2pocket: ' num2str(balls{i,14}(j)) ', magnitude of cue ball: ' num2str(balls{i,11}(j))])
                A=15;
                Ts=90;
                disp(['O: ' num2str(O) ', A: ' num2str(A) ', T: ' num2str(Ts)]) 
                waitkeyin=input('enter to go to next step:');
            end
        end
    end
end
ballInfo=cell2table(balls);
ballInfo.Properties.VariableNames={'num','x','y','p2gb_mag','p2gb_vectX','p2gb_vect_Y','ghost_x','ghost_y','gb2p_x','gb2p_y','cue_mag','cue_magX','cue_magY','angle','Possible','Bullshit Col'};
end
