%%
clear
clc
clear pi r cam; clc;
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
    img=imread([Directory2Zip  '/Photos/img2.png']);
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
figure('Name','Original Image','NumberTitle','off')
imshow(img)
