%%
global L R T B ball_d  bumper_w
%params youll get from running the rest
%{
T =64.0966;
R =2.7069e+03;
B =1.4757e+03;
L =110.4567;
ball_d=84;
bumper_w=46.666666666666666667*2;
%}


%Shot Determine



%d = imdistline;


[c, r, m] = imfindcircles(maskedBalls,[30 50],'Sensitivity', .9, 'EdgeThreshold', .2);
viscircles(c, r,'Color','b');


%starts at 0,0 and rotates cw
pocket = [R-(ball_d/2)+bumper_w,T+(ball_d/2)+bumper_w;...         %1, Top Right
    R-(ball_d/2)+bumper_w,B-(ball_d/2)+bumper_w;...     %2, Bottom Right
    ((L+R)/2),B-(ball_d/2)+bumper_w;...           %3, Bottom middle
    L+(ball_d/2)+bumper_w,B-(ball_d/2)+bumper_w;...               %4, Bottom Left
    L+(ball_d/2)+bumper_w,T+(ball_d/2)+bumper_w;...               %5, Top left
    ((L+R)/2),T+(ball_d/2)+bumper_w];           %6, TOP MIDDLE
color = ['b','r','g','c','w','m']; %color matrix
clear balls
for i=1:length(c)
    balls{i,15}=0;
    if i==1
        balls{i,1}=i; 
        %set que at this position
        [balls{i,2},balls{i,3}]=identi_que(img);
    else
        balls{i,1}=i;           %Ball number
        balls{i,2}=c(i,1);              %Ball x
        balls{i,3}=c(i,2);              %Ball y
        for j = 1:6
            
            balls{i,4} = [balls{i,4};sqrt((pocket(j,1)-balls{i,2})^2+(pocket(j,2)-balls{i,3})^2)];
            %unitvector from pocket 2 target ball
            balls{i,5} = [balls{i,5} ; (balls{i,2}-pocket(j,1))/balls{i,4}(j)];%vectx
            balls{i,6} = [balls{i,6} ; (balls{i,3}-pocket(j,2))/balls{i,4}(j)];%vecty
            %find new location for cue ball
            balls{i,7} = [balls{i,7} ; balls{i,2}+ball_d*balls{i,5}(j)];%TargetX
            balls{i,8} = [balls{i,8} ; balls{i,3}+ball_d*balls{i,6}(j)];%target Y
            %used to find unit vector from target ball to pocket
            balls{i,9} = [balls{i,9} ; (pocket(j,1)-balls{i,2}) / (balls{i,4}(j))];%vectortoPocketX
            balls{i,10} = [balls{i,10} ; (pocket(j,2)-balls{i,3}) / (balls{i,4}(j))];%   vectortoPocketY
            %used to find unit vector for cue ball directio
            balls{i,11} = [balls{i,11} ; sqrt((balls{i,7}(j)-balls{1,2})^2+(balls{i,8}(j)-balls{1,3})^2)]; %distance cue ball travel
            balls{i,12} = [balls{i,12} ; (balls{i,7}(j)-balls{1,2})/(balls{i,11}(j))]; %distance cue ball travelx
            balls{i,13} = [balls{i,13} ; (balls{i,8}(j)-balls{1,3})/(balls{i,11}(j))]; %distance cue ball travely
            %angle
            balls{i,14} = [balls{i,14} ; acosd(dot([balls{i,12}(j) balls{i,13}(j)],[balls{i,9}(j) balls{i,10}(j)]))];
            if balls{i,14}(j)<85
                line([balls{i,2},balls{i,7}(j)],[balls{i,3},balls{i,8}(j)],'Color',color(i),'LineWidth',2); %line add behind target direction
                line([pocket(j,1),balls{i,2}], [pocket(j,2),balls{i,3}],'Color', color(i),'LineWidth',2); %draw pocket to target ball center line
                line([balls{1,2},balls{i,7}(j)],[balls{1,3},balls{i,8}(j)],'Color',color(i),'LineWidth',2); %line from cue to target cue center
                %determine which angles are possible & put into array
                balls{i,15}=[balls{i,15} ; 1];
            else
                balls{i,15}=[balls{i,15} ; 0];
            end
            
        end
    end
end
%{

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

for i=1;length(c)
    vars=7;
    for j = 1:7:42
        disp(['(' num2str(i) ',' num2str(j) ','])
        disp(['(' num2str(i) ',' num2str(j+1) ','])
        disp(['(' num2str(i) ',' num2str(j+2) ','])
        disp(['(' num2str(i) ',' num2str(j+3) ','])
        disp(['(' num2str(i) ',' num2str(j+4) ','])
        disp(['(' num2str(i) ',' num2str(j+5) ','])
        disp(['(' num2str(i) ',' num2str(j+6) ','])
    end
end
%}