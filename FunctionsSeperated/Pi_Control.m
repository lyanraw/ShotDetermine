imshow(J1);
hold on; axis on;
([c r] = imfindcircles(J1,[12 22],'Sensitivity', 0.9, 'EdgeThreshold', 0.3339917);
viscircles(c,r,'Color','red');
line([100.27,101.136],[101.72,617.40],'Color', 'red','LineWidth', 2);%leftline
line([1176.7,1168.9],[107.43,625.9],'Color', 'red','LineWidth', 2);%rightline
line([100.27,1176.6],[101.72,107.43],'Color', 'red','LineWidth', 2);%topline
line([101.136,1168.57],[617.4,625.3],'Color', 'red','LineWidth', 2);%bottomline

line([82.46,81.326],[83.91,635.21],'Color', 'y','LineWidth', 2);%leftline
line([1194.51,1186.71],[89.62,643.71],'Color', 'y','LineWidth', 2);%rightline
line([82.89,1194.41],[83.91,89.62],'Color', 'y','LineWidth', 2);%topline
line([81.326,1186.38],[635.21,643.11],'Color', 'y','LineWidth', 2);%bottomline

line([627.898,627.898],[628,725],'Color', 'c','LineWidth', 2);%centerbottom
line([634.75,634.75],[100,10],'Color', 'c','LineWidth', 2);%centertop
disp(c)
%disp(r);
clear B1x B1y B2x B2y
B1x = c(1,1);
B1y = c(1,2);
B2x = c(2,1);
B2y = c(2,2);
balls = [B1x B1y;B2x B2y]; %stores x y coordinates for two balls

pocket = [100.27 101.72; %1
    634.75 100;          %2
    1176.6 107.433;      %3
    1168.57 625.3;       %4
    627.898 628;         %5
    101.136 617.4];      %6
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
    rDist = 37.5525; %(2r)=distance to hit center
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
    if angles(i)<90
        line([B2x,targetX],[B2y,targetY],'Color',color(i),'LineWidth',2); %line add behind target direction
        line([xi,B2x], [yi,B2y],'Color', color(i),'LineWidth',2); %draw pocket to target ball center line
        line([B1x,targetX],[B1y,targetY],'Color',color(i),'LineWidth',2); %line from cue to target cue center
    end
                   
end

clear pi
clear cam
%return vect, 
%end

    


