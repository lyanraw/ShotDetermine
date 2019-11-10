function PossibleShots(img)
%%
global ball_d bumper_w mm2pixel pixel2mm ballInfo pocket O A Ts
%Shot Determine
%d = imdistline;
[c, r, m] = imfindcircles(img,[30 50],'Sensitivity', .9, 'EdgeThreshold', .2);
viscircles(c, r,'Color','b');

color = {[0 0 0];[1 0 0];[0.4660 0.6740 0.1880];[0 0 1];[0.8500 0.3250 0.0980];[1 0 1]}; %color matrix
img_size=size(rgb2gray(img));
count=1;
for i=1:length(c)
    balls{i,16}=0;
    if i==1
        balls{1,i}=i;
        %set que at this position
        [balls{i,2},balls{i,3}]=identi_que(img);
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
                %PossibleShot{count,1}=balls{i,9} balls{i,10}
                
            end
        end
    end
end
ballInfo=cell2table(balls);
ballInfo.Properties.VariableNames={'num','x','y','p2gb_mag','p2gb_vectX','p2gb_vect_Y','ghost_x','ghost_y','gb2p_x','gb2p_y','cue_mag','cue_magX','cue_magY','angle','Possible','Bullshit Col'};


    
   
    
    
  
end