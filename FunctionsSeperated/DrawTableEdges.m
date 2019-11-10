function DrawTableEdges(L,R,T,B)
%both with respect to lines detected
global ball_d bumper_w bOS L R T B pocket
bOS=bumper_w+(ball_d/2);%ball offset


%plot(L,T,'om','LineWidth',1);                              %top Left
%plot(R,T,'om','LineWidth',1);    %top Right
%plot(L,B,'om','LineWidth',1);    %Bottom Left
%plot(R,B,'om','LineWidth',1);    %Bottom Right Magenta
line([L,L],[T ,B],'Color', 'm','LineWidth', 1);%L
line([R,R],[T ,B],'Color', 'm','LineWidth', 1);%R
line([L,R],[T ,T],'Color', 'm','LineWidth', 1);%T
line([L,R],[B ,B],'Color', 'm','LineWidth', 1);%B
%plot(((L+R)/2),B,'oc','LineWidth',1);    %Bottom center
%plot(((L+R)/2),T,'oc','LineWidth',1);    %Top center
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
%Top Right  %Bottom Right   %Bottom middle %Bottom Left %Top Left %TOP Middle
pocket = [R-bOS T+bOS; R-bOS B-bOS; (L+R)/2 B-bOS; L+bOS B-bOS; L+bOS T+bOS; (L+R)/2 T+bOS];  
Cpocket = [R-bumper_w T+bumper_w; R-bumper_w B-bumper_w; (L+R)/2 B-bumper_w; L+bumper_w B-bumper_w; L+bumper_w T+bumper_w; (L+R)/2 T+bumper_w];  
color = {[0 0 0];[1 0 0];[0.4660 0.6740 0.1880];[0 0 1];[0.8500 0.3250 0.0980];[1 0 1]}; %color matrix
%radius=82
for i=1:6
    
    viscircles(Cpocket(i,:), ball_d, 'Color', color{i}, 'LineStyle', '-.');

end
