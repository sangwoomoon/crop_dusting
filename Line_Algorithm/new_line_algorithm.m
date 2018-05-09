
%%%% 변수명 규칙 %%%%

% 장애물과 관련있는 변수 : OBS_
% 필드경계와 관련있는 변수 : FLD_
% 장애물, 필드경계의 정보를 담은 변수 : 모두 대문자 처리
% X,Y 를 의미하는 좌표가 이름에 있는 변수 : 좌표만 대문자 처리
% 변하지 않는 상수 변수 : 대문자 처리
% FLD_ 또는 OBS_ 가 붙지 않는 소문자 변수 : index 변수, 판별변수, 임시저장변수

%%%%%%%%%%%%%%%%%%%%

function [WP_whole, WP, WP_time_x, WP_time_y] = new_line_algorithm(map,FLD_SIZE,OBS_NUM,OBS_SIZE,RANGE,TOT_NUM,v_t,v_acc,del_t)

clc;
clf;
%RANGE = 7.5;    % Meter
%FLD_SIZE = 40;      % number of Points
%OBS_SIZE = 12;       
%OBS_NUM = 1;        % number of Obstacles

% Graph Generation
%map = imread('field.jpg');
image(map), axis equal, hold on;

% figure(1)
% open('modified_boustrophedon_obstacle_final.fig');
% axis equal;
% hold on

% Boundary Point Setting
FLD_PT = field_boundary(FLD_SIZE);
OBS_PT = obstacle_boundary(OBS_NUM,OBS_SIZE);

% Determine CCW / CW
FLD_cw_ccw = determine_cw_ccw(FLD_PT);
OBS_cw_ccw = determine_cw_ccw(OBS_PT);

% Field Boundary Point Squeeze.
[FLD_PT_new,FLD_Ysec,FLD_Yslp] = calculate_new_BP(FLD_PT,FLD_SIZE,FLD_cw_ccw,RANGE,'FLD');
[OBS_PT_new,OBS_Ysec,OBS_Yslp] = calculate_new_BP(OBS_PT,OBS_SIZE,OBS_cw_ccw,RANGE,'OBS');


% Proceed the main Path.
[WP_whole,FLD_Ysec,cost_time] = calculate_WP(FLD_PT,FLD_PT_new,FLD_cw_ccw,FLD_Ysec,FLD_Yslp,FLD_SIZE,TOT_NUM,0,v_t,v_acc,RANGE);

% Make Sub_Paths.
for i = 1 : TOT_NUM
    [temp_wp,FLD_Ysec] = calculate_WP(FLD_PT,FLD_PT_new,FLD_cw_ccw,FLD_Ysec,FLD_Yslp,FLD_SIZE,TOT_NUM,i,v_t,v_acc,RANGE,cost_time);
    if i == 1
        WP = temp_wp;
        WP(:,3) = 0;
        WP(1,3) = 1;
        WP(length(WP(:,1)),3) = 1;
        clear temp_wp;
    else
        idx = length(WP(:,1))+1;
        WP(idx:idx+length(temp_wp(:,1))-1,1:2) = temp_wp;
        WP(idx:idx+length(temp_wp(:,1))-1,3) = 0;
        WP(idx,3) = 1;
        WP(idx+length(temp_wp(:,1))-1,3) = 1;
        clear temp_wp;
    end
end

plot(WP_whole(:,1),WP_whole(:,2));

% calculate spots for given time.
% index = 1;
% for i = 1 : length(WP)
%     if WP(i,3) == 1
%         idx(index,1) = i;
%         index = index + 1;
%     end
% end
% 
% for i = 1 : 2 : length(idx(:,1))-1
%     temp_pts = set_points(WP(idx(i,1):idx(i+1,1),1:2),v_t,v_acc,del_t);
%     if i == 1
%         WP_time_x = temp_pts(:,1);
%         WP_time_y = temp_pts(:,2);
%         clear temp_pts
%     else
%         if (length(WP_time_x(:,1)) < length(temp_pts(:,1)))
%             for k = length(WP_time_x(:,1))+1 : length(temp_pts(:,1))
%                 WP_time_x(k,1:length(WP_time_x(1,:))) = WP_time_x(length(WP_time_x(:,1)),1:length(WP_time_x(1,:)));
%                 WP_time_y(k,1:length(WP_time_y(1,:))) = WP_time_y(length(WP_time_y(:,1)),1:length(WP_time_y(1,:)));                
%             end
%         else
%             for k = length(temp_pts(:,1))+1:length(WP_time_x(:,1))
%                 temp_pts(k,1:2) = temp_pts(length(temp_pts(:,1)),1:2);
%             end
%         end
%         WP_time_x(:,length(WP_time_x(1,:))+1) = temp_pts(:,1);
%         WP_time_y(:,length(WP_time_y(1,:))+1) = temp_pts(:,2);        
%         clear temp_pts;
%     end
% end
% 
% % Play the animation.
% animation_heli(FLD_PT,0,map,WP,WP_time_x,WP_time_y,TOT_NUM,0);



function [FLD_PT] = field_boundary(FLD_SIZE)
    
for i = 1 : FLD_SIZE
    [FLD_PT(i,1), FLD_PT(i,2)] = ginput(1);
    plot(FLD_PT(i,1),FLD_PT(i,2),'ro')
end
FLD_PT(FLD_SIZE+1,:) = FLD_PT(1,:);


function [OBS_PT] = obstacle_boundary(OBS_NUM,OBS_SIZE)

for i = 1 : OBS_NUM
    for j = 1: OBS_SIZE
        [OBS_PT(j,2*(i-1)+1), OBS_PT(j,2*(i-1)+2)] = ginput(1);
        plot(OBS_PT(j,2*(i-1)+1),OBS_PT(j,2*(i-1)+2), 'bo')
    end
end
OBS_PT(OBS_SIZE+1,:) = OBS_PT(1,:);



function [OBJ_cw_ccw] = determine_cw_ccw(OBJ)

OBJ_angle = atan2(OBJ(2,2)-OBJ(1,2),OBJ(2,1)-OBJ(1,1));
OBJ_deter = [OBJ(3,1)-OBJ(1,1),OBJ(3,2)-OBJ(1,2)];
OBJ_det_cw = OBJ_deter(1,1)*sin(-OBJ_angle)+OBJ_deter(1,2)*cos(-OBJ_angle);
if (OBJ_det_cw <= 0)
    OBJ_cw_ccw = 1; % CW
else
    OBJ_cw_ccw = 0; % CCW
end


function [OBJ_new,OBJ_Ysec,OBJ_Yslp] = calculate_new_BP(OBJ,OBJ_SIZE,OBJ_cw_ccw,RANGE,flag)

OBJ_Ysec = zeros(1,length(OBJ));
OBJ_Yslp = zeros(1,length(OBJ));
for i = 1 : OBJ_SIZE
    if abs(OBJ(i+1,1) - OBJ(i,1)) < 0.1
        OBJ(i+1,1) = OBJ(i+1,1) + 0.01;
    end
    OBJ_Yslp(1,i) = (OBJ(i+1,2)-OBJ(i,2))/(OBJ(i+1,1)-OBJ(i,1));
    OBJ_Ysec(1,i) = -OBJ_Yslp(1,i)*OBJ(i,1)+OBJ(i,2);
    if (flag == 'FLD') % field case.
        if (OBJ_cw_ccw == 0)
            OBJ_Ysec(1,i) = OBJ_Ysec(1,i) + RANGE/(2*cos(atan2(OBJ(i+1,2)-OBJ(i,2),(OBJ(i+1,1)-OBJ(i,1)))));
        elseif (OBJ_cw_ccw == 1)
            OBJ_Ysec(1,i) = OBJ_Ysec(1,i) - RANGE/(2*cos(atan2(OBJ(i+1,2)-OBJ(i,2),(OBJ(i+1,1)-OBJ(i,1)))));       
        end
    elseif (flag == 'OBS') % obstacle case.
        if (OBJ_cw_ccw == 0)
            OBJ_Ysec(1,i) = OBJ_Ysec(1,i) - RANGE/(2*cos(atan2(OBJ(i+1,2)-OBJ(i,2),(OBJ(i+1,1)-OBJ(i,1)))));
        elseif (OBJ_cw_ccw == 1)
            OBJ_Ysec(1,i) = OBJ_Ysec(1,i) + RANGE/(2*cos(atan2(OBJ(i+1,2)-OBJ(i,2),(OBJ(i+1,1)-OBJ(i,1)))));       
        end        
    end
end

OBJ_Yslp(1,length(OBJ)) = OBJ_Yslp(1,1);
OBJ_Ysec(1,length(OBJ)) = OBJ_Ysec(1,1);

for i = 1 : OBJ_SIZE
    OBJ_new(i,1) = (OBJ_Ysec(1,i)-OBJ_Ysec(1,i+1))/(-OBJ_Yslp(1,i)+OBJ_Yslp(1,i+1));
    OBJ_new(i,2) = (OBJ_Yslp(1,i+1)*OBJ_Ysec(1,i)-OBJ_Yslp(1,i)*OBJ_Ysec(1,i+1))/(-OBJ_Yslp(1,i)+OBJ_Yslp(1,i+1));
end

OBJ_new(OBJ_SIZE+1,:) = OBJ_new(1,:);



function [det] = determine_on_line(pt,line_BP_1, line_BP_2)

cnt = 0;
if (min(line_BP_1(1,1),line_BP_2(1,1))<=pt(1,1)) && (max(line_BP_1(1,1),line_BP_2(1,1))>=pt(1,1))
    cnt=cnt+1;
end
if (min(line_BP_1(1,2),line_BP_2(1,2))<=pt(1,2)) && (max(line_BP_1(1,2),line_BP_2(1,2))>=pt(1,2))
    cnt=cnt+1;
end
if (cnt == 2)
    det = 1;
else
    det = 0;
end


function [cost_time] = calculate_costtime(WP_1,WP_2,v_t,v_acc)

if (distance(WP_1,WP_2) < (v_t^2/v_acc))
    cost_time = sqrt(distance(WP_1,WP_2)/v_acc);
else
    cost_time = sqrt(v_t^2)+(distance(WP_1,WP_2)-v_t^2/v_acc)/v_t;
end



function [dist] = distance(a,b)

dist = sqrt((a(1,1)-b(1,1))^2 + (a(1,2)-b(1,2))^2);


    
function [WP,FLD_Ysec,cost_time] = calculate_WP(FLD_PT,FLD_PT_new,FLD_cw_ccw,FLD_Ysec,FLD_Yslp,FLD_SIZE,TOT_NUM,num,v_t,v_acc,RANGE,cost_time)

cnt = 1;
flag = 0;
tot_time = 0;
if (num == 0)
    cost_time = 0;
end
tmp_Ysec = FLD_Ysec(1,1);
if (num <= 1)
    cnt = 3;
    WP(1,:) = FLD_PT_new(FLD_SIZE,:);
    WP(2,:) = FLD_PT_new(1,:);
    tot_time = tot_time + calculate_costtime(WP(1,:),WP(2,:),v_t,v_acc);
end

while (1)
    if (FLD_cw_ccw == 0)
        FLD_Ysec(1,1) = FLD_Ysec(1,1) + RANGE/cos(atan2(FLD_PT(2,2)-FLD_PT(1,2),(FLD_PT(2,1)-FLD_PT(1,1))));
    elseif (FLD_cw_ccw == 1)
        FLD_Ysec(1,1) = FLD_Ysec(1,1) - RANGE/cos(atan2(FLD_PT(2,2)-FLD_PT(1,2),(FLD_PT(2,1)-FLD_PT(1,1))));
    end
    for j = 1 : FLD_SIZE-1
        tmp(1,1) = (FLD_Ysec(1,1)-FLD_Ysec(1,j+1))/(-FLD_Yslp(1,1)+FLD_Yslp(1,j+1));
        tmp(1,2) = (FLD_Yslp(1,j+1)*FLD_Ysec(1,1)-FLD_Yslp(1,1)*FLD_Ysec(1,j+1))/(-FLD_Yslp(1,1)+FLD_Yslp(1,j+1));
        if (determine_on_line(tmp,FLD_PT_new(j,:),FLD_PT_new(j+1,:)))
            WP(cnt,:) = tmp(1,:);
            cnt = cnt + 1;
            flag = 1;
        end
    end
    if (flag == 0)
        cost_time = tot_time;
        break;
    end
    if (cnt > 3)
        if (distance(WP(cnt-2,:),WP(cnt-3,:)) >= distance(WP(cnt-1,:),WP(cnt-3,:)))
            tmp(1,:) = WP(cnt-1,:);
            WP(cnt-1,:) = WP(cnt-2,:);
            WP(cnt-2,:) = tmp(1,:);
        end
    elseif (cnt == 3)
        if abs(atan2(FLD_PT_new(1,2)-FLD_PT_new(FLD_SIZE,2),FLD_PT_new(1,1)-FLD_PT_new(FLD_SIZE,1))-atan2(FLD_PT_new(2,2)-FLD_PT_new(1,2),FLD_PT_new(2,1)-FLD_PT_new(1,1))) > 1
            tmp(1,:) = WP(2,:);
            WP(2,:) = WP(1,:);
            WP(1,:) = tmp(1,:);
        elseif (atan2(FLD_PT(1,2)-FLD_PT(2,2),FLD_PT(1,1)-FLD_PT(2,1))+atan2(FLD_PT(3,2)-FLD_PT(3,1),FLD_PT(2,2)-FLD_PT(2,1)) > pi/2)
            tmp(1,:) = WP(2,:);
            WP(2,:) = WP(1,:);
            WP(1,:) = tmp(1,:);
        end
    end
   
    if (cnt > 3)
        for i = 2 : 3
            tot_time = tot_time + calculate_costtime(WP(cnt-i,:),WP(cnt-i+1,:),v_t,v_acc);
        end
    elseif (cnt == 3)
        tot_time = tot_time + calculate_costtime(WP(1,:),WP(2,:),v_t,v_acc);
    end
        
    if (num ~= 0) && (cost_time < tot_time)
        cost_time = tot_time;
        break;
    end 
    flag = 0;
end
if (num == 0)
    FLD_Ysec(1,1) = tmp_Ysec;
    cost_time = tot_time / TOT_NUM;
end