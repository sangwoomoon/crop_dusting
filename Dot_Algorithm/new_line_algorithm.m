
%%%% 변수명 규칙 %%%%

% 장애물과 관련있는 변수 : OBS_
% 필드경계와 관련있는 변수 : FLD_
% 장애물, 필드경계의 정보를 담은 변수 : 모두 대문자 처리
% X,Y 를 의미하는 좌표가 이름에 있는 변수 : 좌표만 대문자 처리
% 변하지 않는 상수 변수 : 대문자 처리
% FLD_ 또는 OBS_ 가 붙지 않는 소문자 변수 : index 변수, 판별변수, 임시저장변수

%%%%%%%%%%%%%%%%%%%%

clear all;
clc;
clf;
RANGE = 7.5;    % Meter
PI = 3.141592;
FLD_SIZE = 30;      % number of Points
OBS_SIZE = 8;       
OBS_NUM = 1;        % number of Obstacles

% Graph Generation
map = imread('field.jpg');
image(map), axis equal;
hold on

% Boundary Point Marking
 for i = 1 : FLD_SIZE
    [FLD_PT(i,1), FLD_PT(i,2)] = ginput(1);
    plot(FLD_PT(i,1),FLD_PT(i,2),'ro')
 end
FLD_PT(FLD_SIZE+1,:) = FLD_PT(1,:);

%Obstacle Boundary Point Marking
for i = 1 : OBS_NUM
    for j = 1: OBS_SIZE
        [OBS_PT(j,2*(i-1)+1), OBS_PT(j,2*(i-1)+2)] = ginput(1);
        plot(OBS_PT(j,2*(i-1)+1),OBS_PT(j,2*(i-1)+2), 'bo')
    end
end
OBS_PT(OBS_SIZE+1,:) = OBS_PT(1,:);

% Determine CCW / CW : BP of Field.
FLD_angle = atan2(FLD_PT(2,2)-FLD_PT(1,2),FLD_PT(2,1)-FLD_PT(1,1));
FLD_deter = [FLD_PT(3,1)-FLD_PT(1,1),FLD_PT(3,2)-FLD_PT(1,2)];
FLD_det_cw = FLD_deter(1,1)*sin(-FLD_angle)+FLD_deter(1,2)*cos(-FLD_angle);
if (FLD_det_cw <= 0)
    FLD_cw_ccw = 1; % CW
else
    FLD_cw_ccw = 0; % CCW
end

% Determine CCW / CW : BP of Obstacle.
OBS_angle = atan2(OBS_PT(2,2)-OBS_PT(1,2),OBS_PT(2,1)-OBS_PT(1,1));
OBS_deter = [OBS_PT(3,1)-OBS_PT(1,1),OBS_PT(3,2)-OBS_PT(1,2)];
OBS_det_cw = OBS_deter(1,1)*sin(-OBS_angle)+OBS_deter(1,2)*cos(-OBS_angle);
if (OBS_det_cw <= 0)
    OBS_cw_ccw = 1; % CW
else
    OBS_cw_ccw = 0; % CCW
end

% Field Boundary Point Squeeze.
FLD_Ysec = zeros(1,length(FLD_PT));
FLD_Yslp = zeros(1,length(FLD_PT));
for i = 1 : FLD_SIZE
    if abs(FLD_PT(i+1,1) - FLD_PT(i,1)) < 0.1
        FLD_PT(i+1,1) = FLD_PT(i+1,1) + 0.01;
    end
    FLD_Yslp(1,i) = (FLD_PT(i+1,2)-FLD_PT(i,2))/(FLD_PT(i+1,1)-FLD_PT(i,1));
    FLD_Ysec(1,i) = -FLD_Yslp(1,i)*FLD_PT(i,1)+FLD_PT(i,2);
    if (FLD_cw_ccw == 0)
        FLD_Ysec(1,i) = FLD_Ysec(1,i) + RANGE/(2*cos(atan2(FLD_PT(i+1,2)-FLD_PT(i,2),(FLD_PT(i+1,1)-FLD_PT(i,1)))));
    elseif (FLD_cw_ccw == 1)
        FLD_Ysec(1,i) = FLD_Ysec(1,i) - RANGE/(2*cos(atan2(FLD_PT(i+1,2)-FLD_PT(i,2),(FLD_PT(i+1,1)-FLD_PT(i,1)))));       
    end
end

FLD_Yslp(1,length(FLD_PT)) = FLD_Yslp(1,1);
FLD_Ysec(1,length(FLD_PT)) = FLD_Ysec(1,1);

for i = 1 : FLD_SIZE
    FLD_PT_new(i,1) = (FLD_Ysec(1,i)-FLD_Ysec(1,i+1))/(-FLD_Yslp(1,i)+FLD_Yslp(1,i+1));
    FLD_PT_new(i,2) = (FLD_Yslp(1,i+1)*FLD_Ysec(1,i)-FLD_Yslp(1,i)*FLD_Ysec(1,i+1))/(-FLD_Yslp(1,i)+FLD_Yslp(1,i+1));
end

FLD_PT_new(FLD_SIZE+1,:) = FLD_PT_new(1,:);


% Obstacle check.

% OBSTACLE CHECK :: Step 1. Extend Obs. Region to take a configuration space.
OBS_Ysec = zeros(1,length(OBS_PT));
OBS_Yslp = zeros(1,length(OBS_PT));
for i = 1 : OBS_SIZE
    if abs(OBS_PT(i+1,1) - OBS_PT(i,1)) < 0.1
        OBS_PT(i+1,1) = OBS_PT(i+1,1) + 0.01;
    end
    OBS_Yslp(1,i) = (OBS_PT(i+1,2)-OBS_PT(i,2))/(OBS_PT(i+1,1)-OBS_PT(i,1));
    OBS_Ysec(1,i) = -OBS_Yslp(1,i)*OBS_PT(i,1)+OBS_PT(i,2);
    if (OBS_cw_ccw == 0)
        OBS_Ysec(1,i) = OBS_Ysec(1,i) - RANGE/(2*cos(atan2(OBS_PT(i+1,2)-OBS_PT(i,2),(OBS_PT(i+1,1)-OBS_PT(i,1)))));
    elseif (OBS_cw_ccw == 1)
        OBS_Ysec(1,i) = OBS_Ysec(1,i) + RANGE/(2*cos(atan2(OBS_PT(i+1,2)-OBS_PT(i,2),(OBS_PT(i+1,1)-OBS_PT(i,1)))));       
    end
end

OBS_Yslp(1,length(OBS_PT)) = OBS_Yslp(1,1);
OBS_Ysec(1,length(OBS_PT)) = OBS_Ysec(1,1);

for i = 1 : OBS_SIZE
    OBS_PT_new(i,1) = (OBS_Ysec(1,i)-OBS_Ysec(1,i+1))/(-OBS_Yslp(1,i)+OBS_Yslp(1,i+1));
    OBS_PT_new(i,2) = (OBS_Yslp(1,i+1)*OBS_Ysec(1,i)-OBS_Yslp(1,i)*OBS_Ysec(1,i+1))/(-OBS_Yslp(1,i)+OBS_Yslp(1,i+1));
end

OBS_PT_new(OBS_SIZE+1,:) = OBS_PT_new(1,:);

% Proceed the main root.
WP(1,:) = FLD_PT_new(FLD_SIZE,:);
WP(2,:) = FLD_PT_new(1,:);
cnt = 3;
flag = 0;
while (1)
    if (FLD_cw_ccw == 0)
        FLD_Ysec(1,1) = FLD_Ysec(1,1) + RANGE/cos(atan2(FLD_PT(2,2)-FLD_PT(1,2),(FLD_PT(2,1)-FLD_PT(1,1))));
    elseif (FLD_cw_ccw == 1)
        FLD_Ysec(1,1) = FLD_Ysec(1,1) - RANGE/cos(atan2(FLD_PT(2,2)-FLD_PT(1,2),(FLD_PT(2,1)-FLD_PT(1,1))));
    end
    for j = 1 : FLD_SIZE-1
        tmp(1,1) = (FLD_Ysec(1,1)-FLD_Ysec(1,j+1))/(-FLD_Yslp(1,1)+FLD_Yslp(1,j+1));
        tmp(1,2) = (FLD_Yslp(1,j+1)*FLD_Ysec(1,1)-FLD_Yslp(1,1)*FLD_Ysec(1,j+1))/(-FLD_Yslp(1,1)+FLD_Yslp(1,j+1));
        if ((tmp(1,1) >= min(FLD_PT_new(j,1),FLD_PT_new(j+1,1))) && (tmp(1,1) <= max(FLD_PT_new(j,1),FLD_PT_new(j+1,1)))) && ((tmp(1,2) >= min(FLD_PT_new(j,2),FLD_PT_new(j+1,2))) && (tmp(1,2) <= max(FLD_PT_new(j,2),FLD_PT_new(j+1,2))))
            WP(cnt,:) = tmp(1,:);
            cnt = cnt + 1;
            flag = 1;
        end
    end
    if (flag == 0)
        break;
    end
    if (distance(WP(cnt-2,:),WP(cnt-3,:)) >= distance(WP(cnt-1,:),WP(cnt-3,:)))
        tmp(1,:) = WP(cnt-1,:);
        WP(cnt-1,:) = WP(cnt-2,:);
        WP(cnt-2,:) = tmp(1,:);
    end
    
    % OBSTACLE CHECK :: Step 2. check whether the obstacle is located on the
    % waypoint-line or not.
    % OBS_cntct : OBSTACLE contact point, 0, 1 ,2.
    OBS_cntct_cnt = 0;
    for j = 1 : OBS_SIZE
       tmp(1,1) = (FLD_Ysec(1,1)-OBS_Ysec(1,j+1))/(-FLD_Yslp(1,1)+OBS_Yslp(1,j+1));
       tmp(1,2) = (OBS_Yslp(1,j+1)*FLD_Ysec(1,1)-FLD_Yslp(1,1)*OBS_Ysec(1,j+1))/(-FLD_Yslp(1,1)+OBS_Yslp(1,j+1));
        if ((tmp(1,1) >= min(OBS_PT_new(j,1),OBS_PT_new(j+1,1))) && (tmp(1,1) <= max(OBS_PT_new(j,1),OBS_PT_new(j+1,1)))) && ((tmp(1,2) >= min(OBS_PT_new(j,2),OBS_PT_new(j+1,2))) && (tmp(1,2) <= max(OBS_PT_new(j,2),OBS_PT_new(j+1,2))))
            OBS_cntct_cnt = OBS_cntct_cnt + 1;
            OBS_cntct(OBS_cntct_cnt,[1,2]) = tmp(1,:);
            OBS_cntct(OBS_cntct_cnt,3) = j; % where this contacted point...
        end
    end    
    if (OBS_cntct_cnt == 2)
        if (distance(WP(cnt-2,:),OBS_cntct(1,[1,2])) >= distance(WP(cnt-2,:),OBS_cntct(2,[1,2])))
            temp(1,:) = OBS_cntct(2,:);
            OBS_cntct(2,:) = OBS_cntct(1,:);
            OBS_cntct(1,:) = temp(1,:);
        end        
        OBS_dst = zeros(1,2);
        pt = zeros(1,2);
        if (OBS_cntct(1,3) == OBS_SIZE)
            pt(1,1) = 1;
            pt(1,2) = OBS_cntct(1,3);
        else
            pt(1,1) = OBS_cntct(1,3)+1;
            pt(1,2) = OBS_cntct(1,3); 
        end
        OBS_dst(1,1) = distance(OBS_cntct(1,[1,2]),OBS_PT_new(pt(1,1),:));
        OBS_dst(1,2) = distance(OBS_cntct(1,[1,2]),OBS_PT_new(pt(1,2),:));
        
        while (pt(1,1) ~= OBS_cntct(2,3))
            if (pt(1,1) == OBS_SIZE)
                OBS_dst(1,1) = OBS_dst(1,1) + distance(OBS_PT_new(OBS_SIZE,:),OBS_PT_new(1,:));
                pt(1,1) = 1;
            else
                OBS_dst(1,1) = OBS_dst(1,1) + distance(OBS_PT_new(pt(1,1),:),OBS_PT_new(pt(1,1)+1,:));
                pt(1,1) = pt(1,1) + 1;                
            end
        end
        if (pt(1,1) == 1)
            OBS_dst(1,1) = OBS_dst(1,1) - distance(OBS_PT_new(pt(1,1),:),OBS_PT_new(OBS_SIZE,:)) + distance(OBS_PT_new(pt(1,1),:),OBS_cntct(2,[1,2]));
        else
            OBS_dst(1,1) = OBS_dst(1,1) - distance(OBS_PT_new(pt(1,1)-1,:),OBS_PT_new(pt(1,1),:)) + distance(OBS_PT_new(pt(1,1),:),OBS_cntct(2,[1,2]));
        end
        
        while (pt(1,2) ~= OBS_cntct(2,3))
            if (pt(1,2) == 1)
                OBS_dst(1,2) = OBS_dst(1,2) + distance(OBS_PT_new(OBS_SIZE,:),OBS_PT_new(1,:));
                pt(1,2) = OBS_SIZE;
            else
                OBS_dst(1,2) = OBS_dst(1,2) + distance(OBS_PT_new(pt(1,2),:),OBS_PT_new(pt(1,2)-1,:));
                pt(1,2) = pt(1,2) - 1;                
            end
        end
        if (pt(1,2) == OBS_SIZE)
            OBS_dst(1,2) = OBS_dst(1,2) - distance(OBS_PT_new(OBS_SIZE,:),OBS_PT_new(1,:)) + distance(OBS_PT_new(pt(1,2),:),OBS_cntct(2,[1,2]));
        else
            OBS_dst(1,2) = OBS_dst(1,2) - distance(OBS_PT_new(pt(1,2),:),OBS_PT_new(pt(1,2)+1,:)) + distance(OBS_PT_new(pt(1,2),:),OBS_cntct(2,[1,2]));
        end
        
        if (OBS_dst(1,1) < OBS_dst(1,2))
            tmp(1,:) = WP(cnt-1,:);
            WP(cnt-1,:) = OBS_cntct(1,[1,2]);
            cnt = cnt + 1;
            if (OBS_cntct(1,3) == OBS_SIZE)
                pt(1,1) = 1;
            else
                pt(1,1) = OBS_cntct(1,3)+1;
            end
            WP(cnt-1,:) = OBS_PT_new(pt(1,1),:);
            cnt = cnt + 1;
            while (pt(1,1) ~= OBS_cntct(2,3))
                if (pt(1,1) == OBS_SIZE)
                    pt(1,1) = 1;
                    WP(cnt-1,:) = OBS_PT_new(pt(1,1),:);
                    cnt = cnt + 1;
                else
                    pt(1,1) = pt(1,1) + 1;
                    WP(cnt-1,:) = OBS_PT_new(pt(1,1),:);
                    cnt = cnt + 1;
                end
            end
        else
            tmp(1,:) = WP(cnt-1,:);
            WP(cnt-1,:) = OBS_cntct(1,[1,2]);
            cnt = cnt + 1;
            pt(1,2) = OBS_cntct(1,3);
            WP(cnt-1,:) = OBS_PT_new(pt(1,2),:);
            cnt = cnt + 1;
            while (pt(1,2) ~= OBS_cntct(2,3))
                if (pt(1,2) == 1)
                    pt(1,2) = OBS_SIZE;
                    WP(cnt-1,:) = OBS_PT_new(pt(1,2),:);
                    cnt = cnt + 1;
                else
                    pt(1,2) = pt(1,2) - 1;
                    WP(cnt-1,:) = OBS_PT_new(pt(1,2),:);
                    cnt = cnt + 1;                    
                end
            end
        end
        if (OBS_dst(1,1) >= OBS_dst(1,2))
            WP(cnt-2,:) = OBS_cntct(2,[1,2]);
            WP(cnt-1,:) = tmp(1,:);
        else
            WP(cnt-1,:) = OBS_cntct(2,[1,2]);
            WP(cnt,:) = tmp(1,:);
            cnt = cnt + 1;
        end
    end
    flag = 0;
end

%%%% Plotting %%%%
patch(OBS_PT(:,1),OBS_PT(:,2),'white');
plot(FLD_PT(:,1),FLD_PT(:,2)); axis equal;
plot(WP(:,1),WP(:,2),'color','green');
plot(WP(1,1),WP(1,2),'b*');
%%%%%%%%%%%%%%%%%%
        