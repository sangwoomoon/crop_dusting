function [pts] = set_points(WP, v_t, v_acc, del_t)

clear pts;
s = 0.5*v_acc*(v_t/v_acc)^2;
cnt = 0;
tmp = zeros(1,2);

for i = 1 : length(WP)-1
    theta = atan2(WP(i+1,2)-WP(i,2),WP(i+1,1)-WP(i,1));
    if (distance(WP(i+1,:),WP(i,:)) < 2*s)
        cnt=cnt+1;
        tmp(cnt,:) = WP(i,:);
        t = del_t;
        while (distance(tmp(cnt,:),WP(i+1,:))>distance(WP(i+1,:),WP(i,:))/2)
            cnt=cnt+1;
            tmp(cnt,1) = WP(i,1)+0.5*v_acc*t^2*cos(theta);
            tmp(cnt,2) = WP(i,2)+0.5*v_acc*t^2*sin(theta);
            t = t+del_t;
        end
        cnt = cnt-1;
        half_time = t;
        half_vel = v_acc*half_time;
        pos = zeros(1,2);
        pos(1,1) = tmp(cnt,1);
        pos(1,2) = tmp(cnt,2);
        while (half_vel-v_acc*(t-half_time)>0)
            cnt=cnt+1;
            tmp(cnt,1) = pos(1,1)+(half_vel*(t-half_time)-0.5*v_acc*(t-half_time)^2)*cos(theta);
            tmp(cnt,2) = pos(1,2)+(half_vel*(t-half_time)-0.5*v_acc*(t-half_time)^2)*sin(theta);
            t = t+del_t;
        end
    else
        cnt=cnt+1;
        tmp(cnt,:) = WP(i,:);
        t = del_t;
        while (v_acc*t < v_t)
            cnt=cnt+1;
            tmp(cnt,1) = WP(i,1)+0.5*v_acc*t^2*cos(theta);
            tmp(cnt,2) = WP(i,2)+0.5*v_acc*t^2*sin(theta);
            t = t+del_t;
        end
        cnt = cnt-1;        
        mid_time = t;
        pos(1,1) = tmp(cnt,1);
        pos(1,2) = tmp(cnt,2);
        while (distance(tmp(cnt,:),WP(i+1,:))>s)
            cnt=cnt+1;
            tmp(cnt,1) = pos(1,1)+v_t*(t-mid_time)*cos(theta);
            tmp(cnt,2) = pos(1,2)+v_t*(t-mid_time)*sin(theta);
            t = t+del_t;
        end
        cnt = cnt-1;        
        mid_time = t;
        pos(1,1) = tmp(cnt,1);
        pos(1,2) = tmp(cnt,2);
        while (v_t-v_acc*(t-mid_time) > 0 )
            cnt=cnt+1;
            tmp(cnt,1) = pos(1,1)+(v_t*(t-mid_time)-0.5*v_acc*(t-mid_time)^2)*cos(theta);
            tmp(cnt,2) = pos(1,2)+(v_t*(t-mid_time)-0.5*v_acc*(t-mid_time)^2)*sin(theta);
            t = t+del_t;       
        end
    end
end
       
pts = tmp;
%plot(pts(:,1),pts(:,2),'k.');
%axis equal;


function [dist] = distance(a,b)

dist = sqrt((a(1,1)-b(1,1))^2 + (a(1,2)-b(1,2))^2);
    