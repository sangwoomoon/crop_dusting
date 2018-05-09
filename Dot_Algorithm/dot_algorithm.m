
clear all;
clc;
clf;
range = 5;

% Graph Generation
axis([-50 50 -50 50]);
grid on
hold on

% Boundary Point Marking
 for i = 1 : 4
    [P(i,1), P(i,2)] = ginput(1);
    plot(P(i,1),P(i,2),'ro')
 end
%P = [46.1256, 194.3158; 216.6025, 43.5263; 213.7730, 150.8947; 129.5956, 203.7895];
P(5,:) = P(1,:);

%Obstacle Boundary Point Marking
for i = 1 : 1
    for j = 1: 4
        [O(j,2*(i-1)+1), O(j,2*(i-1)+2)] = ginput(1);
        plot(O(j,2*(i-1)+1),O(j,2*(i-1)+2), 'bo')
    end
end
O(5,:) = O(1,:);

alpha = atan2((P(2,2)-P(1,2)),(P(2,1)-P(1,1)));

angle = atan2(P(2,2)-P(1,2),P(2,1)-P(1,1));
deter = [P(3,1)-P(1,1),P(3,2)-P(1,2)];
det_cw = deter(1,1)*sin(-angle)+deter(1,2)*cos(-angle);
if (det_cw <= 0)
    cw_ccw = 1;
else
    cw_ccw = 0;
end

% Grid Making
for j = 1 : 495*4 / range
    for i = 1 : 495*4 / range
        A_TMP(i+(j-1)*132,1) = range/4*(j-66);
        A_TMP(i+(j-1)*132,2) = range/4*(i-66);
        A(i+(j-1)*132,1) = sqrt((range/4*(j-66))^2+(range/4*(i-66))^2)*cos(alpha+atan2(range/4*(i-66),range/4*(j-66)));
        A(i+(j-1)*132,2) = sqrt((range/4*(j-66))^2+(range/4*(i-66))^2)*sin(alpha+atan2(range/4*(i-66),range/4*(j-66)));
    end
end

% boundary Determinent Making
det = 0;
index = 1;
for i = 1 : length(A)
    for iter = 1 : length(P)-1
        theta(1,iter) = atan2((P(iter+1,2)-P(iter,2)),(P(iter+1,1)-P(iter,1)));
        det_bp_tmp(iter,1) = A(i,1)-P(iter,1);
        det_bp_tmp(iter,2) = A(i,2)-P(iter,2);
        det_bp_tmp(iter,2) = det_bp_tmp(iter,1)*sin(-theta(1,iter))+det_bp_tmp(iter,2)*cos(-theta(1,iter));
        if (cw_ccw == 1) && (det_bp_tmp(iter,2) > -range/2) 
            det = 1;
            break;
        elseif (cw_ccw == 0) && (det_bp_tmp(iter,2) < range/2)
            det = 1;
            break;
        end
    end
    if (det == 0)
         for ii = 1 : length(P)-1
             if (cw_ccw == 1) && (det_bp_tmp(ii,2) > -range*3/4)           
                B(index,:) = A(i,:);
                B_TMP(index,:) = A_TMP(i,:);
                index = index + 1;
                break;
            elseif (cw_ccw == 0) && (det_bp_tmp(ii,2) < range*3/4)
                B(index,:) = A(i,:);
                B_TMP(index,:) = A_TMP(i,:);
                index = index + 1;
                break;
             end
         end
    end
    det = 0;
end

% Obstacle check.

% Step 1. Extend Obs. Region to take a configuration space.
y_sec = zeros(1,length(O));
y_slp = zeros(1,length(O));
for i = 1 : length(O)-1
    if (O(i+1,2) == O(i,2))
        O(i+1,2) = O(i+1,2) + 0.001;
    end
    y_slp(1,i) = (O(i+1,2)-O(i,2))/(O(i+1,1)-O(i,1));
    y_sec(1,i) = -y_slp(1,i)*O(i,1)+O(i,2);
    if (cw_ccw == 0)
        y_sec(1,i) = y_sec(1,i) - range/(2*cos(atan2(O(i+1,2)-O(i,2),(O(i+1,1)-O(i,1)))));
    elseif (cw_ccw == 1)
        y_sec(1,i) = y_sec(1,i) + range/(2*cos(atan2(O(i+1,2)-O(i,2),(O(i+1,1)-O(i,1)))));       
    end
end

y_slp(1,length(O)) = y_slp(1,1);
y_sec(1,length(O)) = y_sec(1,1);

for i = 1 : length(O)-1
    O_new(i,1) = (y_sec(1,i)-y_sec(1,i+1))/(-y_slp(1,i)+y_slp(1,i+1));
    O_new(i,2) = (y_slp(1,i+1)*y_sec(1,i)-y_slp(1,i)*y_sec(1,i+1))/(-y_slp(1,i)+y_slp(1,i+1));
end

O_new(5,:) = O_new(1,:);

% obstacle Determinent Making
% det = 0;
% index = 1;
% for i = 1 : length(A)
%     for iter = 1 : length(O)-1
%         theta(1,iter) = atan2((O(iter+1,2)-O(iter,2)),(O(iter+1,1)-O(iter,1)));
%         det_bp_tmp(iter,1) = A(i,1)-O(iter,1);
%         det_bp_tmp(iter,2) = A(i,2)-O(iter,2);
%         det_bp_tmp(iter,2) = det_bp_tmp(iter,1)*sin(-theta(1,iter))+det_bp_tmp(iter,2)*cos(-theta(1,iter));
%         if (cw_ccw == 1) && (det_bp_tmp(iter,2) > range*3/4) 
%             det = 1;
%             break;
%         elseif (cw_ccw == 0) && (det_bp_tmp(iter,2) < -range*3/4)
%             det = 1;
%             break;
%         end
%     end
%     if (det == 0)
%          for ii = 1 : length(P)-1
%              if (cw_ccw == 1) && (det_bp_tmp(ii,2) > range/2)           
%                 C(index,:) = A(i,:);
%                 C_TMP(index,:) = A_TMP(i,:);
%                 index = index + 1;
%                 break;
%             elseif (cw_ccw == 0) && (det_bp_tmp(ii,2) < -range/2)
%                 C(index,:) = A(i,:);
%                 C_TMP(index,:) = A_TMP(i,:);
%                 index = index + 1;
%                 break;
%              end
%          end
%     end
%     det = 0;
% end

% Boundary Determinent
% index = 1;
% if (cw_ccw == 1)
% for i = 1 : length(A)
%     det = 0;
%     for iter = 1 : length(P)-1
%         if(C(i,(iter-1)*2+2) > 0) %&& (C(i, (iter-1)*2+2) > -range*2)
%             det = 1;
%         end
%     end
%     if (det == 0)
%         B(index,:) = A(i,:);
%         B_TMP(index,:) = A_TMP(i,:);
%         index = index + 1;
%     end
% end
% elseif cw_ccw == 0
% for i = 1 : length(A)
%     det = 0;
%     for iter = 1 : length(P)-1
%         if(C(i,(iter-1)*2+2) < 0) %&& (C(i, (iter-1)*2+2) < range*2)
%             det = 1;
%         end
%     end
%     if (det == 0)
%         B(index,:) = A(i,:);
%         B_TMP(index,:) = A_TMP(i,:);
%         index = index + 1;
%     end
% end
% end


% Obstacle Determinent Making
% for iter = 1 : length(O)-1
%     for i = 1 : length(B)
%         theta(1,iter) = atan2((O(iter+1,2)-O(iter,2)),(O(iter+1,1)-O(iter,1)));
%         O_1(i,(iter-1)*2+1) = B(i,1)-O(iter,1);
%         O_1(i,(iter-1)*2+2) = B(i,2)-O(iter,2);
%         O_1(i,(iter-1)*2+2) = O_1(i,(iter-1)*2+1)*sin(-theta(1,iter))+O_1(i,(iter-1)*2+2)*cos(-theta(1,iter));
%     end
% end

RES = B;
RES_TMP = B_TMP;

% Obstacle Determinent
% index = 1;
% for i = 1 : length(B)
%     det = 0;
%     for iter = 1 : length(O)-1
%         if(O_1(i,(iter-1)*2+2) <= -range/4)
%             det = 1;
%         end
%     end
%     if (det == 1)
%         RES(index,:) = B(i,:);
%         RES_TMP(index,:) = B_TMP(i,:);
%         index = index + 1;
%     end
% end

% Result Plot
plot(RES(:,1),RES(:,2),'g.')
plot(C(:,1),C(:,2),'k.')
line(P(:,1),P(:,2),'LineWidth',2)
%patch(O(:,1),O(:,2),'g')

% Path Generation
RES_TMP_max_y = max(RES_TMP(:,2));
RES_TMP_min_y = min(RES_TMP(:,2));
flag = 1;
num = 1;
det = 0;

while (1)
    if (flag == 1)
        if (cw_ccw == 0)
            Y_REF = RES_TMP_min_y;
        elseif (cw_ccw == 1)
            Y_REF = RES_TMP_max_y;
        end
    end
    
    if (flag > 1)
        if (cw_ccw == 0) % CCW
            Y_REF = Y_REF + range;
        elseif (cw_ccw == 1) % CW
            Y_REF = Y_REF - range;
        end
    end
    
    index = 1;
    for i = 1 : length(RES_TMP)
        if (RES_TMP(i,2) == Y_REF)
            WP_TMP(index,1) = RES_TMP(i,1);
            WP_TMP(index,2) = RES_TMP(i,2);
            DATA(index,1) = RES(i,1);
            DATA(index,2) = RES(i,2);
            index = index + 1;
            det = 1;
        end
    end

    if (det == 0)
        break;
    end
    
    if (rem(flag,2) == 1) %odd. use '%'.
        for i = 1 : length(DATA(:,1))-1
            m = i;
            for j = i+1 : length(DATA(:,1))
                if (DATA(i,2) > DATA(j,2))
                    m = j;
                end
            end
            temp = DATA(i,:);
            DATA(i,:) = DATA(m,:);
            DATA(m,:) = temp;
        end
        if (alpha >= 0 )
            WP(num,:) = DATA(1,:);
            num = num + 1;
            WP(num,:) = DATA(length(DATA(:,1)),:);
            num = num + 1;
        else
            WP(num,:) = DATA(length(DATA(:,1)),:);
            num = num + 1;
            WP(num,:) = DATA(1,:);
            num = num + 1;
        end
    else
        for i = 1 : length(DATA(:,1))-1
            m = i;
            for j = i+1 : length(DATA(:,1))
                if (DATA(i,2) < DATA(j,2))
                    m = j;
                end
            end
            temp = DATA(i,:);
            DATA(i,:) = DATA(m,:);
            DATA(m,:) = temp;
        end
        if (alpha >= 0 )
            WP(num,:) = DATA(1,:);
            num = num + 1;
            WP(num,:) = DATA(length(DATA(:,1)),:);
            num = num + 1;
        else
            WP(num,:) = DATA(length(DATA(:,1)),:);
            num = num + 1;
            WP(num,:) = DATA(1,:);
            num = num + 1;
        end
    end
    flag = flag + 1;
    
    clear DATA;
    clear WP_TMP;
    det = 0;
end

plot(WP(1,1), WP(1,2),'b*'), axis equal
line(WP(:,1), WP(:,2)), axis equal
hold off


