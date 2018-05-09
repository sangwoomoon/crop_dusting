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
for j = 1 : 100*4 / range
    for i = 1 : 100*4 / range
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
det = 0;
index = 1;
for i = 1 : length(A)
    for iter = 1 : length(O)-1
        theta(1,iter) = atan2((O(iter+1,2)-O(iter,2)),(O(iter+1,1)-O(iter,1)));
        det_bp_tmp(iter,1) = A(i,1)-O(iter,1);
        det_bp_tmp(iter,2) = A(i,2)-O(iter,2);
        det_bp_tmp(iter,2) = det_bp_tmp(iter,1)*sin(-theta(1,iter))+det_bp_tmp(iter,2)*cos(-theta(1,iter));
        if (cw_ccw == 1) && (det_bp_tmp(iter,2) > range*3/4) 
            det = 1;
            break;
        elseif (cw_ccw == 0) && (det_bp_tmp(iter,2) < -range*3/4)
            det = 1;
            break;
        end
    end
    if (det == 0)
         for ii = 1 : length(P)-1
             if (cw_ccw == 1) && (det_bp_tmp(ii,2) > range/2)           
                C(index,:) = A(i,:);
                C_TMP(index,:) = A_TMP(i,:);
                index = index + 1;
                break;
            elseif (cw_ccw == 0) && (det_bp_tmp(ii,2) < -range/2)
                C(index,:) = A(i,:);
                C_TMP(index,:) = A_TMP(i,:);
                index = index + 1;
                break;
             end
         end
    end
    det = 0;
end

% Boundary Determinent
index = 1;
if (cw_ccw == 1)
for i = 1 : length(A)
    det = 0;
    for iter = 1 : length(P)-1
        if(C(i,(iter-1)*2+2) > 0) %&& (C(i, (iter-1)*2+2) > -range*2)
            det = 1;
        end
    end
    if (det == 0)
        B(index,:) = A(i,:);
        B_TMP(index,:) = A_TMP(i,:);
        index = index + 1;
    end
end
elseif cw_ccw == 0
for i = 1 : length(A)
    det = 0;
    for iter = 1 : length(P)-1
        if(C(i,(iter-1)*2+2) < 0) %&& (C(i, (iter-1)*2+2) < range*2)
            det = 1;
        end
    end
    if (det == 0)
        B(index,:) = A(i,:);
        B_TMP(index,:) = A_TMP(i,:);
        index = index + 1;
    end
end
end


% Obstacle Determinent Making
for iter = 1 : length(O)-1
    for i = 1 : length(B)
        theta(1,iter) = atan2((O(iter+1,2)-O(iter,2)),(O(iter+1,1)-O(iter,1)));
        O_1(i,(iter-1)*2+1) = B(i,1)-O(iter,1);
        O_1(i,(iter-1)*2+2) = B(i,2)-O(iter,2);
        O_1(i,(iter-1)*2+2) = O_1(i,(iter-1)*2+1)*sin(-theta(1,iter))+O_1(i,(iter-1)*2+2)*cos(-theta(1,iter));
    end
end

RES = B;
RES_TMP = B_TMP;

% Obstacle Determinent
index = 1;
for i = 1 : length(B)
    det = 0;
    for iter = 1 : length(O)-1
        if(O_1(i,(iter-1)*2+2) <= -range/4)
            det = 1;
        end
    end
    if (det == 1)
        RES(index,:) = B(i,:);
        RES_TMP(index,:) = B_TMP(i,:);
        index = index + 1;
    end
end

% Result Plot
plot(RES(:,1),RES(:,2),'g.')
plot(C(:,1),C(:,2),'k.')
line(P(:,1),P(:,2),'LineWidth',2)
%patch(O(:,1),O(:,2),'g')

% Path Generation
WP(1,1) = RES(1,1);
WP(1,2) = RES(1,2);

for iter = 1 : length(RES)-1

    theta = 0;
    if (iter >= 2)
        theta = atan2(WP(iter,2)-WP(iter-1,2), WP(iter,1)-WP(iter-1,1));
        for i = 1 : length(RES)
            RES_1(i,1) = (RES(i,1)-WP(iter,1))*cos(-theta)-(RES(i,2)-WP(iter,2))*sin(-theta);
            RES_1(i,2) = (RES(i,1)-WP(iter,1))*sin(-theta)+(RES(i,2)-WP(iter,2))*cos(-theta);
        end
    else
        RES_1(:,1) = RES(:,1) - WP(1,1);
        RES_1(:,2) = RES(:,2) - WP(1,2);
    end
    
    for i = 1 : length(RES)
        if (RES(i,3) == 1)
            angle(i,1) = -100;  % make an impossible situation.
        else
            angle(i,1) = atan2(RES_1(i,2), RES_1(i,1));
        end
    end
    angle_max = max(angle);

    % BUGS ARE DISCOVERED !!! => Matlab bug : not ==, but <= .
    
    ii = 1;
    for i = 1 : length(RES)
        if abs(angle_max - angle(i,1)) <= 0.00001
            WP_TMP(ii,[1:2]) = RES(i,[1:2]);
            ii = ii + 1;
        end
    end

    for i = 1 : length(WP_TMP(:,1))          % sizeof(array) / (sizeof(member)*3)
        DIST_WP(i,1) = sqrt((WP(iter,1)-WP_TMP(i,1))^2 + (WP(iter,2)-WP_TMP(i,2))^2);
    end

    dist_min = min(DIST_WP);
    for i = 1 : length(WP_TMP(:,1))          % sizeof(array) / (sizeof(member)*3)
        if (DIST_WP(i,1) == dist_min)
            WP(iter+1,:) = WP_TMP(i,[1:2]);
        end
    end
    
    for i = 1 : length(RES)
        if (RES(i,1) == WP(iter+1,1)) && (RES(i,2) == WP(iter+1,2))
            RES(i,3) = 1;
        end
    end
    
    clear WP_TMP DIST_WP;
end

line(WP(:,1), WP(:,2))
hold off


