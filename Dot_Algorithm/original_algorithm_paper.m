clear all;
clc;
clf;
range = 5;

% Boundary Point Marking

P = [-26.61,-39.91;-34.91,-35.23; -36.52,-25.29;-36.52,-14.18;-32.6,-3.655;-18.09,-2.778;-10.25,-5.117;-8.41,-16.23;-8.641,-31.43;-13.25,-37.28];

figure(1)
axis([0 300 0 300]);
grid on; axis equal; hold on;
 for i = 1 : 20
    [P(i,1), P(i,2)] = ginput(1);
    plot(P(i,1),P(i,2),'ro')
 end
a = length(P(:,1));
P(a+1,:) = P(1,:);
plot(P(:,1),P(:,2),'b-','linewidth',3);

figure(2)
patch(P(:,1),P(:,2),'green')
axis equal, grid on

for i = 5 : 8
    [L(i,1),L(i,2)] = ginput(1);
    plot(L(i,1),L(i,2),'bo')
end

for i = 1 : 6
    [OBS_2(i,1),OBS_2(i,2)] = ginput(1);
    plot(OBS_2(i,1),OBS_2(i,2),'bo');
end


plot(L([1,2],1),L([1,2],2),'Linewidth',3);
plot(L([3,4],1),L([3,4],2),'Linewidth',3);

for i = 4 : 5
    plot(L([i,i+1],1),L([i,i+1],2));
end


P(a+2,:) = P(2,:);


% obstacles formulation
patch(P(:,1),P(:,2),'green')
grid on; axis equal; hold on;
patch(OBS(:,1),OBS(:,2),'red')
patch(OBS_2(:,1),OBS_2(:,2),'blue')
plot(L([1,2],1),L([1,2],2),'Linewidth',3);
plot(L([3,4],1),L([3,4],2),'Linewidth',3);
plot(L([5,6],1),L([5,6],2),'Linewidth',3);
plot(L([7,8],1),L([7,8],2),'Linewidth',3);



%Obstacle Boundary Point Marking
% for i = 1 : 1
%     for j = 1: 4
%         [O(j,2*(i-1)+1), O(j,2*(i-1)+2)] = ginput(1);
%         plot(O(j,2*(i-1)+1),O(j,2*(i-1)+2), 'bo')
%     end
% end
% O(5,:) = O(1,:);

alpha = atan2((P(2,2)-P(1,2)),(P(2,1)-P(1,1)));

% Grid Making
for j = 1 : 60
    for i = 1 : 60
        A_TMP(i+(j-1)*60,1) = j;
        A_TMP(i+(j-1)*60,2) = i;
        A(i+(j-1)*60,1) = sqrt((j)^2+(i)^2)*cos(alpha+atan2(i,j));
        A(i+(j-1)*60,2) = sqrt((j)^2+(i)^2)*sin(alpha+atan2(i,j));
    end
end

A(:,1) = A(:,1) + 2.5;
A(:,2) = A(:,2) - 1.25;

% Graph Generation
figure(1)
grid on
hold on
axis equal
patch(P(:,1),P(:,2),'green');
plot(A(:,1),A(:,2),'r.');

% boundary Determinent Making
det = 0;
index = 1;
for i = 1 : length(A)
    for iter = 1 : length(P)-2
        angle = atan2(P(iter+1,2)-P(iter,2),P(iter+1,1)-P(iter,1));
        deter = [P(iter+2,1)-P(iter,1),P(iter+2,2)-P(iter,2)];
        det_cw = deter(1,1)*sin(-angle)+deter(1,2)*cos(-angle);
        if (det_cw <= 0)
            cw_ccw = 1;
        else
            cw_ccw = 0;
        end
        
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
        B(index,:) = A(i,:);
        B_TMP(index,:) = A_TMP(i,:);
        index = index + 1;
    end
    det = 0;
end

figure(2)
axis([-60 60 -60 60]);
grid on; axis equal;
hold on;
patch(P(1:11,1),P(1:11,2),'green');
plot(A_TMP(:,1),A_TMP(:,2),'r.');

figure(3)
axis([-50 50 -50 50]);
grid on; axis equal;
hold on;
patch(P(1:11,1),P(1:11,2),'green');
plot(A(:,1),A(:,2),'r.');

figure(4)
axis([-40 0 -45 0]);
grid on; axis equal;
hold on;
patch(P(1:11,1),P(1:11,2),'green');
plot(B(:,1),B(:,2),'r.');

% waypoints
WP = [
    -25.97, -37.15;
    -32.07, -33.72;
    -33.21, -29.63;
    -33.86, -24.67;
    -34.01, -18.84;
    -33.79, -14.38;
    -31.71, -8.658;
    -30.24, -6.045;
    -24.41, -5.888;
    -20.81, -5.62;
    -18.58, -5.732;
    -15.48, -6.334;
    -12.86, -7.808;
    -11.72, -11.89;
    -11.08, -16.85;
    -11.3, -21.32;
    -11.41,-23.55;
    -11.14, -27.15;
    -11.25, -29.38;
    -13.71, -33.74;
    -15.56, -34.99;
    -16.93, -35.37;
    -19.16, -35.26;
    -23.25, -36.39
    -30.21, -32.46;
    -32, -23.42;
    -32.05,-15.36;
    -29.97, -9.641;
    -28.99, -7.899;
    -21.8, -7.362;
    -19.56, -7.474;
    -16.46, -8.076;
    -14.72, -9.059;
    -13.58, -13.15;
    -12.93, -18.1;
    -13.26, -24.8;
    -13, -28.4;
    -15.45, -32.75;
    -16.82, -33.13;
    -19.05, -33.02;
    -23.14, -34.16;
    -28.36, -31.21;
    -29.77,-23.53;
    -29.81, -15.47;
    -27.74, -9.572;
    -20.55, -9.216;
    -16.57, -10.31;
    -15.43,-14.4;
    -15.28,-20.23;
    -15.5,-24.69;
    -15.23,-28.29;
    -16.7, -30.9;
    -18.94,-30.79;
    -23.02,-31.93;
    -26.51, -29.96;
    -27.54, -23.64;
    -27.58, -15.58;
    -26.49, -11.61;
    -20.66,-11.45;
    -18.42, -11.56;
    -17.67, -14.29;
    -17.4, -17.88;
    -17.62, -22.35;
    -17.46, -28.18;
    -17.95, -29.05;
    -20.19, -28.94;
    -22.91, -29.7;
    -24.65, -28.71;
    -25.3, -23.75;
    -25.35,-15.69;
    -24.36, -13.95;
    -20.77, -13.68;
    -19.9, -14.17;
    -19.63, -17.77;
    -19.85, -22.24;
    -20.08, -26.7;
    -22.8, -27.46;
    -22.69, -25.23;
    -22.73,-17.17];

WP_obs = [
    -25.97, -37.15;
    -32.07, -33.72;
    -33.21, -29.63;
    -33.86, -24.67;
    -34.01, -18.84;
    -33.79, -14.38;
    -31.71, -8.658;
    -30.24, -6.045;
    -24.41, -5.888;
    -20.81, -5.62;
    -18.58, -5.732;
    -15.48, -6.334;
    -12.86, -7.808;
    -11.72, -11.89;
    -11.08, -16.85;
    -11.3, -21.32;
    -11.41,-23.55;
    -11.14, -27.15;
    -11.25, -29.38;
    -13.71, -33.74;
    -15.56, -34.99;
    -16.93, -35.37;
    -19.16, -35.26;
    -23.25, -36.39
    -30.21, -32.46;
    -32, -23.42;
    -32.05,-15.36;
    -29.97, -9.641;
    -28.99, -7.899;
    -21.8, -7.362;
    -19.56, -7.474;
    -16.46, -8.076;
    -14.72, -9.059;
    -13.58, -13.15;
    -12.93, -18.1;
    -13.26, -24.8;
    -13, -28.4;
    -15.45, -32.75;
    -16.82, -33.13;
    -19.05, -33.02;
    -23.14, -34.16;
    -28.36, -31.21;
    -29.77,-23.53;
    -29.81, -15.47;
    -27.74, -9.572;
    -20.55, -9.216;
    -16.57, -10.31;
    -15.43,-14.4;
    -15.28,-20.23;
    -15.5,-24.69;
    -15.23,-28.29;
    -16.7, -30.9;
    -18.94,-30.79;
    -23.02,-31.93;
    -26.51, -29.96;
    -26.78, -26.37;
    -23.67, -26.97;
    -20.08, -26.7;
    -17.35, -25.94;
    -17.46, -28.18;
    -17.95, -29.05;
    -20.19, -28.94;
    -22.91, -29.7;
    -24.65, -28.71];    

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


