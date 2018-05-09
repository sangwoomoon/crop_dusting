
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
