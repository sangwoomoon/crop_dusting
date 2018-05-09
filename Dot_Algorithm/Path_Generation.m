WP(1,1) = RES(1,1);
WP(1,2) = RES(1,2);
flag = 0;

for iter = 1 : 1000
    for n = 1 : length(RES)
        if ((RES(n,1) == WP(iter,1)) && (RES(n,2) == WP(iter,2)+range)) && (RES(n,3) == 0)
            WP(iter+1,1) = RES(n,1);
            WP(iter+1,2) = RES(n,2);
            flag = 1;
            RES(n,3) = 1;
            break;
        end
    end
    
    if (flag == 0)
        for n = 1 : length(RES)
            if ((RES(n,1) == WP(iter,1)+range) && (RES(n,2) == WP(iter,2))) && (RES(n,3) == 0)
               WP(iter+1,1) = RES(n,1);
                WP(iter+1,2) = RES(n,2);
                flag = 1;
                RES(n,3) = 1;
               break;
            end
        end
    end
    
    if (flag == 0)
        for n = 1 : length(RES)
            if ((RES(n,1) == WP(iter,1)) && (RES(n,2) == WP(iter,2)-range)) && (RES(n,3) == 0)
               WP(iter+1,1) = RES(n,1);
                WP(iter+1,2) = RES(n,2);
                flag = 1;
                RES(n,3) = 1;
               break;
            end
        end
    end        

    if (flag == 0)
        for n = 1 : length(RES)
            if ((RES(n,1) == WP(iter,1)-range) && (RES(n,2) == WP(iter,2))) && (RES(n,3) == 0)
               WP(iter+1,1) = RES(n,1);
                WP(iter+1,2) = RES(n,2);
                flag = 1;
                RES(n,3) = 1;
               break;
            end
        end
    end 
    
    if (flag == 0)
        break;
    end
end