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