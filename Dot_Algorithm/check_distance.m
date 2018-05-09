        if (distance(WP(cnt-1,:),OBS_cntct(1,:)) >= distance(WP(cnt-1,:),OBS_cntct(2,:)))
            tmp(1,:) = OBS_cntct(2,:);
            OBS_cntct(2,:) = OBS_cntct(1,:);
            OBS_cntct(1,:) = tmp(1,:);
        end
        for i = 1 : SIZE
            if ((OBS_cntct(1,1) >= min(OBS_PT_new(i,1),OBS_PT_new(i+1,1))) && (OBS_cntct(1,1) <= max(OBS_PT_new(i,1),OBS_PT_new(i+1,1)))) && ((OBS_cntct(1,2) >= min(OBS_PT_new(i,2),OBS_PT_new(i+1,2))) && (OBS_cntct(1,2) <= max(OBS_PT_new(i,2),OBS_PT_new(i+1,2))))
                break;
            end
        end
                
        flag = 0;

        OBS_flag = 0;
        OBS_dst = zeros(1,2);
        k = zeros(1,2);
        if (i == 1)
            OBS_dst(1,1) = distance(OBS_cntct(1,:),OBS_PT_new(SIZE,:));
            OBS_dst(1,2) = distance(OBS_cntct(1,:),OBS_PT_new(i,:));
            k(1,1) = SIZE;
            k(1,2) = i;
            plot(OBS_PT_new(SIZE,1),OBS_PT_new(SIZE,2),'ko');
        else
            OBS_dst(1,1) = distance(OBS_cntct(1,:),OBS_PT_new(i-1,:));
            OBS_dst(1,2) = distance(OBS_cntct(1,:),OBS_PT_new(i,:));
            k(1,1) = i-1;
            k(1,2) = i;
            plot(OBS_cntct(1,1),OBS_cntct(1,2),'ko');   
            plot(OBS_PT_new(i-1,1),OBS_PT_new(i-1,2),'ko');   
        end
        while(1)
            if (k(1,1) == 1)
                OBS_dst(1,1) = OBS_dst(1,1) + distance(OBS_PT_new(k(1,1),:),OBS_PT_new(SIZE,:));
                plot(OBS_PT_new(SIZE,1),OBS_PT_new(SIZE,2),'ko');   
                k(1,1) = SIZE;
                if ((OBS_cntct(2,1) >= min(OBS_PT_new(k(1,1),1),OBS_PT_new(k(1,1)+1,1))) && (OBS_cntct(2,1) <= max(OBS_PT_new(k(1,1),1),OBS_PT_new(k(1,1)+1,1)))) && ((OBS_cntct(2,2) >= min(OBS_PT_new(k(1,1),2),OBS_PT_new(k(1,1)+1,2))) && (OBS_cntct(2,2) <= max(OBS_PT_new(k(1,1),2),OBS_PT_new(k(1,1)+1,2))))
                    OBS_dst(1,1) = OBS_dst(1,1) - distance(OBS_PT_new(k(1,1),:),OBS_PT_new(1,:)) + distance(OBS_cntct(2,:),OBS_PT_new(1,:));   
                    break;
                end
            else
                OBS_dst(1,1) = OBS_dst(1,1) + distance(OBS_PT_new(k(1,1),:),OBS_PT_new(k(1,1)-1,:));
                plot(OBS_PT_new(k(1,1)-1,1),OBS_PT_new(k(1,1)-1,2),'ko');   
                k(1,1) = k(1,1) - 1;          
                if ((OBS_cntct(2,1) >= min(OBS_PT_new(k(1,1),1),OBS_PT_new(k(1,1)+1,1))) && (OBS_cntct(2,1) <= max(OBS_PT_new(k(1,1),1),OBS_PT_new(k(1,1)+1,1)))) && ((OBS_cntct(2,2) >= min(OBS_PT_new(k(1,1),2),OBS_PT_new(k(1,1)+1,2))) && (OBS_cntct(2,2) <= max(OBS_PT_new(k(1,1),2),OBS_PT_new(k(1,1)+1,2))))
                    OBS_dst(1,1) = OBS_dst(1,1) - distance(OBS_PT_new(k(1,1),:),OBS_PT_new(k(1,1)+1,:)) + distance(OBS_cntct(2,:),OBS_PT_new(k(1,1),:));
                    break;
                end
            end
        end
        
        while(1)
            if (k(1,2) == SIZE)
                OBS_dst(1,2) = OBS_dst(1,2) + distance(OBS_PT_new(k(1,2),:),OBS_PT_new(1,:));
                k(1,2) = 1;
                if ((OBS_cntct(2,1) >= min(OBS_PT_new(k(1,2),1),OBS_PT_new(SIZE,1))) && (OBS_cntct(2,1) <= max(OBS_PT_new(k(1,2),1),OBS_PT_new(SIZE,1)))) && ((OBS_cntct(2,2) >= min(OBS_PT_new(k(1,2),2),OBS_PT_new(SIZE,2))) && (OBS_cntct(2,2) <= max(OBS_PT_new(k(1,2),2),OBS_PT_new(SIZE,2))))
                    OBS_dst(1,1) = OBS_dst(1,1) - distance(OBS_PT_new(k(1,2),:),OBS_PT_new(SIZE,:)) + distance(OBS_cntct(2,:),OBS_PT_new(SIZE,:));
                    break;
                end
            else
                OBS_dst(1,2) = OBS_dst(1,2) + distance(OBS_PT_new(k(1,2),:),OBS_PT_new(k(1,2)+1,:));
                k(1,2) = k(1,2) + 1;
                if ((OBS_cntct(2,1) >= min(OBS_PT_new(k(1,2),1),OBS_PT_new(k(1,2)-1,1))) && (OBS_cntct(2,1) <= max(OBS_PT_new(k(1,2),1),OBS_PT_new(k(1,2)-1,1)))) && ((OBS_cntct(2,2) >= min(OBS_PT_new(k(1,2),2),OBS_PT_new(k(1,2)-1,2))) && (OBS_cntct(2,2) <= max(OBS_PT_new(k(1,2),2),OBS_PT_new(k(1,2)-1,2))))
                    OBS_dst(1,1) = OBS_dst(1,1) - distance(OBS_PT_new(k(1,2)-1,:),OBS_PT_new(k(1,2),:)) + distance(OBS_cntct(2,:),OBS_PT_new(k(1,2),:));
                    break;
                end                
            end
        end