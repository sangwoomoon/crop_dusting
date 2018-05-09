
min_dist;

for i = 1 : length(heli_wp_x)
    for idx_i = 1 : heli_num-1
        for idx_j = idx_i+1 : heli_num
            if (distance(heli_wp_x(i,idx_i),heli_wp_x(i,idx_j),heli_wp_y(i,idx_i),heli_wp_y(i,idx_j)) < min_dist)
                wp_x(i,idx_i) = wp_x(i-1,idx_i);
                wp_y(i,idx_i) = wp_y(i-1,idx_i);
                
                
              