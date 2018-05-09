function animation_heli(FLD_PT,OBS_PT,map,WP,heli_wp_x,heli_wp_y,heli_num,deter_movie_make,movie_name,frames_per_sec)

clf;
fig = figure(1);

%%%%%%%%%%%%%%%%%%%%%%%%  Plot Paths %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
image(map);
grid on, axis equal;
hold on;
if (OBS_PT ~= 0)
    patch(OBS_PT(:,1),OBS_PT(:,2),'white');
end
plot(FLD_PT(:,1),FLD_PT(:,2),'LineWidth',3); axis equal;
for i = 1 : heli_num
    switch rem(i,3)
        case (1)
            plot(heli_wp_x(:,i),heli_wp_y(:,i),':y');
        case (2)
            plot(heli_wp_x(:,i),heli_wp_y(:,i),':g');
        otherwise
            plot(heli_wp_x(:,3),heli_wp_y(:,3),':w');
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
set(fig,'DoubleBuffer','on');
% Helicopter scale.
scale = 0.7;

%%%%%% Draw helicopters :: head + rotor + boom + tail + (rod) %%%%%%%
heli_head=[7,0;-3,3;-3,-3;7,0]*scale;
heli_boom = [-3,0;-10,0]*scale;
heli_tail = [-10,0;-11,-2;-9,0;-11,2;-10,0]*scale;
heli_rod = [0,3.75;0,-3.75]; % fixed!
part_num = 40;
for idx=1:part_num
   x=6*cos(2*pi/part_num*idx)*scale;
   y=6*sin(2*pi/part_num*idx)*scale;
   heli_rotor(idx,1)=x;
   heli_rotor(idx,2)=y;
end
heli_rotor(part_num+1,:)=heli_rotor(1,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculate the heading angle of helicopters
theta = atan2(WP(2,2)-WP(1,2),WP(2,1)-WP(1,1));

%%%%%%%%%%%%%%%%%%%%%%%%% Heading Assign %%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1 : length(heli_head)
    tmp_head(1) = heli_head(i,1);
    tmp_head(2) = heli_head(i,2);
    heli_head(i,1) = tmp_head(1)*cos(theta)-tmp_head(2)*sin(theta);
    heli_head(i,2) = tmp_head(1)*sin(theta)+tmp_head(2)*cos(theta);
end

for i = 1 : length(heli_tail)
    tmp_tail(1) = heli_tail(i,1);
    tmp_tail(2) = heli_tail(i,2);
    heli_tail(i,1) = tmp_tail(1)*cos(theta)-tmp_tail(2)*sin(theta);
    heli_tail(i,2) = tmp_tail(1)*sin(theta)+tmp_tail(2)*cos(theta);
end

for i = 1 : 2
    tmp_boom(1) = heli_boom(i,1);
    tmp_boom(2) = heli_boom(i,2);
    heli_boom(i,1) = tmp_boom(1)*cos(theta)-tmp_boom(2)*sin(theta);
    heli_boom(i,2) = tmp_boom(1)*sin(theta)+tmp_boom(2)*cos(theta);
    tmp_rod(1) = heli_rod(i,1);
    tmp_rod(2) = heli_rod(i,2);
    heli_rod(i,1) = tmp_rod(1)*cos(theta)-tmp_rod(2)*sin(theta);
    heli_rod(i,2) = tmp_rod(1)*sin(theta)+tmp_rod(2)*cos(theta);    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%% Helicopter Assign %%%%%%%%%%%%%%%%%%%%%%%%%%%%
for idx = 1 : heli_num
    rod(idx) = line(heli_rod(:,1),heli_rod(:,2),'color','cyan','linewidth',2,'EraseMode','none');
    head(idx) = patch(heli_head(:,1),heli_head(:,2),rand(1,3));
    rotor(idx) = line(heli_rotor(:,1),heli_rotor(:,2),'color','black');
    boom(idx) = line(heli_boom(:,1),heli_boom(:,2),'color','black','linewidth',3);
    tail(idx) = patch(heli_tail(:,1),heli_tail(:,2),rand(1,3));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%% Makinng Movie Clip %%%%%%%%%%%%%%%%%%%%%%%%%%%
if (deter_movie_make == 1)
    numframes = length(heli_wp_x);
    aviobj = avifile(movie_name,'fps', frames_per_sec,'compression','Cinepak','Quality',100);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%% Play the Animation %%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1 : length(heli_wp_x)
    for idx = 1 : heli_num
        axis([min(heli_wp_x(i,:))-30,max(heli_wp_x(i,:))+30,min(heli_wp_y(i,:))-30,max(heli_wp_y(i,:))+30]);
        set(rod(idx),'Xdata',heli_rod(:,1)+heli_wp_x(i,idx),'YData',heli_rod(:,2)+heli_wp_y(i,idx),'EraseMode','none');    
        set(head(idx),'Xdata',heli_head(:,1)+heli_wp_x(i,idx),'YData',heli_head(:,2)+heli_wp_y(i,idx));
        set(rotor(idx),'Xdata',heli_rotor(:,1)+heli_wp_x(i,idx),'YData',heli_rotor(:,2)+heli_wp_y(i,idx));
        set(boom(idx),'Xdata',heli_boom(:,1)+heli_wp_x(i,idx),'YData',heli_boom(:,2)+heli_wp_y(i,idx));
        set(tail(idx),'Xdata',heli_tail(:,1)+heli_wp_x(i,idx),'YData',heli_tail(:,2)+heli_wp_y(i,idx));
        drawnow;
    end
    if (deter_movie_make == 1)
        frame = getframe(gcf);
        aviobj = addframe(aviobj, frame);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (deter_movie_make == 1)
    aviobj = close(aviobj);
end






