function [tot_time] = estimate_time(test,v_t,acc)

time = 0;
s = 0.5*acc*(v_t/acc)^2;
for i = 1 : length(test)-1
 distance = sqrt((test(i+1,1)-test(i,1))^2+(test(i+1,2)-test(i,2))^2);
 if (distance >= 2*s)
     time = time + (distance - 2*s)/v_t+2*v_t/acc;
 else
     time = time + sqrt(distance/acc)*2;
 end
end

tot_time = time;
