clear all;
clc;
clf;

axis([-100 100 -100 100]);
grid on
hold on

 for i = 1 : 4
    [P(i,1), P(i,2)] = ginput(1);
    plot(P(i,1),P(i,2),'ro')
 end
 
 P(5,:) = P(1,:);
 
theta = atan2(P(2,2)-P(1,2),P(2,1)-P(1,1));
det = [P(3,1)-P(1,1),P(3,2)-P(1,2)];

C = det(1,1)*sin(-theta)+det(1,2)*cos(-theta);

if (C <= 0)
    res = 1;
else
    res = 0;
end

hold off