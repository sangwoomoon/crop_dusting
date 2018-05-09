
clc;
clear all;

a = 1000;
b = 800;
c = 20;  % covered range : half range.
theta_1 = -0.2;
theta_2 = -0.05;

% Transition Matrix
T = [1 tan(theta_1); tan(theta_2) 1];

A = [c,c;
    c, a-c;
    b-c, a-c;
    b-c, c];

i = 1;
while (1)
    A(4*i+1,1) = A(1,1)+2*i*c;
    A(4*i+1,2) = A(1,2)+2*(i-1)*c;
    if (distance(A(4*i,:),A(4*i+1,:)) < 2*c)
        A(4*i+2,1) = A(2,1)+2*i*c;
        A(4*i+2,2) = A(2,2)-2*i*c;        
        break;
    end
    A(4*i+2,1) = A(2,1)+2*i*c;
    A(4*i+2,2) = A(2,2)-2*i*c;
    if (distance(A(4*i+1,:),A(4*i+2,:)) < 2*c)
        A(4*i+3,1) = A(3,1)-2*i*c;
        A(4*i+3,2) = A(3,2)-2*i*c;
        break;
    end
    A(4*i+3,1) = A(3,1)-2*i*c;
    A(4*i+3,2) = A(3,2)-2*i*c;
    if (distance(A(4*i+2,:),A(4*i+3,:)) < 2*c)
        A(4*i+4,1) = A(4,1)-2*i*c;
        A(4*i+4,2) = A(4,2)+2*i*c;        
        break;
    end
    A(4*i+4,1) = A(4,1)-2*i*c;
    A(4*i+4,2) = A(4,2)+2*i*c;
    if (distance(A(4*i+3,:),A(4*i+4,:)) < 2*c)
        A(4*i+5,1) = A(1,1)+2*i*c;
        A(4*i+5,2) = A(1,2)+2*(i-1)*c;
        break;
    end
    i = i + 1;
end

    A(1,:) = T*A(1,:)';
for i = 2 : length(A)
    B(i,:) = T*A(i,:)';
    if (distance(A(i-1,:), A(i,:)) < 3*c)
        break;
    end
end

line(B(:,1), B(:,2)), axis equal, grid on


