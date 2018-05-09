
test = load('test.dat');
patch(P(:,1),P(:,2),'green'), grid on, hold on, axis equal
plot(test(:,1),test(:,2))
plot(test(1,1),test(1,2),'ro')