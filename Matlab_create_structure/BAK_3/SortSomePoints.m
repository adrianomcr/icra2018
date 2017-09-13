
w_s = [-3, 3, -2, 2];
clc
for k = 1:1:100
    x = (w_s(2)-w_s(1))*rand(1,1) + w_s(1);
    y = (w_s(4)-w_s(3))*rand(1,1) + w_s(3);
    fprintf('p%d = [%.3f, %.3f]\n',k,x,y)
end