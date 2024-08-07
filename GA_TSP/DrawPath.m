function DrawPath(route, City)
%% 画路径函数
% 输入
% route     待画路径   
% City      各城市坐标位置

figure
hold on % 保留当前坐标区中的绘图，从而使新添加到坐标区中的绘图不会删除现有绘图
box on % 通过将当前坐标区的 Box 属性设置为 'on' 在坐标区周围显示框轮廓
xlim([min(City(:,1))-0.01, max(City(:,1))+0.01]) % 手动设置x轴范围
ylim([min(City(:,2))-0.01, max(City(:,2))+0.01]) % 手动设置y轴范围
zlim([min(City(:,3))-0.01, max(City(:,3))+0.01]) % 手动设置z轴范围
axis equal;

% 画配送中心点
plot3(City(1,1), City(1,2), City(1,3), 'bp', 'MarkerFaceColor', 'r', 'MarkerSize', 15) 

% 画需求点
plot3(City(2:end,1), City(2:end,2), City(2:end,3), 'o', 'color', [0.5, 0.5, 0.5], 'MarkerFaceColor', 'g') 

% 添加点编号
for i = 1:size(City,1)
    text(City(i,1)+0.002, City(i,2)-0.002, City(i,3)-0.002, num2str(i-1)); % 为点进行编号
end

 % 使XYZ轴的刻度比例一致

% 画箭头
A = City(route+1,:);
arrcolor = rand(1,3);

for i = 2:length(A)
    % 计算箭头的方向和长度
    dir = A(i,:) - A(i-1,:);
    quiver3(A(i-1,1), A(i-1,2), A(i-1,3), dir(1), dir(2), dir(3), 0, 'Color', arrcolor, 'LineWidth', 2, 'MaxHeadSize', 0.5);
end

xlabel('X Coordinate')
ylabel('Y Coordinate')
zlabel('Z Coordinate')
title('Route Map')

hold off % 将保留状态设置为 off，从而使新添加到坐标区中的绘图清除现有绘图并重置所有的坐标区属性
end