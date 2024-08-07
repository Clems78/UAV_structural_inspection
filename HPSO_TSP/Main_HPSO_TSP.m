% clc %清空命令行窗口
% clear %从当前工作区中删除所有变量，并将它们从系统内存中释放
close all %删除其句柄未隐藏的所有图窗
tic % 保存当前时间
%% GA-PSO算法求解TSP
%输入：
%City           需求点经纬度
%Distance       距离矩阵
%NIND           种群个数
%MAXGEN         遗传到第MAXGEN代时程序停止

%输出：
%Gbest          最短路径
%GbestDistance	最短路径长度
%% 加载数据
% load('../test_data/City.mat')	      %需求点经纬度，用于画实际路径的XY坐标
% load('../test_data/Distance.mat')	  %距离矩阵

City = waypoints;
Distance = pdist(waypoints, 'euclidean');
Distance = squareform(Distance);
%% 初始化问题参数
CityNum=size(City,1)-1; %需求点个数

%% 初始化算法参数
NIND=1000; %A larger population size can explore the search space more thoroughly.
MAXGEN=2000; %More iterations allow the algorithm to refine solutions further.

%% 为预分配内存而初始化的0矩阵
Population = zeros(NIND,CityNum+2); %预分配内存，用于存储种群数据
PopDistance = zeros(NIND,1); %预分配矩阵内存
GbestDisByGen = zeros(MAXGEN,1); %预分配矩阵内存 用于绘制迭代图

for i = 1:NIND
    %% 初始化各粒子
    Population(i,2:end-1)=InitPop(CityNum); %随机生成TSP路径
    
    %% 计算路径长度
    PopDistance(i) = CalcDis(Population(i,:),Distance); % 计算路径长度
end

%% 存储Pbest数据
Pbest = Population; % 初始化Pbest为当前粒子集合
PbestDistance = PopDistance; % 初始化Pbest的目标函数值为当前粒子集合的目标函数值

%% 存储Gbest数据
[mindis,index] = min(PbestDistance); %获得Pbest中最小值
Gbest = Pbest(index,:); % 初始Gbest粒子
GbestDistance = mindis; % 初始Gbest粒子的目标函数值

%% 开始迭代
gen=1;

while gen <= MAXGEN
    %% 每个粒子更新
    for i=1:NIND
        %% 粒子与Pbest交叉
        Population(i,2:end-1)=Crossover(Population(i,2:end-1),Pbest(i,2:end-1)); %交叉
        
        % 新路径长度变短则记录至Pbest
        PopDistance(i) = CalcDis(Population(i,:),Distance); %计算距离
        if PopDistance(i) < PbestDistance(i) %若新路径长度变短
            Pbest(i,:)=Population(i,:); %更新Pbest
            PbestDistance(i)=PopDistance(i); %更新Pbest距离
        end
        
        %% 根据Pbest更新Gbest
        [mindis,index] = min(PbestDistance); %找出Pbest中最短距离

        if mindis < GbestDistance %若Pbest中最短距离小于Gbest距离
            Gbest = Pbest(index,:); %更新Gbest
            GbestDistance = mindis; %更新Gbest距离
        end
        
        %% 粒子与Gbest交叉
        Population(i,2:end-1)=Crossover(Population(i,2:end-1),Gbest(2:end-1));
        
        % 新路径长度变短则记录至Pbest
        PopDistance(i) = CalcDis(Population(i,:),Distance); %计算距离
        if PopDistance(i) < PbestDistance(i) %若新路径长度变短
            Pbest(i,:)=Population(i,:); %更新Pbest
            PbestDistance(i)=PopDistance(i); %更新Pbest距离
        end
        
        %% 粒子自身变异
        Population(i,:)=Mutate(Population(i,:));

        % 新路径长度变短则记录至Pbest
        PopDistance(i) = CalcDis(Population(i,:),Distance); %计算距离
        if PopDistance(i) < PbestDistance(i) %若新路径长度变短
            Pbest(i,:)=Population(i,:); %更新Pbest
            PbestDistance(i)=PopDistance(i); %更新Pbest距离
        end
        
        %% 根据Pbest更新Gbest
        [mindis,index] = min(PbestDistance); %找出Pbest中最短距离

        if mindis < GbestDistance %若Pbest中最短距离小于Gbest距离
            Gbest = Pbest(index,:); %更新Gbest
            GbestDistance = mindis; %更新Gbest距离
        end
    end
    
    %% 显示此代信息
    fprintf('Iteration = %d, Min Distance = %.2f m  \n',gen,GbestDistance/1000)
    
	%% 存储此代最短距离
    GbestDisByGen(gen)=GbestDistance;
    
    %% 更新迭代次数
    gen=gen+1;
end

%% 计算结果数据输出到命令行
disp('-------------------------------------------------------------')
toc %显示运行时间
TextOutput(Gbest,GbestDistance)  %显示最优路径

%% 迭代图
figure
plot(GbestDisByGen,'LineWidth',2) %展示目标函数值历史变化
xlim([1 gen-1]) %设置 x 坐标轴范围
set(gca, 'LineWidth',1)
xlabel('Iterations')
ylabel('Min Distance(km)')
title('HPSO Process')

%% 绘制实际路线
DrawPath(Gbest,City)
