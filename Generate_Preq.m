%% 利用整车参数，通过整车动力学方程，计算整车在驾驶循环下的请求功率P_req
load WLTC V;                                            % 导入预先定义好的驾驶循环的数组,为车速数组V，单位为km/h
delta_t = 1;                                            % 导入的驾驶循环的时间步长(s)
DrivingCycleSize=size(V);                               
DrivingCycleLength = DrivingCycleSize(2);               % 驾驶循环的长度（数据点个数）。Matlab是1-indexed下标从1开始，并且取数是用圆括号'()'
%%%% 可以直接用Matlab的矩阵运算代替for循环，大幅提高效率(比如下面加速度的计算）。矩阵运算本来也是Matlab的特色。原有的代码确实优化做得不是很好
Acceleration = (V(2:DrivingCycleLength) - V(1:DrivingCycleLength-1))/delta_t; % 加速度a = (v(k+1) -v(k))/delta_t
P_req = zeros(size(V));

% 车辆参数
delta = 1.1;                                            % delta为旋转质量换算系数
M = 1800;                                               % M为整车的质量，单位为kg
g = 9.8;                                                % g为重力加速度，单位为m/s^2
f = 0.01;                                              % f为滚动阻力系数
Cd = 0.374;                                              % Cd为风阻系数
A = 2.586;                                          % A为迎风面积，单位为m^2
yita_t = 0.92;                                    % 驱动链的效率

% 不同车速下，驱动电机的需求功率，未考虑爬坡，假设Δt为1秒
for i = 2:DrivingCycleLength
    P_req(i-1) = (delta*M*V(i-1)*Acceleration(i-1)/3.6 + M*g*f*V(i-1) + (Cd*A*V(i-1)^3)/21.15)/(3600*yita_t);
end
plot(P_req)