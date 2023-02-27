% 这个文档就把我看迷糊了，单步转移损失函数和论文里写的不一样，只考虑了油耗？？？
function L = SubObjectFun(P_RE)
Q_HV = 46000;                                                 % 汽油热值，为4.6×10^4 kJ/kg
P_RE_best = [5 10 15 20 25 30 35 40 45];
% fuel_g_kwh = [370, 305, 285, 282, 270, 267, 287, 292, 307];
% 通过燃油消耗率(g/kWh)和发电功率(kW)计算BSFC最佳燃油消耗曲线(kg/h)
m_fuel_best = [1.8500, 3.0500, 4.2750, 5.6400, 6.7500, 8.0100, 10.0450, 11.6800, 13.8150];
m_fuel = interp1(P_RE_best,m_fuel_best,P_RE);                 % 通过插值函数，找到当前增程器请求功率下的最小油耗值

% I_b_k = feval(TransferFun,k,SOC,P_RE);
% U_oc_k =  -1.031*exp(-35*SOC/100) + 3.685 + 0.2156*SOC/100 - 0.1178*(SOC/100)^2 + 0.3201*(SOC/100)^3;
% R_int = 0.1562*exp(-24.37*SOC/100) + 0.07446;
% P_batt_loss(k) = I_b_k^2*R_int;                             % 电池放热损失功率
% P_batt_loss = (U_oc_k*109)^2*R_int;

%% 当前方案就是损失函数只和发电功率有关
    if P_RE ==0 % 发电功率为0，亦有停机和怠速两种情况
%         L(k) = P_batt_loss;                                 % 假设控制变量P_RE为0，表示增程器是完全停机，不发电的，发动机也是停机的
        L = 0;
    else   
%         L(k) = P_RE_loss(k) + P_batt_loss;
        L = m_fuel/3600*Q_HV;
    end
end