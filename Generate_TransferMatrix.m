%% 从更高的层次看，我们希望我们的DP算法能够对于任意长时车速曲线都有较快的计算效率
% 因此，一些需要复杂计算的部分，可以通过提前预计算保存结果，在线应用时只需要查找或插值计算即可
% 如状态转移，假设delta_t已确定，那么SOC(k+1) = f(SOC(k), P_batt)
% 原代码中的P_batt与驾驶循环的P_req相关，这导致对于新的驾驶循环，需要重新计算P_batt(k) = P_req(k)-P_RE
% 因此考虑计算出Transfer Matrix保存SOC_new = (P_batt, SOC)的结果

% 时间网格大小(s)
delta_t = 1;

% P_batt范围和网格大小设置
P_batt_max=55;  %P_batt单位：kw
P_batt_min=-55;
P_batt_resolution = 1;
P_batt = P_batt_min:P_batt_resolution:P_batt_max;
num_P_batt = length(P_batt);

% SOC范围和网格大小设置
SOC_max = 51;
SOC_min = 19;
SOC_resolution = 0.01;
SOC = SOC_min:SOC_resolution:SOC_max; 
num_SOC_states = length(SOC);

% 电池包相关参数,这个内阻和开路电压在20~50的SOC范围内基本没啥变化。。。
Parallel = 1;         %并联数
Series = 96;       %串联数
U_oc = -0.925*exp(-0.0701*SOC) + 4.1 + SOC.*(-0.024 + SOC.*(0.000396 - 1.47*0.1^6*SOC));  %单体电池的开路电压，是SOC的函数
U_batt = U_oc*Series;                     %整个电池包的开路电压
R_int = 0.000178*exp(-0.048*SOC) + 0.00149;    %单体电池的电阻，也是SOC的函数
R_batt = (R_int*Series)/Parallel;              %整个电池包的电阻（####这儿有个疑问，并联等效电阻不应该是(R_int*b)/a吗####
Q_b = 50*3600;                                 % Q_b为电池容量，单位为Ah，需要转换为As

% shape: P_batt(kW) * SOC(0~100)
TransferValue = SOC - 100 * delta_t * (U_batt-(U_batt.^2-4000*R_batt.*P_batt.').^0.5)./(2*R_batt*Q_b);
% TransferValue = round(TransferValue*(1/SOC_resolution)) / (1/SOC_resolution); % 精确到指定位数（具体哪一位和SOC_resolution相关）

% TransferValue(i,j):
% i: (P_batt_resolution*(i-1)+P_batt_min); 对任意不在网格上的P_batt: i = round((P_batt-P_batt_min)/P_batt_resolution+1)
% j: (SOC_resolution*(j-1)+SOC_min); 对任意不在网格上的P_batt: j = round((SOC-SOC_min)/SOC_resolution+1)
save TransferValue TransferValue;