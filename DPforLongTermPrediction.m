clear;
clc;

P_batt_max=55;  %P_batt单位：kw
P_batt_min=-55;
P_batt_resolution = 1;

SOC_max = 51;
SOC_min = 19;
SOC_resolution = 0.01;

delta_t= 1;

%% SOC可行域
SOC_range = nan*ones((SOC_max-SOC_min)/SOC_resolution+1,1); % 3201 = (51-19)*1000 + 1
num_SOC_states = length(SOC_range);
Start_SOC = 50;
Start_SOC_idx = (Start_SOC-SOC_min)/SOC_resolution+1;

%% SOC可行域计算（初始SOC固定，所以能到达的SOC范围是慢慢扩大的） 
% 计算在一步之内，SOC最多增加或减少多少（也即在一步之内能够传递多大范围）
OneStepMaxSOCStep = round((TransferFun(P_batt_min, Start_SOC, delta_t) - Start_SOC)/SOC_resolution + 1);
OneStepMinSOCStep = round((Start_SOC - TransferFun(P_batt_max, Start_SOC, delta_t))/SOC_resolution + 1);

load P_req P_req
k = length(P_req);
SOC_avail_range = nan*ones(num_SOC_states, k);
tmp_availSOC_upper = Start_SOC_idx; %SOC上界，对应的下标也越大
tmp_availSOC_lower = Start_SOC_idx; %定个上界和下界双指针，每一轮分别往上、下拓展
tmp_step_idx = 2;
SOC_avail_range(Start_SOC_idx, 1) = Start_SOC;
while(tmp_step_idx<=k)              % 因为这个代码只跑一次，所以无所谓优化不优化
    tmp_availSOC_upper = min(tmp_availSOC_upper + OneStepMaxSOCStep, num_SOC_states); %下标往大了加，不超过上界
    tmp_availSOC_lower = max(tmp_availSOC_lower - OneStepMinSOCStep, 1); %下标往小了减，不低于下界
    idx = tmp_availSOC_lower:1:tmp_availSOC_upper;
    SOC_avail_range(idx, tmp_step_idx) = ((idx-1) * SOC_resolution) + SOC_min;
    tmp_step_idx = tmp_step_idx + 1;
end

SOC = SOC_avail_range;

[p_opt,fval] = DP(SOC,@SubObjectFun);

subplot(4,1,1);
plot(p_opt(:,1));
title('SOC曲线');
xlabel('time/s');
ylabel('SOC/%');

subplot(4,1,2);
plot(p_opt(:,2));
title('增程器工作点变化');
xlabel('time/s');
ylabel('P_R_E/kW');

subplot(4,1,3);
Q_HV = 46000;  
plot(p_opt(:,3)/Q_HV);
title('累积燃油消耗');
xlabel('time/s');
ylabel('kg');

subplot(4,1,4);  
plot(P_req/Q_HV);
title('需求功率');
xlabel('time/s');
ylabel('kW');


%%  DP主函数
function [p_opt,total_fuel] = DP(SOC,SubObjectFun)
% 输入的SOC应当是SOC可行域
% x为状态变量，一列代表一个阶段的状态
% m函数DecisFun(k,s)表示，k阶段s状态下的可用决策集合（改进代码里面去除了这一步骤，不可用决策即使得SOC超出范围（或未来可能考虑的电池充放电电流）的决策，直接在DP过程中判断决策是否可用即可）
% m函数SubObjFun(k,s,u)表示，k阶段s状态下，采用u决策时的单步损失函数
% m函数TransFun(k,s,u)表示，k阶段s状态下，采用u决策时下一时刻转移到的状态
% m函数ObjFun(v,f)表示，第k阶段到最后阶段的指标函数值（可能是想说是目标SOC偏移带来的惩罚项，这个也在改进代码里弃用了）

    tic
    SOC_max = 51;
    SOC_min = 19;
    SOC_resolution = 0.01;
    P_batt_max=55;  %P_batt单位：kw
    P_batt_min=-55;
    P_batt_resolution = 1;
    k = size(SOC); k = k(2);            % k为阶段数
    SOCNotNan = ~isnan(SOC);            % SOC可行域限制下，可达的SOC状态下标集合
    tmp_soc_states = find(SOCNotNan(:,k));        % 找出第k阶段非NaN状态的下标，find函数用于找到非零元素的编号
    tmp_num_soc_states = length(tmp_soc_states);  % tmp1的长度，即每个阶段k下，有多少种不同的状态结果
    
    OptLoss = inf*ones(size(SOC));     % 从当前时刻当前状态出发的最佳loss(选择最优action得到)。规模与SOC数组相同
    OptAction = nan*ones(size(SOC));              % 不同阶段、状态下的最优决策矩阵，初值为NaN
    
    % 决策向量固定
    action_max=45;
    action_min=0;
    action_resolution=5;
    P_RE_vector = action_min:action_resolution:action_max;
    num_actions = length(P_RE_vector);
    LossOfActions = nan*ones(num_actions,1);          % 不同决策向量对应的loss，只与P_RE有关
    load TransferValue TransferValue                  % 提前预计算好的状态转移矩阵
    
    %% 由于当前目标函数仅包含油耗，因此损失函数可直接由每种action查表得到
    for action_idx = 1:num_actions
        LossOfActions(action_idx) = SubObjectFun(P_RE_vector(action_idx));
    end
    
    %% 计算终止时刻，各状态下的损失（可包含目标SOC约束）
    target_SOC = 20;
    terminal_SOC_penalty = 500; % 终止SOC惩罚系数是一个可调参数
    % 为了避免惩罚项系数过大，先将SOC的网格大小变成1(SOC都先除以网格大小)，这样δ(SOC)平方之后不会变小
    OptLoss(:,k) = terminal_SOC_penalty * (SOC(tmp_soc_states, k)/SOC_resolution - target_SOC/SOC_resolution).^2;
    save LossOfActions LossOfActions;
    
    load P_req P_req;
    
    for step_idx = k-1:-1:1                      % 从后往前递推求出f_opt和d_opt
        tmp_SOCnotNan = find(SOCNotNan(:, step_idx)); % 找出状态值不是NaN的下标集合
        tmp_num_SOC = length(tmp_SOCnotNan);
        for avail_soc_idx = 1:tmp_num_SOC
            SOC_idx = tmp_SOCnotNan(avail_soc_idx); % 把非nan下标集合的下标转化为原始SOC下标
            for action_idx = 1:num_actions      
                 P_batt = P_req(step_idx) - P_RE_vector(action_idx);
                 P_batt = min(P_batt, P_batt_max);
                 P_batt = max(P_batt, P_batt_min); %电池最大充放电功率
                 P_batt_idx = round((P_batt-P_batt_min)/P_batt_resolution+1);
                 
                 % 令TransfrerValue查得的SOC与SOC_resolution一样的网格大小，使tmp_SOC_next肯定能直接定位到对应网格
                 tmp_SOC_next = round(TransferValue(P_batt_idx, SOC_idx)/SOC_resolution)*SOC_resolution; % 根据state和action查状态转移
                 if(tmp_SOC_next>SOC_max || tmp_SOC_next<SOC_min)
                     continue; % 跳过会导致SOC越界的决策
                 end
                 SOC_next_idx = round((tmp_SOC_next - SOC_min)/SOC_resolution+1);
                 tmp_loss = LossOfActions(action_idx) + OptLoss(SOC_next_idx, step_idx+1); % 单步loss和状态转移后的累积loss
                                   
                 if tmp_loss <= OptLoss(SOC_idx, step_idx)        % 找每个阶段使loss最小的action
                     OptAction(SOC_idx, step_idx) = action_idx;   % 保存每个子状态下最优的loss所对应的决策变量的下标
                     OptLoss(SOC_idx, step_idx) = tmp_loss;       % 保存每个子状态下最优的loss
                 end
            end
        end
    end
    save OptActions OptAction;
    save OptLoss OptLoss;
    save SOCNotNan SOCNotNan;
    toc
    time_consume = num2str(toc);
    
    total_fuel = OptLoss(SOCNotNan(:,1), 1);      % 从后往前迭代到初始时刻，就是累积油耗
    tmp_state = nan*ones(k,1);                    % 用于存储各阶段的状态值(SOC)
    tmp_action = nan*ones(k,1);                   % 用于存储各阶段的决策值
    tmp_fuel = nan*ones(k,1);                     % 用于存储各阶段的指标函数值
    start_index = find(SOCNotNan(:,1));           % 找到初始状态点的下标
    
    % load P_req P_req;
    % for k = 1:1:3602
    %     P_RE_vector = feval(DecisionFun,k);
    %     num_actions = length(P_RE_vector);
    %     for action_idx = 1: num_actions
    %         P_batt(k,action_idx) = P_req(k) - P_RE_vector(action_idx);  % 不同时刻，电池的充放电功率
    %     end
    % end
    
    % state的长度肯定是k，action的长度肯定是k-1(不包含最后一刻),
    % fuel和action直接关联，但是k-1时刻的action产生的fuel是加在k时刻上的(初始时刻累积fuel为0)
    SOC_idx = start_index;
    tmp_action(1) = OptAction(SOC_idx, 1);
    tmp_fuel(1) = 0;
    tmp_state(1) = SOC(SOC_idx, 1);
    P_batt = min(P_req(1) - P_RE_vector(tmp_action(1)), P_batt_max);
    P_batt = max(P_batt, P_batt_min); %电池最大充放电功率
    P_batt_idx = round((P_batt-P_batt_min)/P_batt_resolution+1);
    SOC_idx = round((TransferValue(P_batt_idx, SOC_idx) - SOC_min)/SOC_resolution+1);
    
    for i=2:k-1
        tmp_action(i) = OptAction(SOC_idx, i);       % 当前时刻的决策值下标
        tmp_state(i) = SOC(SOC_idx, i);              % 当前时刻的状态值
        tmp_fuel(i) = tmp_fuel(i-1) + LossOfActions(tmp_action(i-1));   % 当前时刻累积油耗为上时刻action产生的单步油耗+上时刻累积油耗
        P_batt = min(P_req(i) - P_RE_vector(tmp_action(i)), P_batt_max);
        P_batt = max(P_batt, P_batt_min); %电池最大充放电功率
        P_batt_idx = round((P_batt-P_batt_min)/P_batt_resolution+1);
        tmp_SOC = max(SOC_min, TransferValue(P_batt_idx, SOC_idx));
        tmp_SOC = min(tmp_SOC, SOC_max);
        SOC_idx = round((tmp_SOC - SOC_min)/SOC_resolution+1);
    end
    tmp_state(k) = SOC(SOC_idx, i);
    tmp_fuel(k) = tmp_fuel(k-1) + LossOfActions(tmp_action(k-1));
    
    p_opt = [tmp_state, tmp_action, tmp_fuel];
    save p_opt.mat p_opt;
end