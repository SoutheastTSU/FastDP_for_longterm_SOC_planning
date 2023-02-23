clear;
clc;

P_batt_max=55;  %P_batt��λ��kw
P_batt_min=-55;
P_batt_resolution = 1;

SOC_max = 51;
SOC_min = 19;
SOC_resolution = 0.01;

delta_t= 1;

%% SOC������
SOC_range = nan*ones((SOC_max-SOC_min)/SOC_resolution+1,1); % 3201 = (51-19)*1000 + 1
num_SOC_states = length(SOC_range);
Start_SOC = 50;
Start_SOC_idx = (Start_SOC-SOC_min)/SOC_resolution+1;

%% SOC��������㣨��ʼSOC�̶��������ܵ����SOC��Χ����������ģ� 
% ������һ��֮�ڣ�SOC������ӻ���ٶ��٣�Ҳ����һ��֮���ܹ����ݶ��Χ��
OneStepMaxSOCStep = round((TransferFun(P_batt_min, Start_SOC, delta_t) - Start_SOC)/SOC_resolution + 1);
OneStepMinSOCStep = round((Start_SOC - TransferFun(P_batt_max, Start_SOC, delta_t))/SOC_resolution + 1);

load P_req P_req
k = length(P_req);
SOC_avail_range = nan*ones(num_SOC_states, k);
tmp_availSOC_upper = Start_SOC_idx; %SOC�Ͻ磬��Ӧ���±�ҲԽ��
tmp_availSOC_lower = Start_SOC_idx; %�����Ͻ���½�˫ָ�룬ÿһ�ֱַ����ϡ�����չ
tmp_step_idx = 2;
SOC_avail_range(Start_SOC_idx, 1) = Start_SOC;
while(tmp_step_idx<=k)              % ��Ϊ�������ֻ��һ�Σ���������ν�Ż����Ż�
    tmp_availSOC_upper = min(tmp_availSOC_upper + OneStepMaxSOCStep, num_SOC_states); %�±������˼ӣ��������Ͻ�
    tmp_availSOC_lower = max(tmp_availSOC_lower - OneStepMinSOCStep, 1); %�±���С�˼����������½�
    idx = tmp_availSOC_lower:1:tmp_availSOC_upper;
    SOC_avail_range(idx, tmp_step_idx) = ((idx-1) * SOC_resolution) + SOC_min;
    tmp_step_idx = tmp_step_idx + 1;
end

SOC = SOC_avail_range;

[p_opt,fval] = DP(SOC,@SubObjectFun);

subplot(4,1,1);
plot(p_opt(:,1));
title('SOC����');
xlabel('time/s');
ylabel('SOC/%');

subplot(4,1,2);
plot(p_opt(:,2));
title('������������仯');
xlabel('time/s');
ylabel('P_R_E/kW');

subplot(4,1,3);
Q_HV = 46000;  
plot(p_opt(:,3)/Q_HV);
title('�ۻ�ȼ������');
xlabel('time/s');
ylabel('kg');

subplot(4,1,4);  
plot(P_req/Q_HV);
title('������');
xlabel('time/s');
ylabel('kW');


%%  DP������
function [p_opt,total_fuel] = DP(SOC,SubObjectFun)
% �����SOCӦ����SOC������
% xΪ״̬������һ�д���һ���׶ε�״̬
% m����DecisFun(k,s)��ʾ��k�׶�s״̬�µĿ��þ��߼��ϣ��Ľ���������ȥ������һ���裬�����þ��߼�ʹ��SOC������Χ����δ�����ܿ��ǵĵ�س�ŵ�������ľ��ߣ�ֱ����DP�������жϾ����Ƿ���ü��ɣ�
% m����SubObjFun(k,s,u)��ʾ��k�׶�s״̬�£�����u����ʱ�ĵ�����ʧ����
% m����TransFun(k,s,u)��ʾ��k�׶�s״̬�£�����u����ʱ��һʱ��ת�Ƶ���״̬
% m����ObjFun(v,f)��ʾ����k�׶ε����׶ε�ָ�꺯��ֵ����������˵��Ŀ��SOCƫ�ƴ����ĳͷ�����Ҳ�ڸĽ������������ˣ�

    tic
    SOC_max = 51;
    SOC_min = 19;
    SOC_resolution = 0.01;
    P_batt_max=55;  %P_batt��λ��kw
    P_batt_min=-55;
    P_batt_resolution = 1;
    k = size(SOC); k = k(2);            % kΪ�׶���
    SOCNotNan = ~isnan(SOC);            % SOC�����������£��ɴ��SOC״̬�±꼯��
    tmp_soc_states = find(SOCNotNan(:,k));        % �ҳ���k�׶η�NaN״̬���±꣬find���������ҵ�����Ԫ�صı��
    tmp_num_soc_states = length(tmp_soc_states);  % tmp1�ĳ��ȣ���ÿ���׶�k�£��ж����ֲ�ͬ��״̬���
    
    OptLoss = inf*ones(size(SOC));     % �ӵ�ǰʱ�̵�ǰ״̬���������loss(ѡ������action�õ�)����ģ��SOC������ͬ
    OptAction = nan*ones(size(SOC));              % ��ͬ�׶Ρ�״̬�µ����ž��߾��󣬳�ֵΪNaN
    
    % ���������̶�
    action_max=45;
    action_min=0;
    action_resolution=5;
    P_RE_vector = action_min:action_resolution:action_max;
    num_actions = length(P_RE_vector);
    LossOfActions = nan*ones(num_actions,1);          % ��ͬ����������Ӧ��loss��ֻ��P_RE�й�
    load TransferValue TransferValue                  % ��ǰԤ����õ�״̬ת�ƾ���
    
    %% ���ڵ�ǰĿ�꺯���������ͺģ������ʧ������ֱ����ÿ��action���õ�
    for action_idx = 1:num_actions
        LossOfActions(action_idx) = SubObjectFun(P_RE_vector(action_idx));
    end
    
    %% ������ֹʱ�̣���״̬�µ���ʧ���ɰ���Ŀ��SOCԼ����
    target_SOC = 20;
    terminal_SOC_penalty = 500; % ��ֹSOC�ͷ�ϵ����һ���ɵ�����
    % Ϊ�˱���ͷ���ϵ�������Ƚ�SOC�������С���1(SOC���ȳ��������С)��������(SOC)ƽ��֮�󲻻��С
    OptLoss(:,k) = terminal_SOC_penalty * (SOC(tmp_soc_states, k)/SOC_resolution - target_SOC/SOC_resolution).^2;
    save LossOfActions LossOfActions;
    
    load P_req P_req;
    
    for step_idx = k-1:-1:1                      % �Ӻ���ǰ�������f_opt��d_opt
        tmp_SOCnotNan = find(SOCNotNan(:, step_idx)); % �ҳ�״ֵ̬����NaN���±꼯��
        tmp_num_SOC = length(tmp_SOCnotNan);
        for avail_soc_idx = 1:tmp_num_SOC
            SOC_idx = tmp_SOCnotNan(avail_soc_idx); % �ѷ�nan�±꼯�ϵ��±�ת��ΪԭʼSOC�±�
            for action_idx = 1:num_actions      
                 P_batt = P_req(step_idx) - P_RE_vector(action_idx);
                 P_batt = min(P_batt, P_batt_max);
                 P_batt = max(P_batt, P_batt_min); %�������ŵ繦��
                 P_batt_idx = round((P_batt-P_batt_min)/P_batt_resolution+1);
                 
                 % ��TransfrerValue��õ�SOC��SOC_resolutionһ���������С��ʹtmp_SOC_next�϶���ֱ�Ӷ�λ����Ӧ����
                 tmp_SOC_next = round(TransferValue(P_batt_idx, SOC_idx)/SOC_resolution)*SOC_resolution; % ����state��action��״̬ת��
                 if(tmp_SOC_next>SOC_max || tmp_SOC_next<SOC_min)
                     continue; % �����ᵼ��SOCԽ��ľ���
                 end
                 SOC_next_idx = round((tmp_SOC_next - SOC_min)/SOC_resolution+1);
                 tmp_loss = LossOfActions(action_idx) + OptLoss(SOC_next_idx, step_idx+1); % ����loss��״̬ת�ƺ���ۻ�loss
                                   
                 if tmp_loss <= OptLoss(SOC_idx, step_idx)        % ��ÿ���׶�ʹloss��С��action
                     OptAction(SOC_idx, step_idx) = action_idx;   % ����ÿ����״̬�����ŵ�loss����Ӧ�ľ��߱������±�
                     OptLoss(SOC_idx, step_idx) = tmp_loss;       % ����ÿ����״̬�����ŵ�loss
                 end
            end
        end
    end
    save OptActions OptAction;
    save OptLoss OptLoss;
    save SOCNotNan SOCNotNan;
    toc
    time_consume = num2str(toc);
    
    total_fuel = OptLoss(SOCNotNan(:,1), 1);      % �Ӻ���ǰ��������ʼʱ�̣������ۻ��ͺ�
    tmp_state = nan*ones(k,1);                    % ���ڴ洢���׶ε�״ֵ̬(SOC)
    tmp_action = nan*ones(k,1);                   % ���ڴ洢���׶εľ���ֵ
    tmp_fuel = nan*ones(k,1);                     % ���ڴ洢���׶ε�ָ�꺯��ֵ
    start_index = find(SOCNotNan(:,1));           % �ҵ���ʼ״̬����±�
    
    % load P_req P_req;
    % for k = 1:1:3602
    %     P_RE_vector = feval(DecisionFun,k);
    %     num_actions = length(P_RE_vector);
    %     for action_idx = 1: num_actions
    %         P_batt(k,action_idx) = P_req(k) - P_RE_vector(action_idx);  % ��ͬʱ�̣���صĳ�ŵ繦��
    %     end
    % end
    
    % state�ĳ��ȿ϶���k��action�ĳ��ȿ϶���k-1(���������һ��),
    % fuel��actionֱ�ӹ���������k-1ʱ�̵�action������fuel�Ǽ���kʱ���ϵ�(��ʼʱ���ۻ�fuelΪ0)
    SOC_idx = start_index;
    tmp_action(1) = OptAction(SOC_idx, 1);
    tmp_fuel(1) = 0;
    tmp_state(1) = SOC(SOC_idx, 1);
    P_batt = min(P_req(1) - P_RE_vector(tmp_action(1)), P_batt_max);
    P_batt = max(P_batt, P_batt_min); %�������ŵ繦��
    P_batt_idx = round((P_batt-P_batt_min)/P_batt_resolution+1);
    SOC_idx = round((TransferValue(P_batt_idx, SOC_idx) - SOC_min)/SOC_resolution+1);
    
    for i=2:k-1
        tmp_action(i) = OptAction(SOC_idx, i);       % ��ǰʱ�̵ľ���ֵ�±�
        tmp_state(i) = SOC(SOC_idx, i);              % ��ǰʱ�̵�״ֵ̬
        tmp_fuel(i) = tmp_fuel(i-1) + LossOfActions(tmp_action(i-1));   % ��ǰʱ���ۻ��ͺ�Ϊ��ʱ��action�����ĵ����ͺ�+��ʱ���ۻ��ͺ�
        P_batt = min(P_req(i) - P_RE_vector(tmp_action(i)), P_batt_max);
        P_batt = max(P_batt, P_batt_min); %�������ŵ繦��
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