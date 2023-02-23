clear;

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

%% 计算在一步之内，SOC最多增加或减少多少（也即在一步之内能够传递多大范围）
OneStepMaxSOCStep = round((TransferFun(P_batt_min, Start_SOC, delta_t) - Start_SOC)/SOC_resolution + 1);
OneStepMinSOCStep = round((Start_SOC - TransferFun(P_batt_max, Start_SOC, delta_t))/SOC_resolution + 1);

load P_req P_req
k = length(P_req);
SOC_avail_range = nan*ones(num_SOC_states, k);
tmp_availSOC_upper = Start_SOC_idx; %SOC上界，对应的下标也越大
tmp_availSOC_lower = Start_SOC_idx; %定个上界和下界双指针，每一轮分别往上、下拓展
tmp_step_idx = 1;
SOC_avail_range(Start_SOC_idx, 1) = Start_SOC;
while(tmp_step_idx<=k) %因为这个代码只跑一次，所以无所谓优化不优化
    tmp_step_idx = tmp_step_idx + 1;
    tmp_availSOC_upper = min(tmp_availSOC_upper + OneStepMaxSOCStep, num_SOC_states); %下标往大了加
    tmp_availSOC_lower = max(tmp_availSOC_lower - OneStepMinSOCStep, 1); %下标往小了减
    idx = tmp_availSOC_lower:1:tmp_availSOC_upper;
    SOC_avail_range(idx, tmp_step_idx) = ((idx-1) * SOC_resolution) + SOC_min;
end

% t = SOC_range;
% c = SOC_min:SOC_resolution:SOC_max; %SOC上限51，SOC下限19，step0.01
% d=round(c*1000)/1000; %这个确实是不太理解，但是结果就是d==c==SOC以step0.01降序的序列
% SOC_range(1) = 50; % start SOC
% t(1) = 20; % end SOC
% SOC = nan*ones(3201,3602); % SOC_space * time_space(3602)
% SOC(:,1) = SOC_range; % start SOC
% SOC(:,3602) = t; % end SOC
% for i=2:3601
%     SOC(:,i)=d;
% end

SOC = SOC_avail_range;

[p_opt,fval] = DPforLongTermPrediction(SOC,@SubObjectFun);