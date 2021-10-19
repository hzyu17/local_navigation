clear all
close all
clc

number_of_files_sim = 11;
number_of_files_real = 5;
%% specifying the filenames in simulation 
% 指定csv文件
files_depth_noi_sim = [
'../simulation/depth_noi/routeCommand_2019_05_13_17_54_27.csv';
'../simulation/depth_noi/routeCommand_2019_05_13_18_05_30.csv';
'../simulation/depth_noi/routeCommand_2019_05_13_18_15_41.csv';
'../simulation/depth_noi/routeCommand_2019_05_13_18_25_53.csv';
'../simulation/depth_noi/routeCommand_2019_05_13_18_34_50.csv';
'../simulation/depth_noi/routeCommand_2019_05_14_16_40_04.csv';
'../simulation/depth_noi/routeCommand_2019_05_14_16_48_20.csv';
'../simulation/depth_noi/routeCommand_2019_05_14_16_56_47.csv';
'../simulation/depth_noi/routeCommand_2019_05_14_17_05_50.csv';
'../simulation/depth_noi/routeCommand_2019_05_14_17_14_53.csv';
'../simulation/depth_noi/routeCommand_2019_05_14_19_15_12.csv';
'../simulation/depth_noi/routeCommand_2019_05_14_19_25_10.csv'
];

files_rgb_sim = [
'../simulation/rgb/routeCommand_2019_05_15_14_36_49.csv';
'../simulation/rgb/routeCommand_2019_05_15_14_51_13.csv';
'../simulation/rgb/routeCommand_2019_05_15_15_03_21.csv';
'../simulation/rgb/routeCommand_2019_05_15_15_18_11.csv';
'../simulation/rgb/routeCommand_2019_05_15_15_30_49.csv';
'../simulation/rgb/routeCommand_2019_05_15_15_42_35.csv';
'../simulation/rgb/routeCommand_2019_05_15_15_56_24.csv';
'../simulation/rgb/routeCommand_2019_05_15_16_08_44.csv';
'../simulation/rgb/routeCommand_2019_05_15_16_20_30.csv';
'../simulation/rgb/routeCommand_2019_05_15_16_32_11.csv';
'../simulation/rgb/routeCommand_2019_05_15_16_44_29.csv';
'../simulation/rgb/routeCommand_2019_05_15_16_57_22.csv'
 ];

files_dep_semantic_noi_sim = [
'../simulation/semantic_noi/routeCommand_2019_05_13_18_56_09.csv';
'../simulation/semantic_noi/routeCommand_2019_05_14_14_06_32.csv';
'../simulation/semantic_noi/routeCommand_2019_05_14_14_17_33.csv';
'../simulation/semantic_noi/routeCommand_2019_05_14_14_26_58.csv';
'../simulation/semantic_noi/routeCommand_2019_05_14_14_53_03.csv';
'../simulation/semantic_noi/routeCommand_2019_05_14_15_13_15.csv';
'../simulation/semantic_noi/routeCommand_2019_05_14_15_34_50.csv';
'../simulation/semantic_noi/routeCommand_2019_05_14_15_43_07.csv';
'../simulation/semantic_noi/routeCommand_2019_05_14_15_55_14.csv';
'../simulation/semantic_noi/routeCommand_2019_05_14_16_13_10.csv';
'../simulation/semantic_noi/routeCommand_2019_05_14_18_49_00.csv'
];

len_file_dep_sim = length(files_depth_noi_sim(:,1));
len_file_dep_sem_sim = length(files_dep_semantic_noi_sim(:,1));
len_file_rgb_sim = length(files_rgb_sim(:,1));

% cell collecting the average velocities of all the tests (6 * file numbers) 
% the cell: 
% 1st row: the avg actual velocities
% 2nd row: the avg command velocities
% 3rd row: the avg minimum distances
% 4rd row: the avg average distances
% 5th row: robotPos
% 6th row: avg_vel_decline_percentage_in_range

%下面这几个cell里面是我们需要的结果数据。每一行是一个指标（比如说实际速度），每一列是一次trial
%例如：cell_avg_dep_noi_sim{1,1}是第一个trial中(depth noi simulation的情况)，每次遇到人的过程中的平均速度，是1*19的数组，说明第一次trial中遇到了19次人。
cell_avg_dep_noi_sim = cell(6,len_file_dep_sim);   
cell_avg_dep_sem_noi_sim = cell(6,len_file_dep_sem_sim);
cell_avg_rgb_sim = cell(6,len_file_rgb_sim);
total_duration_dep_noi_sim = zeros(1,len_file_dep_sim);
total_duration_dep_sem_noi_sim = zeros(1,len_file_dep_sem_sim);
total_duration_rgb_sim = zeros(1,len_file_rgb_sim);
% loop on the sim files
range_near = 3; % 3m 范围
% 批量处理csv sim
for i = 1:number_of_files_sim
    i_filename_dep_sim = files_depth_noi_sim(i,:);
    i_filename_dep_sem_sim = files_dep_semantic_noi_sim(i,:);
    i_filename_dep_rgb_sim = files_rgb_sim(i,:);
    [ cell_avg_dep_noi_sim{1,i}, cell_avg_dep_noi_sim{2,i}, cell_avg_dep_noi_sim{3,i},...
        cell_avg_dep_noi_sim{4,i},cell_avg_dep_noi_sim{5,i},cell_avg_dep_noi_sim{6,i},total_duration_dep_noi_sim(1,i) ] = ...
        processOneFile_sim( i_filename_dep_sim,'depth', range_near);
    [ cell_avg_dep_sem_noi_sim{1,i}, cell_avg_dep_sem_noi_sim{2,i}, cell_avg_dep_sem_noi_sim{3,i},...
        cell_avg_dep_sem_noi_sim{4,i},cell_avg_dep_sem_noi_sim{5,i},cell_avg_dep_sem_noi_sim{6,i},total_duration_dep_sem_noi_sim(1,i) ] = ...
        processOneFile_sim( i_filename_dep_sem_sim ,'depth_semantic',range_near);
    [ cell_avg_rgb_sim{1,i}, cell_avg_rgb_sim{2,i}, cell_avg_rgb_sim{3,i},...
        cell_avg_rgb_sim{4,i},cell_avg_rgb_sim{5,i},cell_avg_rgb_sim{6,i},total_duration_rgb_sim(1,i) ] = ...
        processOneFile_sim( i_filename_dep_rgb_sim ,'rgb',range_near);
end

%% draw trajectories
% 画轨迹，包括地图、障碍物。在函数drawRect_all_in_one中实现
draw_number_of_trj = 10;
figure
hold on
grid minor
drawRect_all_in_one(cell_avg_dep_sem_noi_sim,draw_number_of_trj)
figure
hold on
grid minor
drawRect_all_in_one(cell_avg_rgb_sim,draw_number_of_trj)
figure
hold on
grid minor
drawRect_all_in_one(cell_avg_dep_noi_sim,draw_number_of_trj)
figure
hold on
grid minor
drawRect_all_in_one(cell_avg_dep_sem_noi_sim,draw_number_of_trj)
drawRect_all_in_one(cell_avg_rgb_sim,draw_number_of_trj)
drawRect_all_in_one(cell_avg_dep_noi_sim,draw_number_of_trj)



%% ----------------- in real scenario ------------------ %%
%% ----------------- 以下是实际场景的处理 ---------------%%
files_depth_noi_real = [
    '../realscenario/depth/routeCommand_crush2_misstur0_2019_05_16_16_42_08.csv';
    '../realscenario/depth/routeCommand_crush2_misstur0_2019_05_16_17_02_25.csv';
    '../realscenario/depth/routeCommand_crush3_misstur0_2019_05_16_16_15_32.csv';
    '../realscenario/depth/routeCommand_crush4_misstur0_2019_05_16_16_36_06.csv';
    '../realscenario/depth/routeCommand_crush4_misstur0_2019_05_16_16_57_04.csv'
    ];
files_dep_semantic_noi_real = [
    '../realscenario/depth-semantic/routeCommand_crush0_misstur0_2019_05_16_15_55_13.csv';
    '../realscenario/depth-semantic/routeCommand_crush0_misstur2_2019_05_16_15_49_35.csv';
    '../realscenario/depth-semantic/routeCommand_crush1_misstur0_2019_05_16_15_59_23.csv';
    '../realscenario/depth-semantic/routeCommand_crush1_misstur1_2019_05_16_15_36_57.csv';
    '../realscenario/depth-semantic/routeCommand_crush2_misstur1_2019_05_16_15_31_44.csv'
    ];
len_file_dep_real = length(files_depth_noi_real(:,1));
len_file_dep_sem_real = length(files_dep_semantic_noi_real(:,1));

% cell collecting the average velocities of all the tests (2 * file numbers) 
% the cell: 
% 1st row: the avg actual velocities
% 2nd row: the avg command velocities
% 3rd row: the avg minimum distances
% 4th row: the avg of avg distances
% 5th row: robotPos
% 6th row: vel_decline_percent_in_range
% 7th row: cmd_vel_decline_percent_in_range

cell_avg_dep_noi_real = cell(7,len_file_dep_real);   
cell_avg_dep_sem_noi_real = cell(7,len_file_dep_sem_real);
cell_avg_rgb_real = cell(7,len_file_dep_sem_real);
total_duration_dep_noi_real = zeros(1,len_file_dep_sem_real);
total_duration_dep_sem_noi_real = zeros(1,len_file_dep_sem_real);
total_duration_rgb_real = zeros(1,len_file_dep_sem_real);
% ... no rgb data in real scenario tests ...

% loop on the real scenario files
for i = 1:number_of_files_real
    i_filename_dep_noi_real = files_depth_noi_real(i,:);
    i_filename_dep_sem_noi_real = files_dep_semantic_noi_real(i,:);
    [ cell_avg_dep_noi_real{1,i}, cell_avg_dep_noi_real{2,i},cell_avg_dep_noi_real{3,i},...
        cell_avg_dep_noi_real{4,i},cell_avg_dep_noi_real{5,i},cell_avg_dep_noi_real{6,i}, ...
        cell_avg_dep_noi_real{7,i},total_duration_dep_noi_real(1,i)] = ...
        processOneFile_real( i_filename_dep_noi_real,range_near );
    [ cell_avg_dep_sem_noi_real{1,i}, cell_avg_dep_sem_noi_real{2,i},cell_avg_dep_sem_noi_real{3,i},...
        cell_avg_dep_sem_noi_real{4,i},cell_avg_dep_sem_noi_real{5,i},cell_avg_dep_sem_noi_real{6,i}, ...
        cell_avg_dep_sem_noi_real{7,i},total_duration_dep_sem_noi_real(1,i) ] = ...
        processOneFile_real( i_filename_dep_sem_noi_real,range_near );
end

%% plot delines in actual velocities and command velocities
% average
% row 1: depth_real
% row 2: depth_semantic_real
decline_vel_abs_real = zeros(2,number_of_files_real);
decline_cmd_vel_real = zeros(2,number_of_files_real);

% var
var_decline_vel_abs_real = zeros(2,number_of_files_real);
var_decline_cmd_vel_real = zeros(2,number_of_files_real);

for i = 1:number_of_files_real
    % 平均值 方差 实际速度 depth_noi
    decline_vel_abs_real(1,i) = sum(cell_avg_dep_noi_real{6,i})/length(cell_avg_dep_noi_real{6,i});
    var_decline_vel_abs_real(1,i) = var(cell_avg_dep_noi_real{6,i});
    % 平均值 方差 控制输出 depth_noi
    decline_vel_abs_real(2,i) = sum(cell_avg_dep_sem_noi_real{6,i})/length(cell_avg_dep_sem_noi_real{6,i});
    var_decline_vel_abs_real(2,i) = var(cell_avg_dep_sem_noi_real{6,i});
    
    % 平均值 方差 实际速度 depth_semantic
    decline_cmd_vel_real(1,i) = sum(cell_avg_dep_noi_real{7,i})/length(cell_avg_dep_noi_real{7,i});
    var_decline_cmd_vel_real(1,i) = var(cell_avg_dep_noi_real{7,i});
    % 平均值 方差 控制输出 depth_semantic
    decline_cmd_vel_real(2,i) = sum(cell_avg_dep_sem_noi_real{7,i})/length(cell_avg_dep_sem_noi_real{7,i});
    var_decline_cmd_vel_real(2,i) = var(cell_avg_dep_sem_noi_real{7,i});
end

%%
data_depth = decline_vel_abs_real(1,:)';
var_data_depth = var_decline_vel_abs_real(1,:)';
data_depth_semantic = decline_vel_abs_real(2,:)';
var_data_depth_semantic = var_decline_vel_abs_real(2,:)';
figure 
lgd = {'depth','depth_semantic'};
groupnames= {'trial 1', 'trial 2','trial 3','trial 4','trial 5'};
data_barweb = [data_depth, data_depth_semantic].*100;
var_data_barweb = [var_data_depth, var_data_depth_semantic].*100;
barweb(data_barweb,var_data_barweb,1,groupnames, ...
    'decline percentage in velocity','trials','decline(%)',jet, 'y',lgd,2,'plot')

%%
figure
x = [-1,-2;-4,-5;-6,7;8,9;-10,11];
y = [0.1,0.2;0.3,0.4;0.5,0.6;0.7,0.8;1,1.1];
barweb(x,y)
% bar([data_var_depth,var_data_depth_semantic],'BaseValue',10)
% bar(data_depth,'b')
% bar([data_depth_semantic,var_data_depth_semantic],'stacked')
% errorbar(data_depth,data_var_depth,'LineStyle','none')
% errorbar(data_depth_semantic,var_data_depth_semantic,'LineStyle','none')
% legend('depth','var','depth_semantic','var')

%%
% interference or missturns depth
crush_and_missturns_depth = zeros(len_file_dep_real,2);
crush_and_missturns_depth_semantic = zeros(len_file_dep_sem_real,2);
for i = 1:len_file_dep_real
    indx_crush = strfind( files_depth_noi_real(i,:), 'crush' ) + length('crush');
    indx_missturns = strfind( files_depth_noi_real(i,:), 'misstur' ) + length('misstur');
    crush_and_missturns_depth(i,1) = str2num(files_depth_noi_real(i,indx_crush));
    crush_and_missturns_depth(i,2) = str2num(files_depth_noi_real(i,indx_missturns));
end

% interference or missturns depth semantics
for i = 1:len_file_dep_real
    indx_crush = strfind( files_dep_semantic_noi_real(i,:), 'crush' ) + length('crush');
    indx_missturns = strfind( files_dep_semantic_noi_real(i,:), 'misstur' ) + length('misstur');
    crush_and_missturns_depth_semantic(i,1) = str2num(files_dep_semantic_noi_real(i,indx_crush));
    crush_and_missturns_depth_semantic(i,2) = str2num(files_dep_semantic_noi_real(i,indx_missturns));
end

%% -------------------- calculation of average numbers ---------------------- %%
%% avg on total duration
% sim
avg_total_duration_dep_noi_sim = sum(total_duration_dep_noi_sim)/length(total_duration_dep_noi_sim);
avg_total_duration_dep_sem_noi_sim = sum(total_duration_dep_sem_noi_sim)/length(total_duration_dep_sem_noi_sim);
avg_total_duration_rgb_noi_sim = sum(total_duration_rgb_sim)/length(total_duration_rgb_sim);

% real world
avg_total_duration_dep_noi_real = sum(total_duration_dep_noi_real)/length(total_duration_dep_noi_real);
avg_total_duration_dep_sem_noi_real = sum(total_duration_dep_sem_noi_real)/length(total_duration_dep_noi_real);
avg_total_duration_rgb_noi_real = sum(total_duration_rgb_real)/length(total_duration_dep_noi_real);

%% avg on command velocities, actual velocities, minimum distances and average distances
% simulation
[avg_total_dep_noi_sim,avg_total_dep_sem_noi_sim,avg_total_rgb_sim] = ...
    avgCmdvelMindist(cell_avg_dep_noi_sim,cell_avg_dep_sem_noi_sim,cell_avg_rgb_sim,number_of_files_sim,true,'sim');
% real world
[avg_total_dep_noi_real,avg_total_dep_sem_noi_real,avg_total_rgb_real] = ...
    avgCmdvelMindist(cell_avg_dep_noi_real,cell_avg_dep_sem_noi_real,cell_avg_rgb_real,number_of_files_real,true,'real');

%% avg on the decline_percentage of the velocity in the range threshold, compared to normal vel

%% avg crushes, miss turns and duration in real scenario
% avg crushes and missturns
avg_crush_and_missturns_dep_sem_noi = sum(crush_and_missturns_depth_semantic,1)/number_of_files_real;
avg_crush_and_missturns_dep_noi = sum(crush_and_missturns_depth,1)/number_of_files_real;


%% plot interference bars for real_scenario
figure
crushs = [crush_and_missturns_depth(:,1),crush_and_missturns_depth_semantic(:,1)];
bar(crushs);
title('number of crush in real world trials')
grid on 
set(gca,'XTickLabel',{'0','1','2','3','4','5','6'});
% set(ch,'FaceVertexCData',[1 0 1;0 0 0;])
legend('depth','depth_semantic');
xlabel('trial ');
ylabel('crushes');

figure
crushs = [crush_and_missturns_depth(:,2),crush_and_missturns_depth_semantic(:,2)];
bar(crushs);
title('number of missed turns in real world trials')
grid on 
set(gca,'XTickLabel',{'0','1','2','3','4','5','6'});
% set(ch,'FaceVertexCData',[1 0 1;0 0 0;])
legend('depth','depth_semantic');
xlabel('trail ');
ylabel('miss turns');

% map & plot
% h = drawRect(robotPos);

%% drafts
% test draw obstacles
file_example = '../simulation/rgb/routeCommand_2019_05_15_14_36_49.csv';
data_example = importfile(file_example);
robotPos = data_example(2:end,7:8);
drawRect(robotPos,file_example,'rgb');
