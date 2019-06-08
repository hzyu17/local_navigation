clear all
close all
clc

number_of_files_sim = 11;
number_of_files_real = 5;
%% specifying the filenames in simulation 
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

% cell collecting the average velocities of all the tests (2 * file numbers) 
% the cell: 
% 1st line: the avg actual velocities
% 2nd line: the avg command velocities
% 3rd line: the avg minimum distances
% 4rd line: the avg average distances
% 5th line: robotPos
cell_avg_dep_noi_sim = cell(5,len_file_dep_sim);   
cell_avg_dep_sem_noi_sim = cell(5,len_file_dep_sem_sim);
cell_avg_rgb_sim = cell(5,len_file_rgb_sim);
total_duration_dep_noi_sim = zeros(1,len_file_dep_sim);
total_duration_dep_sem_noi_sim = zeros(1,len_file_dep_sem_sim);
total_duration_rgb_sim = zeros(1,len_file_rgb_sim);
% loop on the sim files
for i = 1:number_of_files_sim
    i_filename_dep_sim = files_depth_noi_sim(i,:);
    i_filename_dep_sem_sim = files_dep_semantic_noi_sim(i,:);
    i_filename_dep_rgb_sim = files_rgb_sim(i,:);
    [ cell_avg_dep_noi_sim{1,i}, cell_avg_dep_noi_sim{2,i}, cell_avg_dep_noi_sim{3,i},...
        cell_avg_dep_noi_sim{4,i},cell_avg_dep_noi_sim{5,i},total_duration_dep_noi_sim(1,i) ] = processOneFile_sim( i_filename_dep_sim,'depth');
    [ cell_avg_dep_sem_noi_sim{1,i}, cell_avg_dep_sem_noi_sim{2,i}, cell_avg_dep_sem_noi_sim{3,i},...
        cell_avg_dep_sem_noi_sim{4,i},cell_avg_dep_sem_noi_sim{5,i},total_duration_dep_sem_noi_sim(1,i) ] = processOneFile_sim( i_filename_dep_sem_sim ,'depth_semantic');
    [ cell_avg_rgb_sim{1,i}, cell_avg_rgb_sim{2,i}, cell_avg_rgb_sim{3,i},...
        cell_avg_rgb_sim{4,i},cell_avg_rgb_sim{5,i},total_duration_rgb_sim(1,i) ] = processOneFile_sim( i_filename_dep_rgb_sim ,'rgb');
end

%% draw trajectories
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
% 1st line: the avg actual velocities
% 2nd line: the avg command velocities
% 3rd line: the avg minimum distances
% 4th line: the avg of avg distances
% 5th line: robotPos
cell_avg_dep_noi_real = cell(5,len_file_dep_real);   
cell_avg_dep_sem_noi_real = cell(5,len_file_dep_sem_real);
cell_avg_rgb_real = cell(5,len_file_dep_sem_real);
total_duration_dep_noi_real = zeros(1,len_file_dep_sem_real);
total_duration_dep_sem_noi_real = zeros(1,len_file_dep_sem_real);
total_duration_rgb_real = zeros(1,len_file_dep_sem_real);
% ... no rgb data in real scenario tests ...

% loop on the real scenario files
for i = 1:number_of_files_real
    i_filename_dep_noi_real = files_depth_noi_real(i,:);
    i_filename_dep_sem_noi_real = files_dep_semantic_noi_real(i,:);
    [ cell_avg_dep_noi_real{1,i}, cell_avg_dep_noi_real{2,i},cell_avg_dep_noi_real{3,i},...
        cell_avg_dep_noi_real{4,i},cell_avg_dep_noi_real{5,i},total_duration_dep_noi_real(1,i)] = processOneFile_real( i_filename_dep_noi_real );
    [ cell_avg_dep_sem_noi_real{1,i}, cell_avg_dep_sem_noi_real{2,i},cell_avg_dep_sem_noi_real{3,i},...
        cell_avg_dep_sem_noi_real{4,i},cell_avg_dep_sem_noi_real{5,i},total_duration_dep_sem_noi_real(1,i) ] = processOneFile_real( i_filename_dep_sem_noi_real );
end

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
    avgCmdvelMindist(cell_avg_dep_noi_sim,cell_avg_dep_sem_noi_sim,cell_avg_rgb_sim,number_of_files_sim,false);
% real world
[avg_total_dep_noi_real,avg_total_dep_sem_noi_real,avg_total_rgb_real] = ...
    avgCmdvelMindist(cell_avg_dep_noi_real,cell_avg_dep_sem_noi_real,cell_avg_rgb_real,number_of_files_real,true);

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
