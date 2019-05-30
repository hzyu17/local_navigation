clear all
close all
clc

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
cell_avg_dep_noi_sim = cell(2,len_file_dep_sim);   
cell_avg_dep_sem_noi_sim = cell(2,len_file_dep_sem_sim);
cell_avg_rgb_sim = cell(2,len_file_rgb_sim);

% loop on the sim files
for i = 1:11
    i_filename_dep_sim = files_depth_noi_sim(i,:);
    i_filename_dep_sem_sim = files_dep_semantic_noi_sim(i,:);
    i_filename_dep_rgb_sim = files_rgb_sim(i,:);
    [ cell_avg_dep_noi_sim{1,i}, cell_avg_dep_noi_sim{2,i}, cell_avg_dep_noi_sim{3,i} ] = processOneFile_sim( i_filename_dep_sim,'depth');
    [ cell_avg_dep_sem_noi_sim{1,i}, cell_avg_dep_sem_noi_sim{2,i}, cell_avg_dep_sem_noi_sim{3,i} ] = processOneFile_sim( i_filename_dep_sem_sim ,'depth_semantic');
    [ cell_avg_rgb_sim{1,i}, cell_avg_rgb_sim{2,i}, cell_avg_rgb_sim{3,i} ] = processOneFile_sim( i_filename_dep_rgb_sim ,'rgb');
end

% interference or missturns


%% in real scenario
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
cell_avg_dep_noi_real = cell(2,len_file_dep_real);   
cell_avg_dep_sem_noi_real = cell(2,len_file_dep_sem_real);
% ... no rgb data in real scenario tests ...

% loop on the real scenario files
for i = 1:len_file_dep_real
    i_filename_dep_real = files_depth_noi_real(i,:);
    i_filename_dep_sem_real = files_dep_semantic_noi_real(i,:);
    [ cell_avg_dep_noi_real{1,i}, cell_avg_dep_noi_real{2,i}, cell_avg_dep_noi_real{3,i} ] = processOneFile_real( i_filename_dep_real );
    [ cell_avg_dep_sem_noi_real{1,i}, cell_avg_dep_sem_noi_real{2,i}, cell_avg_dep_sem_noi_real{3,i} ] = processOneFile_real( i_filename_dep_sem_real );
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
% map & plot
% h = drawRect(robotPos);
