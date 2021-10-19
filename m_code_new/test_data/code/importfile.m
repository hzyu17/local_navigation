function routeCommand = importfile(filename, startRow, endRow)
%IMPORTFILE ���ı��ļ��е���ֵ������Ϊ�����롣
%   ROUTECOMMAND20190513175427 = IMPORTFILE(FILENAME) ��ȡ�ı��ļ� FILENAME
%   ��Ĭ��ѡ����Χ�����ݡ�
%
%   ROUTECOMMAND20190513175427 = IMPORTFILE(FILENAME, STARTROW, ENDROW)
%   ��ȡ�ı��ļ� FILENAME �� STARTROW �е� ENDROW ���е����ݡ�
%
% Example:
%   routeCommand20190513175427 = importfile('routeCommand_2019_05_13_17_54_27.csv', 1, 3554);
%
%    ������� TEXTSCAN��

% �� MATLAB �Զ������� 2019/05/25 10:19:18

%% ��ʼ��������
delimiter = ',';
if nargin<=2
    startRow = 1;
    endRow = inf;
end

%% ÿ���ı��еĸ�ʽ:
%   ��1: ˫����ֵ (%f)
%	��2: ˫����ֵ (%f)
%   ��3: ˫����ֵ (%f)
%	��4: ˫����ֵ (%f)
%   ��5: ˫����ֵ (%f)
%	��6: ˫����ֵ (%f)
%   ��7: ˫����ֵ (%f)
%	��8: ˫����ֵ (%f)
%   ��9: ˫����ֵ (%f)
%	��10: ˫����ֵ (%f)
%   ��11: ˫����ֵ (%f)
%	��12: ˫����ֵ (%f)
%   ��13: ˫����ֵ (%f)
%	��14: ˫����ֵ (%f)
%   ��15: ˫����ֵ (%f)
%	��16: ˫����ֵ (%f)
%   ��17: ˫����ֵ (%f)
%	��18: ˫����ֵ (%f)
%   ��19: ˫����ֵ (%f)
%	��20: ˫����ֵ (%f)
%   ��21: ˫����ֵ (%f)
%	��22: ˫����ֵ (%f)
%   ��23: ˫����ֵ (%f)
%	��24: ˫����ֵ (%f)
%   ��25: ˫����ֵ (%f)
%	��26: ˫����ֵ (%f)
%   ��27: ˫����ֵ (%f)
%	��28: ˫����ֵ (%f)
%   ��29: ˫����ֵ (%f)
%	��30: ˫����ֵ (%f)
%   ��31: ˫����ֵ (%f)
%	��32: ˫����ֵ (%f)
% �й���ϸ��Ϣ������� TEXTSCAN �ĵ���
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% ���ı��ļ���
fileID = fopen(filename,'r');

%% ���ݸ�ʽ��ȡ�����С�
% �õ��û������ɴ˴������õ��ļ��Ľṹ����������ļ����ִ����볢��ͨ�����빤���������ɴ��롣
dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', delimiter, 'TextType', 'string', 'EmptyValue', NaN, 'HeaderLines', startRow(1)-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
for block=2:length(startRow)
    frewind(fileID);
    dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', delimiter, 'TextType', 'string', 'EmptyValue', NaN, 'HeaderLines', startRow(block)-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
    for col=1:length(dataArray)
        dataArray{col} = [dataArray{col};dataArrayBlock{col}];
    end
end

%% �ر��ı��ļ���
fclose(fileID);

%% ���޷���������ݽ��еĺ���
% �ڵ��������δӦ���޷���������ݵĹ�����˲�����������롣Ҫ�����������޷���������ݵĴ��룬�����ļ���ѡ���޷������Ԫ����Ȼ���������ɽű���

%% �����������
routeCommand = table2array(table(dataArray{1:end-1}, 'VariableNames', {'VarName1','VarName2','VarName3','VarName4','VarName5','VarName6','VarName7','VarName8','nan','nan1','VarName11','VarName12','VarName13','VarName14','VarName15','VarName16','VarName17','VarName18','VarName19','VarName20','VarName21','VarName22','VarName23','VarName24','VarName25','VarName26','VarName27','VarName28','VarName29','VarName30','VarName31','VarName32'}));

