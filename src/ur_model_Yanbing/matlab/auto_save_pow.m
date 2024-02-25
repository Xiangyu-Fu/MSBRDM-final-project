%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 请先运行其他文件
% 保证Transformation Matrix, M, C, G, Yr, Theta都存在后
% 再运行此文件
% 
% 请在下方设置你想要保存的数组
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Transformation Matrix

skipZeros = false;

folderPath = './save_Matrix/Transformation_Matrix';
if ~exist(folderPath, 'dir')
    mkdir(folderPath);
end

for i = 1:6
    matrixName = sprintf('T%d_0', i);
    fileName = sprintf('%s.txt', matrixName);
    % 动态引用矩阵变量并传递给save_matrix函数
    eval(sprintf('save_matrix(%s, matrixName, fileName, folderPath, skipZeros);', matrixName));
end

for i = 1:6
    matrixName = sprintf('Tcm%d_0', i);
    fileName = sprintf('%s.txt', matrixName);
    eval(sprintf('save_matrix(%s, matrixName, fileName, folderPath, skipZeros);', matrixName));
end

%% Jacobian

skipZeros = true;

folderPath = './save_Matrix/Jacobian';
if ~exist(folderPath, 'dir')
    mkdir(folderPath);
end

for i = 1:6
    matrixName = sprintf('J%d_0', i);
    fileName = sprintf('%s.txt', matrixName);
    eval(sprintf('save_matrix(%s, matrixName, fileName, folderPath, skipZeros);', matrixName));
end

for i = 1:6
    matrixName = sprintf('Jcm%d_0', i);
    fileName = sprintf('%s.txt', matrixName);
    eval(sprintf('save_matrix(%s, matrixName, fileName, folderPath, skipZeros);', matrixName));
end

%% Jacobian_dot

skipZeros = true;

folderPath = './save_Matrix/Jacobian_dot';
if ~exist(folderPath, 'dir')
    mkdir(folderPath);
end

for i = 1:6
    matrixName = sprintf('J%d_0_dot', i);
    fileName = sprintf('%s.txt', matrixName);
    eval(sprintf('save_matrix(%s, matrixName, fileName, folderPath, skipZeros);', matrixName));
end

for i = 1:6
    matrixName = sprintf('Jcm%d_0_dot', i);
    fileName = sprintf('%s.txt', matrixName);
    eval(sprintf('save_matrix(%s, matrixName, fileName, folderPath, skipZeros);', matrixName));
end

%% M, C, G

skipZeros = true;

folderPath = './save_Matrix/Regressor';
if ~exist(folderPath, 'dir')
    mkdir(folderPath);
end

load('M.mat');
load('C.mat');
load('G.mat');

matrixName = 'M';
fileName = sprintf('%s.txt', matrixName);
eval(sprintf('save_matrix(%s, matrixName, fileName, folderPath, skipZeros);', matrixName));

matrixName = 'C';
fileName = sprintf('%s.txt', matrixName);
eval(sprintf('save_matrix(%s, matrixName, fileName, folderPath, skipZeros);', matrixName));

matrixName = 'G';
fileName = sprintf('%s.txt', matrixName);
eval(sprintf('save_matrix(%s, matrixName, fileName, folderPath, skipZeros);', matrixName));

%% Regressor

skipZeros = true;

folderPath = './save_Matrix/Regressor';
if ~exist(folderPath, 'dir')
    mkdir(folderPath);
end

load('Theta.mat');
load('Yr.mat');
load('Y.mat');

matrixName = 'Theta';
fileName = sprintf('%s.txt', matrixName);
eval(sprintf('save_matrix(%s, matrixName, fileName, folderPath, skipZeros);', matrixName));

matrixName = 'Yr';
fileName = sprintf('%s.txt', matrixName);
eval(sprintf('save_matrix(%s, matrixName, fileName, folderPath, skipZeros);', matrixName));

matrixName = 'Y';
fileName = sprintf('%s.txt', matrixName);
eval(sprintf('save_matrix(%s, matrixName, fileName, folderPath, skipZeros);', matrixName));

%% Test
syms qpp1 qpp2 qpp3 real
syms qpp1r qpp2r qpp3r real
syms qp1r qp1 q1 real
syms L1 L2 L3 real

currentElement = qpp1^2 + qpp1r*L3^2 + qp1r*q1 + cos(qp1)^2 
currentElement = char(currentElement);

currentElement = replaceIndexedTerms(currentElement);
currentElement = replacePattern(currentElement, 'q(\d+)', 'q');
currentElement = replacePattern(currentElement, 'qp(\d+)', 'qp');
currentElement = replacePattern(currentElement, 'qpp(\d+)', 'qpp');
currentElement = replacePowerOperation(currentElement);

currentElement




%% Function

function save_matrix(matrix, matrixName, fileName, folderPath, skipZeros)

    fullPath = fullfile(folderPath, fileName);
    fileID = fopen(fullPath, 'w');
    if fileID == -1
        error('File cannot be opened.');
    end

    for i = 1:size(matrix, 1)
        for j = 1:size(matrix, 2)

            currentElement = matrix(i, j);
            currentElement = char(currentElement);
            
            % 解决qpp(i)r和qp(i)r
            currentElement = replaceIndexedTerms(currentElement);
            % 解决q(i),qp(i),qpp(i)
            currentElement = replacePattern(currentElement, 'q(\d+)', 'q');
            currentElement = replacePattern(currentElement, 'qp(\d+)', 'qp');
            currentElement = replacePattern(currentElement, 'qpp(\d+)', 'qpp');

            % 替换幂运算
            currentElement = replacePowerOperation(currentElement);
            
            % 跳过值为0的元素
            if skipZeros && strcmp(currentElement, '0')
                continue;
            end
            
            % str = sprintf('T1(%d,%d) = %s;', i-1, j-1, currentElement);
            str = sprintf('%s(%d,%d) = %s;', matrixName, i-1, j-1, currentElement);
            fprintf(fileID, '%s\n', str);
        end
    end

    fclose(fileID);
    disp(['Matrix ' fileName ' saved to file successfully .']);
end

% 使用正则表达式查找并替代：qi, qpi
function outStr = replacePattern(inputStr, pattern, prefix)

    inputStr = regexprep(inputStr, 'qpp(\d+)r', 'qrpp(\d+-1)');
    inputStr = regexprep(inputStr, 'qp(\d+)r', 'qrp(\d+-1)');

    [startIndex, endIndex, extents, matches] = regexp(inputStr, pattern);
    for k = 1:length(matches)
        match = matches{k};
        numberStr = match(length(prefix)+1:end);
        number = str2double(numberStr) - 1;
        replacement = sprintf('%s(%d)', prefix, number);
        inputStr = regexprep(inputStr, [prefix, numberStr], replacement, 'once');
    end

    outStr = inputStr;
end

% 用于解决qpp(i)r和qp(i)r问题
function outStr = replaceIndexedTerms(inputStr)
    patterns = {'qpp(\d+)r', 'qrpp($1-1)'; 'qp(\d+)r', 'qrp($1-1)'; 'q(\d+)', 'q($1-1)'};
    for i = 1:size(patterns, 1)
        pattern = patterns{i, 1};
        replacementFormat = patterns{i, 2};
        [startIndex, endIndex, tokens] = regexp(inputStr, pattern, 'start', 'end', 'tokens');
        for j = length(startIndex):-1:1
            token = tokens{j}{1};
            index = str2double(token) - 1;
            replacement = regexprep(replacementFormat, '\$(\d+)-1', sprintf('%d', index));
            inputStr = [inputStr(1:startIndex(j)-1), replacement, inputStr(endIndex(j)+1:end)];
        end
    end
    outStr = inputStr;
end


% % 替换幂运算为std::pow
% function outStr = replacePowerOperation(inputStr)
%     pattern = '([\w\(\)\d]+)\^(\d+\.?\d*)'; % 匹配形如 x(i)^2, L5^2, cos(q(4))^2 的模式
%     replaceFormat = 'std::pow($1, $2)'; % 替换格式为 std::pow(x(i), 2), std::pow(L5, 2), std::pow(cos(q(4)), 2)
%     outStr = regexprep(inputStr, pattern, replaceFormat);
% end

% 将幂运算简单替换为重复的乘法形式
function outStr = replacePowerOperation(inputStr)
    outStr = inputStr;
    pattern = '([\w\(\)\d]+)\^(\d+)'; % 匹配形如 x(i)^2, L5^2, cos(q(4))^2 的模式
    
    [startIndex, endIndex, tokens] = regexp(outStr, pattern, 'start', 'end', 'tokens');
    
    for i = length(startIndex):-1:1
        base = tokens{i}{1};
        exponent = str2double(tokens{i}{2});
        
        multiplicationStr = base;
        for j = 2:exponent % 从2开始因为已经有一个基数了
            multiplicationStr = [multiplicationStr '*' base];
        end
        
        outStr = [outStr(1:startIndex(i)-1) multiplicationStr outStr(endIndex(i)+1:end)];
    end
end
