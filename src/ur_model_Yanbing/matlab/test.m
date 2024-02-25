%% Test
syms qpp1 qpp2 qpp3 real
syms qpp1r qpp2r qpp3r real
syms qp1r qp1 q1 real
syms L1 L2 L3 real
syms m1 m2 m3 real

% currentElement = (L2^2*m3)/2 + qpp1r*(L3^2+L1) + qp1r*q1 + cos(q1)^2 
currentElement = (L2^2*m3)/2 

currentElement = char(currentElement);

% currentElement = replaceCosSin(currentElement);
currentElement = replaceIndexedTerms(currentElement);
currentElement = replacePattern(currentElement, 'q(\d+)', 'q');
currentElement = replacePattern(currentElement, 'qp(\d+)', 'qp');
currentElement = replacePattern(currentElement, 'qpp(\d+)', 'qpp');
currentElement = replacePowerOperation(currentElement);

currentElement


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



%% 
function save_matrix(matrix, matrixName, fileName, folderPath, skipZeros)
    fullPath = fullfile(folderPath, fileName);
    fileID = fopen(fullPath, 'w');
    if fileID == -1
        error('File cannot be opened.');
    end

    cosSinDeclarations = containers.Map('KeyType', 'char', 'ValueType', 'char');

    % 先不写入文件，收集ci和si
    for i = 1:size(matrix, 1)
        for j = 1:size(matrix, 2)
            currentElement = matrix(i, j);
            currentElement = char(currentElement);

            % 新增替换逻辑
            [currentElement, cosSinDeclarations] = replaceCosSin(currentElement, cosSinDeclarations);

            % 其余处理保持不变
            currentElement = replaceIndexedTerms(currentElement);
            currentElement = replacePattern(currentElement, 'q(\d+)', 'q');
            currentElement = replacePattern(currentElement, 'qp(\d+)', 'qp');
            currentElement = replacePattern(currentElement, 'qpp(\d+)', 'qpp');
            currentElement = replacePowerOperation(currentElement);
            
            if skipZeros && strcmp(currentElement, '0')
                continue;
            end
            
            str = sprintf('%s(%d,%d) = %s;', matrixName, i-1, j-1, currentElement);
            matrixContent{i, j} = str; % 改为先保存内容
        end
    end

    % 写入ci和si声明
    cosSinKeys = cosSinDeclarations.keys;
    for k = 1:length(cosSinKeys)
        fprintf(fileID, '%s\n', cosSinDeclarations(cosSinKeys{k}));
    end

    % 写入矩阵内容
    for i = 1:size(matrixContent, 1)
        for j = 1:size(matrixContent, 2)
            if ~isempty(matrixContent{i, j})
                fprintf(fileID, '%s\n', matrixContent{i, j});
            end
        end
    end

    fclose(fileID);
    disp(['Matrix ' fileName ' saved to file successfully.']);
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

% 将幂运算简单替换为重复的乘法形式
function outStr = replacePowerOperation(inputStr)
    outStr = inputStr;
    pattern = '([\w\(\)\d]+)\^(\d+)'; % 匹配形如 x(i)^2, L5^2, cos(q(4))^2 的模式

    [startIndex, endIndex, tokens] = regexp(outStr, pattern, 'start', 'end', 'tokens');

    for i = length(startIndex):-1:1
        base = tokens{i}{1};
        exponent = str2double(tokens{i}{2});

        % 检查基数中是否有左括号
        if contains(base, '(')
            % 创建一个去除左括号的基数版本
            base_new = replace(base, '(', '');
            % 首次使用原始基数
            multiplicationStr = base;
        else
            base_new = base;
            multiplicationStr = base;
        end

        for j = 2:exponent % 从2开始因为已经有一个基数了
            multiplicationStr = [multiplicationStr '*' base_new];
        end

        outStr = [outStr(1:startIndex(i)-1) multiplicationStr outStr(endIndex(i)+1:end)];
    end
end


% 替换cos(qi)和sin(qi)
function [outputStr, cosSinDeclarations] = replaceCosSin(inputStr, cosSinDeclarations)
    cosPattern = 'cos\(q(\d+)\)';
    sinPattern = 'sin\(q(\d+)\)';
    
    [startIndex, endIndex, tokens] = regexp(inputStr, cosPattern, 'start', 'end', 'tokens');
    for i = 1:length(tokens)
        index = tokens{i}{1};
        cosStr = sprintf('c%s', index);
        cosSinDeclarations(cosStr) = sprintf('cc::Scalar %s = cos(q(%d));', cosStr, str2double(index)-1);
        inputStr = regexprep(inputStr, ['cos\(q', index, '\)'], cosStr, 'once');
    end

    [startIndex, endIndex, tokens] = regexp(inputStr, sinPattern, 'start', 'end', 'tokens');
    for i = 1:length(tokens)
        index = tokens{i}{1};
        sinStr = sprintf('s%s', index);
        cosSinDeclarations(sinStr) = sprintf('cc::Scalar %s = sin(q(%d));', sinStr, str2double(index)-1);
        inputStr = regexprep(inputStr, ['sin\(q', index, '\)'], sinStr, 'once');
    end

    outputStr = inputStr;
end