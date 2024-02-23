syms qpp1 qpp2 qpp3 real
syms qpp1r qpp2r qpp3r real
syms qp1r qp1 q1 real

currentElement = qpp1 + qpp1r + qp1r*q1 + qp1
currentElement = char(currentElement);

currentElement = replaceIndexedTerms(currentElement);

currentElement = replacePattern(currentElement, 'q(\d+)', 'q');
currentElement = replacePattern(currentElement, 'qp(\d+)', 'qp');
currentElement = replacePattern(currentElement, 'qpp(\d+)', 'qpp');

currentElement



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

function outStr = replaceIndexedTerms(inputStr)
    patterns = {'qpp(\d+)r', 'qrpp($1-1)'; 'qp(\d+)r', 'qrp($1-1)'; 'q(\d+)', 'q($1-1)'};
    for i = 1:size(patterns, 1)
        pattern = patterns{i, 1};
        replacementFormat = patterns{i, 2};
        [startIndex, endIndex, tokens] = regexp(inputStr, pattern, 'start', 'end', 'tokens');
        for j = length(startIndex):-1:1
            token = tokens{j}{1};
            index = str2double(token) - 1; % 计算新的索引值
            replacement = regexprep(replacementFormat, '\$(\d+)-1', sprintf('%d', index)); % 替换索引
            inputStr = [inputStr(1:startIndex(j)-1), replacement, inputStr(endIndex(j)+1:end)];
        end
    end
    outStr = inputStr;
end