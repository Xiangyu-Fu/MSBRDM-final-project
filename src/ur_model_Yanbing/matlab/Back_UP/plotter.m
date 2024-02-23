function plotter(HT, varargin)
    joint = true;
    projection = true;
    base_str = 'O_';
    aLength = 0.2; % 修改轴的长度
    
    if nargin == 2
        joint = varargin{1};
    elseif nargin == 3
        joint = varargin{1};
        projection = varargin{2};
    elseif nargin == 4
        joint = varargin{1};
        projection = varargin{2};
        base_str = varargin{3};
    elseif nargin == 5
        joint = varargin{1};
        projection = varargin{2};
        base_str = varargin{3};
        color = varargin{4};
    elseif nargin == 6
        joint = varargin{1};
        projection = varargin{2};
        base_str = varargin{3};
        color = varargin{4};
        aLength = varargin{5};
    end

     %% Setup
    grid on
    hold on
    
    n = size(HT, 3);
    tOff = 0.005;
    tOff_X = 0.1;
    
    axisX_0=[aLength;0;0;1];
    axisY_0=[0;aLength;0;1];
    axisZ_0=[0;0;aLength;1];
    
    % origins
    O = zeros(4, (n+1)*2);
    
    % axis lines
    axisX = zeros(4,(n+1)*2);
    axisY = zeros(4,(n+1)*2);
    axisZ = zeros(4,(n+1)*2);
    
    %% Compute axis and frame names
    
    % World center
    str = {'O_W'};

    axisX(:,2) = axisX_0;
    axisY(:,2) = axisY_0;
    axisZ(:,2) = axisZ_0;
    
    % frames
    j = 3;
    for k = 1 : n
        % str{end+1} = [base_str num2str(k-2)];
        str{end+1} = [base_str num2str(k)];
        
        O(:,j) = HT(:,4,k);
        O(:,j+1) = HT(:,4,k);
        
        axisX(:,j) = HT(:,4,k);
        axisY(:,j) = HT(:,4,k);
        axisZ(:,j) = HT(:,4,k);
        
        axisX(:,j+1) = HT(:,:,k)*axisX_0;
        axisY(:,j+1) = HT(:,:,k)*axisY_0;
        axisZ(:,j+1) = HT(:,:,k)*axisZ_0;
        
        j = j + 2;
    end
    
    %% Plot all coordinate frames at once
    hold on;
    plot3(reshape(axisX(1,:)',2,[]), reshape(axisX(2,:)',2,[]), reshape(axisX(3,:)',2,[]), 'r -', 'Linewidth',2)
    plot3(reshape(axisY(1,:)',2,[]), reshape(axisY(2,:)',2,[]), reshape(axisY(3,:)',2,[]), 'g -', 'Linewidth',2)
    plot3(reshape(axisZ(1,:)',2,[]), reshape(axisZ(2,:)',2,[]), reshape(axisZ(3,:)',2,[]), 'b -', 'Linewidth',2)
    text(O(1,1:2:end)'+tOff_X,O(2,1:2:end)'+tOff,O(3,1:2:end)'+tOff,str, 'color', 'b');
    
    % %% Plot robot links
    % 用直线链接点（O_W和O_0没连上）
    % linksX = reshape(O(1,4:end-1)',2,[]);
    % linksY = reshape(O(2,4:end-1)',2,[]);
    % linksZ = reshape(O(3,4:end-1)',2,[]);
    % 
    % if(joint == true)
    %     plot3(linksX, linksY, linksZ, 'k-', 'Linewidth', 1.5) % 这里使用 'k-' 表示黑色的实线
    % end
    % 用直线链接点
    for i = 1:size(O,2)-1
        plot3([O(1,i) O(1,i+1)], [O(2,i) O(2,i+1)], [O(3,i) O(3,i+1)], 'k-', 'Linewidth', 1.5);
    end


    %% Plot Origins
    % MarkerSize 修改点的大小
    color = 'b'; % 蓝色
    plot3(O(1,:), O(2,:), O(3,:), 'k .','MarkerSize',20, 'color', color);
end