function plotter_cm(HT, show_label)
        
        % % Visualize the cage and its respective coordinate frames
        tOff_X=-0.5;
        tOff_Y=0.005;
        tOff_Z=0.005;
        aLength=0.05;
        font_size = 9;
        
        grid on
        hold on
        
        axisX_0=[aLength;0;0];
        axisY_0=[0;aLength;0];
        axisZ_0=[0;0;aLength];
        
        O_0=[0;0;0];
        
        [~,~,n]=size(HT);
        
        for i=1:n
            % % Visualize the Origin Oi_0
            Oi_0(:,i)=HT(:,:,i)*[O_0;1];
            axisX1_0=HT(:,:,i)*[axisX_0;1];
            axisY1_0=HT(:,:,i)*[axisY_0;1];
            axisZ1_0=HT(:,:,i)*[axisZ_0;1];
            
            % Origin CF1
            color = 'r'; % 红色
            plot3(Oi_0(1,i),Oi_0(2,i),Oi_0(3,i), 'k .','MarkerSize',15, 'color', color)
            if show_label == true
                text(Oi_0(1,i)+tOff_X,Oi_0(2,i)+tOff_Y,Oi_0(3,i)+tOff_Z, ['cm_' num2str(i)], 'FontSize', font_size, 'color', color);
            end
            % %Plot x-axis
            % plot3([Oi_0(1,i);axisX1_0(1)],[Oi_0(2,i);axisX1_0(2)],[Oi_0(3,i);axisX1_0(3)],'r -', 'Linewidth',2)
            % %Plot y-axis
            % plot3([Oi_0(1,i);axisY1_0(1)],[Oi_0(2,i);axisY1_0(2)],[Oi_0(3,i);axisY1_0(3)],'g -', 'Linewidth',2)
            % %Plot z-axis
            % plot3([Oi_0(1,i);axisZ1_0(1)],[Oi_0(2,i);axisZ1_0(2)],[Oi_0(3,i);axisZ1_0(3)],'b -', 'Linewidth',2)
        end

end