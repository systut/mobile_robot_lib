step = 100;
x=zeros(1,step);
y=linspace(0,10,step);
theta=linspace(0,360,step);
sim_ouput = cat(1,x,y,theta)

model_p = [2 2 4]';

Execute(sim_ouput,model_p)

function Execute(sim_output,model_p)
    figure;
    hold on
    box on
    grid on
    axis equal
    nt = size(sim_output,2);
    L = model_p(1);  % length
    R = model_p(2);  %wheel readius
    W = model_p(3);  % width
    for i = 1:nt
        
        x=sim_output(1,i);
        y=sim_output(2,i);
        theta=sim_output(3,i);
        %Create rectangle represented for robot chassis
        chassis_x = [x-W/2 , x+W/2, x+W/2, x-W/2];
        chassis_y = [y+L/2 , y+L/2 , y-L/2, y-L/2];
        if i==1
            %Initial state
            axis([x-10,x+10,y-10,y+10]);
            %robot
            robot = fill(chassis_x,chassis_y,'b');
            rotate(robot,[0,0,1],theta,[x,y,0]);
            %x,y axes
            x_ax = line([x,x+W],[y,y],'Color','red','LineStyle','--','LineWidth',1);
            rotate(x_ax,[0,0,1],theta,[x,y,0]);
            y_ax = line([x,x],[y,y+W],'Color','green','LineStyle','--','LineWidth',1);
            rotate(y_ax,[0,0,1],theta,[x,y,0]);
           
        else
            %robot
            set(robot,'XData',chassis_x,'YData',chassis_y);
            rotate(robot,[0,0,1],theta,[x,y,0]);
            %x,y axes
            set(x_ax,'XData',[x,x+W],'YData',[y,y]);
            rotate(x_ax,[0,0,1],theta,[x,y,0]);
            set(y_ax,'XData',[x,x],'YData',[y,y+W]);
            rotate(y_ax,[0,0,1],theta,[x,y,0]);
        end
        pause(0.2);
        drawnow
        
    end
end

