% This function animates the solution stored in the solver object. 
function Animate(t)
    % Bring window to front:
    figure(obj.fig);
    slowDown = 1;
    % Define the time grid used for the animation
    t = [obj.solver.t_out(1):obj.rate:obj.solver.t_out(end)];
    nt = size(t,2);
    % Interpolate the results from the solver to this time grid:
    q    = interp1(obj.solver.t_out,obj.solver.q_out.',t.').';
    dqdt = interp1(obj.solver.t_out,obj.solver.dqdt_out.',t.').';
    % Run Animation
    tic
    for i = 1:nt
        Update(t(:,i), q(:,i), dqdt(:,i));
        while toc<t(:,i)*slowDown
            pause(0.01);
        end
    end
end

function Update(t, q, dqdt)
    

end