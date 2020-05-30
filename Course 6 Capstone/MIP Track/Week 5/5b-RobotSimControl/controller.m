
function u = controller(params, t, phi, phidot)
    % STUDENT FILLS THIS OUT

    % Initialize any added state like this:

    persistent newstate t_last
    if isempty(newstate)
        % initialize
        newstate = 0;
        t_last = 0;
    end
    
    dt = t_last - t;
    newstate = newstate + dt*phi;
    t_last = t;

    % Gains
    kp = 10;
    kd = 1;
    ki = -100;

    % Output
    u = kp*phi + kd*phidot + ki*newstate;
end

