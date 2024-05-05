function [gaitname, landHeight, jumpVel, jumpAngle, walkVel] = gaitScheduler_obstacle(X, pf, t)
persistent jumpStart;

if isempty(jumpStart)
    jumpStart = 0;
end
% landHeight return variable only needed for landing
landHeight = 0;
% trot until reaching certain X, do jump, then continue trotting
jumpAngle = 0;
jumpVel = [0;0;0];

walkVel = 0;

if X(1) < 0.5
    % trot up to first obstacle
    if t < 0.65
        gaitname = "standing";
    else
        gaitname = "trotting";
        walkVel = 0.2;
    end
elseif X(1) < 6.5
    % jumping over first obstacle and trotting to last obstacle
    
    if jumpStart == 0
        jumpStart = t;
    end
    
    if t-jumpStart < 0.65
        gaitname = "trotting";
        walkVel = 0;
    elseif t-jumpStart < 0.8
        gaitname = "jumpingg";
        jumpVel = [2;0;3];
        jumpAngle = -pi/4;
    elseif t-jumpStart < 1
        gaitname = "soaringg";
    elseif t-jumpStart < 1.7
        gaitname = "landingg";
        landHeight = 0.4;
    elseif t-jumpStart < 1.85
        gaitname = "jumpingg";
        jumpVel = [2;0;2];
        jumpAngle = 0;
    elseif t-jumpStart < 2.2
        gaitname = "soaringg";
    elseif t-jumpStart < 2.6
        gaitname = "landingg";
        landHeight = 0.2;
    else
        gaitname = "trotting";
    end
else
    % jumping over final obstacle
    if jumpStart < 4
        jumpStart = t;
    end
    
    if t-jumpStart < 0.65
        gaitname = "standing";
    elseif t-jumpStart < 0.8
        gaitname = "jumpingg";
        jumpVel = [2;0;3];
        jumpAngle = -pi/4;
    elseif t-jumpStart < 1
        gaitname = "soaringg";
    elseif t-jumpStart < 1.7
        gaitname = "landingg";
        landHeight = 0.4;
    elseif t-jumpStart < 1.85
        gaitname = "jumpingg";
        jumpVel = [2;0;2];
        jumpAngle = 0;
    elseif t-jumpStart < 2.2
        gaitname = "soaringg";
    elseif t-jumpStart < 2.6
        gaitname = "landingg";
        landHeight = 0.2;
    else
        gaitname = "trotting";
    end
end

end