function [gaitname, landHeight, jumpVel, jumpAngle, walkVel, height] = gaitScheduler_obstacle(X, pf, t)
persistent state2_start;
persistent state3_start;
persistent obstacleState;
% 1 - standing and trotting towards first obstacle
% 2 - trot in place, stop, jump over obstacle and trot towards final obstacle

if isempty(obstacleState)
    obstacleState = 1;
end

if isempty(state2_start)
    state2_start = 0;
end

if isempty(state3_start)
    state3_start = 0;
end
% landHeight return variable only needed for landing
landHeight = 0;
% trot until reaching certain X, do jump, then continue trotting
jumpAngle = 0;
jumpVel = [0;0;0];

height = 0;
walkVel = 0;


if X(1) > 0.3%0.7 % right before first obstacle
    obstacleState = 2;
    
    if state2_start == 0 %first iteration that state2 starts
        state2_start = t;
    end
end

if X(1) > 5.7
    obstacleState = 3;
    
    if state3_start == 0  %first iteration that state3 starts
        state3_start = t;
    end
end

% if t < 0.65
%     gaitname = "standing";
%     walkVel = 0;
% elseif t < 0.8
%     gaitname = "jumpingg";
%     jumpVel = [2;0;3];
%     jumpAngle = -pi/4;
% elseif t < 1
%     gaitname = "soaringg";
% elseif t < 1.7
%     gaitname = "landingg";
%     landHeight = 0.4;
% elseif t < 1.85
%     gaitname = "jumpingg";
%     jumpVel = [2;0;2];
%     jumpAngle = 0;
% elseif t < 2.2
%     gaitname = "soaringg";
% elseif t < 2.6
%     gaitname = "landingg";
%     landHeight = 0.2;
% else
%     gaitname = "trotting";
% end


if obstacleState == 1
    % trot up to first obstacle
    if t < 0.65
        gaitname = "standing";
    else
        gaitname = "trotting";
        height = 0.25;
        % walkVel = 0.2;
        walkVel = speed_ramp(t, 0.65, 1, 0, 0.7);
        disp("first trot")
    end
    
elseif obstacleState == 2
    % jumping over first obstacle and trotting to last obstacle
    
    if t-state2_start < 0.45
        gaitname = "trotting";
        height = 0.25;
        walkVel = speed_ramp(t, 0, 0.4, 0.7, 0);
        disp("second trot")
    elseif t-state2_start < 0.6
        gaitname = "trotting";
        height = 0.25;
        walkVel = 0;
        disp("second trot")
    elseif t-state2_start < 0.65
        gaitname = "standing";
    elseif t-state2_start < 0.8
        gaitname = "jumpingg";
        jumpVel = [2;0;3];
        jumpAngle = -pi/8; %-pi/4;
    elseif t-state2_start < 1.15
        gaitname = "soaringg";
    elseif t-state2_start < 2
        gaitname = "landingg";
        landHeight = 0.4;
    elseif t-state2_start < 2.15
        gaitname = "jumpingg";
        jumpVel = [2;0;3];
        jumpAngle = 0;
    elseif t-state2_start < 2.4
        gaitname = "soaringg";
    elseif t-state2_start < 3
        gaitname = "landingg";
        landHeight = 0.2;
    else
        gaitname = "trotting";
        height = 0.25;
        walkVel = speed_ramp(t-state2_start, 3, 4, 0, 1.5);
        % walkVel = 0.5;
    end
else
    % jumping over final obstacle (state 3)
    deccelerate = 1;
    trotInPlace = 0.45;
    standing = 0.3;
    jumping = 0.15;
    soaring = 0.45;
    landing = 0.5;
    
    if t-state3_start < deccelerate
        gaitname = "trotting";
        height = 0.25;
        walkVel = 1.5 + speed_ramp(t-state3_start, 0, 1, 0, -1.5);
        t-state3_start;
        disp("second trot")
    elseif t-state3_start < deccelerate + trotInPlace
        gaitname = "trotting";
        height = 0.25;
        walkVel = 0;
        disp("second trot")
    elseif t-state3_start < deccelerate + trotInPlace + standing
        gaitname = "standing";
    elseif t-state3_start < deccelerate + trotInPlace + standing + jumping
        gaitname = "jumpingg";
        jumpVel = [2;0;4];
        jumpAngle = -pi/2; %-pi/4;
    elseif t-state3_start < deccelerate + trotInPlace + standing + jumping + soaring
        gaitname = "soaringg";
    elseif t-state3_start < deccelerate + trotInPlace + standing + jumping + soaring + landing
        gaitname = "landingg";
        landHeight = 0.2;
    else
        gaitname = "trotting";
        height = 0.25;
        walkVel = 0.5;
    end
end

end