function [currcontact, ftcontacts] = project_gait(t,N,dt, gaitperiod, gaitname)
persistent currgait
persistent t_gaitchange

if isempty(currgait)
    currgait = "standing";
end

if isempty(t_gaitchange)
    t_gaitchange = 0;
end

gait_ref = [0;0;0;0];
if isequal(gaitname, "standing")
    gait_ref = [1; 1; 1; 1];
elseif isequal(gaitname, "trotting")
    gait_ref = [1; 0; 0; 1; 0; 1; 1; 0];
elseif isequal(gaitname, "bounding")
    gait_ref = [1; 1; 0; 0; 0; 0; 1; 1];
elseif isequal(gaitname, "singleFt")
    gait_ref = [0; 1; 1; 1;
        1; 1; 1; 0;
        1; 0; 1; 1;
        1; 1; 0; 1];
elseif isequal(gaitname, "flyingtr")
    gait_ref = [1; 0; 0; 1; 0; 0; 0; 0; 0; 1; 1; 0; 0; 0; 0; 0];
elseif isequal(gaitname, "jumpingg")
    gait_ref = [1; 1; 1; 1];
elseif isequal(gaitname, "soaringg")
    gait_ref = [0; 0; 0; 0];
elseif isequal(gaitname, "landingg")
    gait_ref = [1; 1; 1; 1];
end

gait_states = length(gait_ref)/4;

if ~isequal(currgait, gaitname)
    t_gaitchange = t;
end

gait_t = t - t_gaitchange;

% Default contact on all feet if last known state was standing
if isequal(currgait, "standing")
    currcontact = [1; 1; 1; 1];
elseif isequal(currgait, "jumpingg") % If last known state was jumping
    currcontact = [1; 1; 1; 1];
elseif isequal(currgait,"landingg") % If last known state was landing
    currcontact = [1; 1; 1; 1];
elseif isequal(currgait, "soaringg") % If last known state was soaring
    currcontact = [0; 0; 0; 0];
else
    % Current gait state based on current time
    curr_state = floor(gait_t/gaitperiod) + 1;
    % If at the gait switch time, current state defaults to state before
    % switching
    
    if gait_t > 0
        if mod(gait_t, gaitperiod) == 0
            curr_state = curr_state - 1;
        end
    end
    % Current gait state starting index in gait_ref
    curr_startind = (curr_state - 1)*4 + 1;
    
    % If current start index is greater than the gait_ref given, loop around
    % and update gait_ref
    if curr_startind > gait_states*4
        curr_startind = mod(curr_startind, gait_states*4);
    end
    
    
    % if curr_startind < 1
    %     currcontact = [1;1;1;1];
    % else
    currcontact = gait_ref(curr_startind:curr_startind+3);
    
end

% Repeat above process for N times to get future foot contacts
ftcontacts = zeros(4*N,1);
for ind = 1:N
    temp_time = gait_t + ind*dt;
    temp_state = floor(temp_time/gaitperiod) + 1;
    if mod(temp_time, gaitperiod) == 0
        temp_state = temp_state - 1;
    end
    
    temp_startind = (temp_state - 1)*4 + 1;
    if temp_startind > gait_states*4
        temp_startind = mod(temp_startind, gait_states*4);
    end
    ftcontacts((ind-1)*4+1:(ind-1)*4 + 4) = gait_ref(temp_startind:temp_startind+3);
end

currgait = gaitname;
end