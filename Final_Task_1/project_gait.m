function [currcontact, ftcontacts] = project_gait(t,N,dt, gaitperiod, gaitname)
gait_ref = [0;0;0;0];
if isequal(gaitname, "standing")
    gait_ref = [1; 1; 1; 1];
elseif isequal(gaitname, "trotting")
    gait_ref = [1; 0; 0; 1; 0; 1; 1; 0];
elseif isequal(gaitname, "bounding")
    gait_ref = [1; 1; 0; 0; 0; 0; 1; 1];
end
gait_states = length(gait_ref)/4;

% Default contact on all feet if t = 0
% if t <= 0.3
    % currcontact = [1; 1; 1; 1];
% else
% Current gait state based on current time
curr_state = floor(t/gaitperiod) + 1;
% If at the gait switch time, current state defaults to state before
% switching
if mod(t, gaitperiod) == 0
    curr_state = curr_state - 1;
end

% Current gait state starting index in gait_ref
curr_startind = (curr_state - 1)*4 + 1;
% If current start index is greater than the gait_ref given, loop around
% and update gait_ref
if curr_startind > gait_states*4
    curr_startind = mod(curr_startind, gait_states*4);
end
currcontact = gait_ref(curr_startind:curr_startind+3);
% end

% Repeat above process for N times to get future foot contacts
ftcontacts = zeros(4*N,1);
for ind = 1:N
    temp_time = t + ind*dt;
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
end