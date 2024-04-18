clc;

t = 0;
N = 4;
dt = 0.01;
gaitperiod = 0.02;
gaitname = 'trotting';

%mpcTable = mohsen_gait(t,N,dt, gaitname)
[currcontact, ftcontacts] = project_gait(t,N,dt, gaitperiod, gaitname)

function [currcontact, ftcontacts] = project_gait(t,N,dt, gaitperiod, gaitname)
if isequal(gaitname, 'standing')
    gait_ref = [1; 1; 1; 1];
elseif isequal(gaitname, 'trotting')
    gait_ref = [1; 0; 0; 1; 0; 1; 1; 0];
elseif isequal(gaitname, 'bounding')
    gait_ref = [1; 1; 0; 0; 0; 0; 1; 1];
end
gait_states = length(gait_ref)/4;

% Default contact on all feet if t = 0
if t == 0
    currcontact = [1; 1; 1; 1];
else
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
end

% Repeat above process for N times to get future foot contacts
ftcontacts = [];
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
    ftcontacts = [ftcontacts; gait_ref(temp_startind:temp_startind+3)];
end
end

% function mpcTable = mohsen_gait(t,N,dt, gaitname)
% if isequal(gaitname, 'standing')
%     offsets = [0,0,0,0];
%     duration = [N,N,N,N];
% elseif isequal(gaitname, 'trotting')
%     offsets = [0,N/2,N/2,0];
%     duration = [N/2,N/2,N/2,N/2];
% elseif isequal(gaitname, 'bounding')
%     offsets = [N/2,N/2,0,0];
%     duration = [N/2,N/2,N/2,N/2];
% end
% 
% mpcTable = zeros(N*4,1);
% iteration = floor(mod(t/dt,N))
% 
% for i = 0:N-1
%     iter = mod((i +1 + iteration),N);
%     progress = iter - offsets;
% 
%     for j = 1:4
%         if progress(j) < 0
%             progress(j) = progress(j) + N;
%         end
%         if progress(j) < duration(j)
%             mpcTable(i*4+j) = 1;
%         else
%             mpcTable(i*4+j) = 0;
%         end    
%     end
% end
% end