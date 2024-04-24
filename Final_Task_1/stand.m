function qDes = stand(t)
% qDefault = [0.26; 1.25; -2.7; -0.26; 1.25; -2.7; 0.26; 1.25; -2.7; -0.26; 1.25; -2.7;];
qDefault = [0; 1.25; -2.7; 0; 1.25; -2.7; 0; 1.25; -2.7; 0; 1.25; -2.7;];
qStand = [0; 0.9; -1.7; 0; 0.9; -1.7; 0; 0.9; -1.8; 0; 0.9; -1.7];

if t > 0.1
    qDes = qStand;
else 
    qDes = qDefault;
end

%{
LR Initial State Target
	Thigh: 1.25
	Calf: -2.7
	Hip: 0.26

Desired (might be old though)
	Hip: 0
	Thigh: 0.2
    Calf: -0.2

%}