function gaitname = gaitScheduler(X, pf, t)

if t < 0.65
    gaitname = "standing";
else
    gaitname = "trotting";
    % gaitname = "singleFt";
    % gaitname = "gallopng";
    % gaitname = "bounding";
    
%end
end