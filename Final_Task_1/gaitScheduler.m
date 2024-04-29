function gaitname = gaitScheduler(X, pf, t)
if t < 0.9
    gaitname = "standing";
else
    gaitname = "trotting";


end
end