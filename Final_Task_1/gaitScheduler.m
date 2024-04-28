function gaitname = gaitScheduler(X, pf, t)
    if t < 1
        gaitname = "standing";
    else
        gaitname = "trotting";
        

    end
end