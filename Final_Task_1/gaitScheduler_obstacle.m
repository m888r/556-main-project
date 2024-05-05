function gaitname = gaitScheduler_obstacle(X, pf, t)

if t < 0.65
    gaitname = "standing";
elseif t < 0.8
    gaitname = "jumpingg";
elseif t < 1.15
    gaitname = "soaringg";
else
    gaitname = "standing";
end

end