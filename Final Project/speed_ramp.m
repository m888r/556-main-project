function interpolatedVelocity = speed_ramp(time, startTime, endTime, startVelocity, endVelocity)
    
    interpolationFactor = (time - startTime) / (endTime - startTime);
    
    interpolatedVelocity = startVelocity + (endVelocity - startVelocity) * interpolationFactor;
    if (abs(interpolatedVelocity) > abs(endVelocity))
        interpolatedVelocity = endVelocity;
    end
    if (abs(interpolatedVelocity) < abs(startVelocity))
        interpolatedVelocity = startVelocity;
    end
end
