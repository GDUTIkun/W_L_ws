function [position, velocity, acceleration] = wheel_position_governor_step( ...
        position, velocity, target, dt, config)
%WHEEL_POSITION_GOVERNOR_STEP Advance one bounded second-order governor step.

omega = 2 * pi * config.frequencyHz;
acceleration = clamp(omega^2 * (target - position) ...
    - 2 * config.damping * omega * velocity, ...
    -config.accelerationMax, config.accelerationMax);
velocityPrevious = velocity;
velocity = clamp(velocity + dt * acceleration, ...
    -config.velocityMax, config.velocityMax);
candidate = position + dt * velocity;
position = clamp(candidate, config.positionMin, config.positionMax);
if position ~= candidate
    velocity = 0;
end
acceleration = (velocity - velocityPrevious) / dt;
end

function y = clamp(x, lowerBound, upperBound)
y = min(max(x, lowerBound), upperBound);
end
