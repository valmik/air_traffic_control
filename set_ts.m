function [ Ts ] = set_ts( params, N, index )
%UNTITLED2 Summary of this function goes here
%   sets timestep based on the distance of the plane from the origin and
%   its current speed

plane = params.aircraft_list(index);
state = plane.state;
pos = state(1:2,1);
dpos = state(3:4,1);
dist = sqrt(pos(1)^2 + pos(2)^2);
vel = sqrt(dpos(1)^2 + dpos(2)^2);

time = dist / vel;
Ts = round(time / N, 2);

end

