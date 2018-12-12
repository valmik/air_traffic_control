function [ dist ] = dist_center( params, index )
%UNTITLED Summary of this function goes here
%   Returns the distance of the index'th plane to the center

plane = params.aircraft_list(index);
state = plane.state;
pos = state(1:2,1);
dist = sqrt(pos(1)^2 + pos(2)^2);

end

