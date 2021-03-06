function params = removePlane(params, id )

% Remove from the aircraft list
if isKey(params.aircraft_list, id)
    remove(params.aircraft_list, id);
end

% Remove from the color list
if isKey(params.color_list, id)
    remove(params.color_list, id);
end

% Remove this plane's reference to everything else
if isKey(params.costs, id)
    remove(params.costs, id)
end

% Remove's everything else's reference to the plane
otherPlanes = keys(params.costs);
for key = otherPlanes
    map = params.costs(key{1});
    if isKey(map, id)
        remove(map, id)
    end
end

% Do the same for collision constraints
if isKey(params.collision_constraints, id)
    remove(params.collision_constraints, id)
end

otherPlanes = keys(params.collision_constraints);
for key = otherPlanes
    map = params.collision_constraints(key{1});
    if isKey(map, id)
        remove(map, id)
    end
end

end
