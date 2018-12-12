function [postS,flag] = postNL(S,Ts,maxV,minV)
%S: initial set polyhedron
% Ts: timestep
% maxV, minV: min and max velocities

% S.computeVRep();
Sverts = S.V'; % limits of current state
g = 9.81;
xdot = @(x,u) [x(3)*cos(x(4)); x(3)*sin(x(4)); u(1); (g/x(3))*tan(u(2))];
minA = -.5*g;
maxA = g;
maxphi = pi/3; minphi = -maxphi;
% nextVerts = zeros(4,size(Sverts,2)*4);
currSize = 5000;
nextVerts = zeros(4,currSize);
ii = 1;
for i = 1:size(Sverts,2)
    currState = Sverts(:,i);
    for a = [minA maxA]
        for phi = [minphi maxphi]
            nextState = currState + xdot(currState,[a;phi])*Ts;
            if nextState(3) > minV && nextState(3) < maxV
                nextVerts(:,ii) = nextState;
                ii = ii+1;
                if ii > currSize
                    nextVerts = [nextVerts zeros(4,currSize)];
                    currSize = 2*currSize; 
                end
            end
        end
    end
end
% disp(size(nextVerts,2));
nextVerts = nextVerts(:,1:ii-1);

postS = Polyhedron('V',nextVerts');
postS = postS.minVRep(); %get rid of redundant vertices for #performance
verts = postS.V;
maxVals = max(verts(:,1:2),[],1);
minVals = min(verts(:,1:2),[],1);
if all(maxVals > 0) && all(minVals < 0) %checking if limits of xy contain 0 aka is (0,0) in the set
    flag = 1;
else
    flag = 0;
end
% disp(size(postS.V',2));
end
    