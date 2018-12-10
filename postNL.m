function postS = postNL(S,Ts,maxV,minV)

%A,B discrete dynamics
%S: initial set polyhedron
%U: input constraint polyhedron 
%X: state constraint polyhedron

S.computeVRep();
Sverts = S.V'; % limits of current state
g = 9.81;
xdot = @(x,u) [x(3)*cos(x(4)); x(3)*sin(x(4)); u(1); (g/x(3))*tan(u(2))];
minA = -.5*g;
maxA = g;
maxphi = pi/3; minphi = -maxphi;
% nextVerts = zeros(4,size(Sverts,2)*4);
nextVerts = [];
% ii = 1;
for i = 1:size(Sverts,2)
    currState = Sverts(:,i);
    for a = [minA maxA]
        for phi = [minphi maxphi]
            nextState = currState + xdot(currState,[a;phi])*Ts;
            if nextState(3) > minV && nextState(3) < maxV
                nextVerts = [nextVerts nextState];
            end
        end
    end
end
disp(size(nextVerts,2));
postS = Polyhedron('V',nextVerts');
postS = postS.minVRep();
disp(size(postS.V',2));
end
    
    
    %nonlinear state [x y v psi]
    %nonlinear input [accel(long) phi]
    
%     svert2nlstate = @(s) [s(1); s(2); sqrt(sum(s(3:4).^2)); atan2(s(4),s(3))];
%     verts = [];
%     for i = 1:size(base,2)
%         for j = 1:size(Bverts,2)
%             state = Sverts(:,i) + (A*Sverts(:,i)+B*Bverts(:,j)).*Ts;
%             speed = sqrt(sum(state(3:4).^2));
%             if speed < maxV && speed > minV
%                 verts = [verts state];
%             end
%         end
%     end
%     postS = Polyhedron('V',verts');
%     disp(size(verts,2));
% %     out = X.intersect(postS);
%     postS = postS.minVRep();
% end