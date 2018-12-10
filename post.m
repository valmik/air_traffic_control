function postS = post(A,B,S,U,X,Ts,maxV,minV)
%A,B discrete dynamics
%S: initial set polyhedron
%U: input constraint polyhedron 
%X: state constraint polyhedron
    S.computeVRep();
    U.computeVRep();
    Sverts = S.V';
    Bverts = U.V';
    base = A*Sverts;
%     verts = zeros(size(A,1),size(Sverts,2)*size(Bverts,2));
    verts = [];
    for i = 1:size(base,2)
        for j = 1:size(Bverts,2)
            state = Sverts(:,i) + (A*Sverts(:,i)+B*Bverts(:,j)).*Ts;
            speed = sqrt(sum(state(3:4).^2));
            if speed < maxV && speed > minV
                verts = [verts state];
            end
        end
    end
    postS = Polyhedron('V',verts');
    disp(size(verts,2));
%     out = X.intersect(postS);
    postS = postS.minVRep();
end