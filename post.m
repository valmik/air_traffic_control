function postS = post(A,B,S,U)
    S.computeVRep();
    U.computeVRep();
    Sverts = S.V';
    Bverts = U.V';
    base = A*Sverts;
    verts = zeros(size(A,1),size(Sverts,2)*size(Bverts,2));
    ii = 1;
    for i = 1:size(base,2)
        for j = 1:size(Bverts,2)
            verts(:,ii) = A*Sverts(:,i)+B*Bverts(:,j);
            ii = ii+1;
        end
    end
    postS = Polyhedron('V',verts');
    postS = postS.minVRep();
end