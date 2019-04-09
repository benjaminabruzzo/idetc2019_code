function wV = wedgevector(Vt)
%% V= vector4x1
V = Vt';
wV = [0      V(3,1) -V(2,1); ...
     -V(3,1) 0       V(1,1); ...
     V(2,1) -V(1,1)  0];
end
