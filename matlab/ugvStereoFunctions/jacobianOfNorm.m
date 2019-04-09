function JofNorm = jacobianOfNorm(pt)
    p = pt';
    x = p(1,1);
    y = p(2,1); 
    z = p(3,1);
    A = 1/(x^2+y^2+z^2)^(1.5);
    B = [...
     y*y+z*z,     -x*y,    -x*z;...
        -x*y,  x*x+z*z,    -y*z;...
        -x*z,     -y*z, x*x+y*y];
    JofNorm = A*B;
end