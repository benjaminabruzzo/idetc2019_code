function px = jacobianOfAtan2(a, b)
    A = 1 / (a*a+b*b);
    px = A*[-b a];
end
