function y = sgn_n(x, rho)
if norm(x,2) < 1e-10
    y=0*x;
else
    y = norm(x,2)^(rho-1 ) * x;
end
end