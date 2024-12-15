function y = ACF(x, acf, rho)

switch acf
    case 'linear'
        y = x;
    case 'ptc' % predefined time convergent
    	y = sgn(x,1-rho/2) + sgn(x,1+rho/2);
    case 'ts_ptc' % time-synchronized ptc
        beta = 0.00001; % for stability
    	y = sgn_n(x,1-rho) + sgn_n(x,1+rho) + beta*sgn_n(x,0);
    otherwise
        y = x;
end

end