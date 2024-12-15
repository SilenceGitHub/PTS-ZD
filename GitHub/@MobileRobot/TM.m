function T = TM(theta,d,a,alpha)
% Transformation matrix
% standard DH
    T=[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
        sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
        0,sin(alpha),cos(alpha),d;
        0,0,0,1];

% % modified DH
% T=[cos(theta),-sin(theta),0,a;
%    sin(theta)*cos(alpha),cos(theta)*cos(alpha),-sin(alpha),-d*sin(alpha);
%    sin(theta)*sin(alpha),cos(theta)*sin(alpha),cos(alpha),d*cos(alpha);
%    0,0,0,1];
end