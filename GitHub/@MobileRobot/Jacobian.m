function [Jacob]=Jacobian(obj, angle)

% DH parameter of the robotic arm
alpha_i = obj.alpha_i;
a_i = obj.a_i;
d_i = obj.d_i;
theta = angle(3:6);

% Parameters of the mobile platform
d = obj.d;
r = obj.r;
b = obj.b;

phi = (angle(2)-angle(1)) * r / (2*b);

% platform 
% T_trans = [1,0,0,xc; 0,1,0,yc; 0,0,1,0; 0,0,0,1];
T_rot = obj.TM(phi, 0, 0, 0);
% robotic arm
T01=obj.TM(theta(1),d_i(1),a_i(1),alpha_i(1));
T12=obj.TM(theta(2),d_i(2),a_i(2),alpha_i(2));
T23=obj.TM(theta(3),d_i(3),a_i(3),alpha_i(3));
T34=obj.TM(theta(4),d_i(4),a_i(4),alpha_i(4));

dTMdPhi = obj.dTMdTheta(phi,0,0,0);
dTMdTheta01 = obj.dTMdTheta(theta(1),d_i(1),a_i(1),alpha_i(1));
dTMdTheta12 = obj.dTMdTheta(theta(2),d_i(2),a_i(2),alpha_i(2));
dTMdTheta23 = obj.dTMdTheta(theta(3),d_i(3),a_i(3),alpha_i(3));
dTMdTheta34 = obj.dTMdTheta(theta(4),d_i(4),a_i(4),alpha_i(4));

Jec=dTMdPhi*T01*T12*T23*T34*[0;0;0;1];
Je1=T_rot*dTMdTheta01*T12*T23*T34*[0;0;0;1];
Je2=T_rot*T01*dTMdTheta12*T23*T34*[0;0;0;1];
Je3=T_rot*T01*T12*dTMdTheta23*T34*[0;0;0;1];
Je4=T_rot*T01*T12*T23*dTMdTheta34*[0;0;0;1];


J(:,1)=Jec(1:3,1);
J(:,2)=Je1(1:3,1);
J(:,3)=Je2(1:3,1);
J(:,4)=Je3(1:3,1);
J(:,5)=Je4(1:3,1);

A = r/(2*b)*[-1,1];
B = r/2*[cos(phi), -sin(phi); sin(phi), cos(phi)]*[1, 1; -d/b, d/b];

Jacob = [B, zeros(2,4); zeros(1,6)] + J * [A, zeros(1,4); zeros(4,2), eye(4)];

end
