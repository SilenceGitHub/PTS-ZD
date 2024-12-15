function EEFPosition=Position(obj, angle)

% DH parameter of the robotic arm
alpha_i = obj.alpha_i;
a_i = obj.a_i;
d_i = obj.d_i;
theta = angle(4:obj.inputNumber);

% Parameters of the mobile platform
d = obj.d;
r = obj.r;
b = obj.b;

xc = angle(1);
yc = angle(2);
phi = angle(3);

% platform 
T_trans = [1,0,0,xc; 0,1,0,yc; 0,0,1,0; 0,0,0,1];
T_rot = obj.TM(phi, 0, 0, 0);
% robotic arm
T01=obj.TM(0,d_i(1),a_i(1),alpha_i(1));
T12=obj.TM(theta(1),d_i(2),a_i(2),alpha_i(2));
T23=obj.TM(theta(2),d_i(3),a_i(3),alpha_i(3));
T34=obj.TM(theta(3),d_i(4),a_i(4),alpha_i(4));
Tw1 = T_trans * T_rot * T01;
Tw2=Tw1*T12;
Tw3=Tw2*T23;
Tw4=Tw3*T34;

pos = Tw4 * [0; 0; 0; 1]; 

EEFPosition = pos(1:3);
