%% Class definition of a wheeled mobile robot
classdef MobileRobot
    %% Variables
    properties(SetAccess = public)

    end
    properties(SetAccess = private)    

        alpha_i = [];
        a_i = [];
        d_i = [];
        % Parameters of the mobile platform
        d = 0;
        r = 0;
        b = 0;

        inputNumber = 6;
        taskDimension = 3;
    end
    
    %% Costructor 
    methods
        
        function obj = MobileRobot(param)
            obj.alpha_i = param.alpha_i;
            obj.a_i = param.a_i;
            obj.d_i = param.d_i;
            obj.d = param.d;
            obj.r = param.r;
            obj.b = param.b;
        end
    end
    
   %% functions
    methods (Static)
        % get end effector position
        position = Position(obj, angle, xc, yc);
        % Get the position of each wheel and each link
        position = LinkPosition(obj, angle, xc, yc);
        
        % get the whole-body Jacobian
        J = Jacobian(obj, angle);
        
        % get the manipulator Jacobian
        J = Jacobian2(obj, angle);
        
        % get end effector velocity
        velocity = Velocity(obj, angle, angularVelocity)
        
%         % get end effector acceleration
%         ret = GetEEFAcc(J, dJ, jointVel, jointAcc);
        function quadnion = Orientation(obj, angle)
            % DH parameter of the robotic arm
            alpha_i = obj.alpha_i;
            a_i = obj.a_i;
            d_i = obj.d_i;
            theta = angle(4:7);

            xc = angle(1);
            yc = angle(2);
            phi = angle(3);

            % platform 
            T_trans = [1,0,0,xc; 0,1,0,yc; 0,0,1,0; 0,0,0,1];
            T_rot = obj.TM(phi, 0, 0, 0);
            % robotic arm
            T01=obj.TM(theta(1),d_i(1),a_i(1),alpha_i(1));
            T12=obj.TM(theta(2),d_i(2),a_i(2),alpha_i(2));
            T23=obj.TM(theta(3),d_i(3),a_i(3),alpha_i(3));
            T34=obj.TM(theta(4),d_i(4),a_i(4),alpha_i(4));
            Tw1 = T_trans * T_rot * T01;
            Tw2=Tw1*T12;
            Tw3=Tw2*T23;
            Tw4=Tw3*T34;
                
            quadnion = rotm2quat(Tw4(1:3,1:3));
        end

        T = TM(theta, d, a, alpha);
        dTdTheta = dTMdTheta(theta, d, a, alpha);
    end

    
end
        