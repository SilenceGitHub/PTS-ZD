function output = Algorithm()
global param robot data
index = param.index;

data.JReal(:,:,index) = robot.Jacobian2(robot, data.inputAngle(:,index));
% data.JHat(:,:,index) = JHat;


p_d = data.desiredPath(:,index);
dp_d = data.desiredVelocity(:,index);

p_a = robot.Position(robot,data.inputAngle(:,index));
data.actualPath(:,index) = p_a;

if index > 1
   delta_p_a = data.actualPath(:,index) - data.actualPath(:,index-1);
   delta_q = data.inputAngle(:,index) - data.inputAngle(:,index-1);   
   
   JHat_t = data.JHat(:,:,index-1);
   for i = 1:param.max_iter_num
        delta_JHat_k = param.gamma*(delta_p_a - JHat_t * delta_q) * delta_q';
        JHat_t = JHat_t + delta_JHat_k;
   end
   data.JHat(:,:,index) = JHat_t;
   delta_p_a - JHat_t * delta_q
end



dq = pinv(data.JHat(:,:,index))*(dp_d + param.lambda*ACF(p_d - p_a, param.acf,param.rho));

% dp_a = robot.Velocity(robot, data.inputAngle(:,index), dq);
% 
% dJHat = param.gamma * (dp_a - data.JHat(:,:,index) * dq) * dq';

output = [dq'];
param.index = param.index + 1;
end