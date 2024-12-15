function position = VirtualFeedback()
global data robot param
index = param.index;
position = robot.Position(robot,data.inputAngle(:,index));
velocity = robot.Velocity(robot, data.inputAngle(:,index), angularVelocity)

data.actualPath(:,index) = position;
end