function RobotPlant(q)
global robot data param
index = param.index;

if index < round(param.taskDuration/param.step)+1
    data.inputAngle(:,index+1) = q;
end

end