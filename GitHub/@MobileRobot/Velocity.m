function velocity = Velocity(obj, angle, angularVelocity)

J = obj.Jacobian2(obj, angle);

velocity  = J * angularVelocity;

end