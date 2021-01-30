function [w, x, y, z] = invquat(quat)

%takes 1x4 matrix representing a quaternion as input and returns the
%inverse quaternion in the form [w x y z]
% Kyle Molinari

w = quat(1)/norm(quat)^2;
x = -quat(2)/norm(quat)^2;
y = -quat(3)/norm(quat)^2;
z = -quat(4)/norm(quat)^2;
end
