function [w, x, y, z] = quatmult(quata, quatb)

%takes two 1x4 matrices representing quaternions as input and returns a
%quaterion representing the multiplied inputs (in the form quata * quatb).
%since quaternion multiplication is not commutative the order of the inputs
%must be specified.
% Kyle Molinari

wb = quatb(1);
xb = quatb(2);
yb = quatb(3);
zb = quatb(4);

wa = quata(1);
xa = quata(2);
ya = quata(3);
za = quata(4);

w = wb*wa-xb*xa-yb*ya-zb*za;
x = wb*xa+xb*wa+zb*ya-yb*za;
y = wb*ya+yb*wa+xb*za-zb*xa;
z = wb*za+zb*wa+yb*xa-xb*ya;
end
