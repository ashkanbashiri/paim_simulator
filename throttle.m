function T = throttle(alpha,v)
%
Tm = 190; % max torque, Nm
omegan = 420; % rad/s, given
beta = 0.4; % given
T = Tm*(1-beta*(alpha*v/omegan-1).^2);
end