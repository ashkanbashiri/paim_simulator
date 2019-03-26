function T = dTdv(alpha,v)
%
Tm = 190; % max torque, Nm
omegan = 420; % rad/s, given
beta = 0.4; % given
T = -2*Tm*beta*alpha^2*v/omegan^2 + 2*alpha*Tm*beta/omegan;
end