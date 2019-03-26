function [ consumptionRate ] = fuelConsumption( v,a )
%computes fuel consumption rate in ml/s
b0 = 0.1569;
b1 = 2.450e-2;
b2 = -7.415e-4;
b3 = 5.975e-5;
c0 = 0.07224;
c1 = 9.681e-2;
c2 = 1.075e-3;

fCruise = b0 + b1*v + b2*v^2 + b3*v^3;
fAccel = a*(c0 + c1*v + c2*v^2);
if(a<0)
    consumptionRate = 0;
else
    consumptionRate = fCruise + fAccel;
end



end

