function [ complexity ] = compcplx( n )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
complexity = 0;
for r=1:n
    for i=0:r-1
        complexity = complexity + ((-1)^i)*((r-i)^n)*  (factorial(r)/(factorial(i)*factorial(r-i)));
    end
end

end

