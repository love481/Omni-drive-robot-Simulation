function [out] = normalizeError(value,range)
out=((value+2*range)*(3/(2*range)))-3;
end

