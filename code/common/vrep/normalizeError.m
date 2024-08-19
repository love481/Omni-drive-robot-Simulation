function [out] = normalizeError(value,range)
out=((value+range)*(1/range))-1;
end

