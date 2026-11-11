function [x, error, valid, strength] = reflectance_position(raw, whiteRef, blackRef, whiteLine)
%#codegen
% Each input sensor/reference vector contains eight values.
% Explicit labeled references work with either ADC polarity.
x = 0; error = 0; valid = false; strength = zeros(8,1);
total = 0; weighted = 0; lo = 1000; hi = 0;
for i = 1:8
    span = double(blackRef(i)) - double(whiteRef(i));
    if abs(span) < 80
        return;
    end
    b = 1000 * (double(raw(i)) - double(whiteRef(i))) / span;
    b = min(1000, max(0, b));
    if whiteLine
        b = 1000 - b;
    end
    strength(i) = b;
    total = total + b;
    weighted = weighted + (i-4.5)*b;
    lo = min(lo,b); hi = max(hi,b);
end
valid = hi >= 400 && hi-lo >= 200 && total > 0;
if valid
    x = weighted / total;
    error = -x;
end
end
