function angleList = computeAngleList(c1, c2, c3, c4, c5, deltaT, t0, tn)
t = t0:deltaT:tn;
angleList = atan(c1 +  2* c2 * t + 3*c3*t.^2 + 4*c4*t.^3 +  5*c5* t.^4) * 180 / pi;
end