% Create fork table

s_throw = 2.5 / 1000;

a_back  = 2.5;
a_ramp  = 5;
a_gear  = [ 1 3 ] * 45;
s_dir   = [ 1 -1 ];

angle   = 0;
fork    = 0;
for i = 1 : length(a_gear)
    angle(end + 1)  = a_gear(i) - a_back - a_ramp;
    angle(end + 1)  = a_gear(i) - a_back;
    angle(end + 1)  = a_gear(i);
    angle(end + 1)  = a_gear(i) + a_back;
    angle(end + 1)  = a_gear(i) + a_back + a_ramp;
    
    fork(end + 1)   = 0;
    fork(end + 1)   = s_dir(i) * s_throw;
    fork(end + 1)   = s_dir(i) * s_throw;
    fork(end + 1)   = s_dir(i) * s_throw;
    fork(end + 1)   = 0;
end
angle(end + 1)  = 360;
fork(end + 1)   = 0;

for i = 1 : length(angle)
    fprintf('%f\t%f\n', angle(i) * pi / 180, fork(i))
end

