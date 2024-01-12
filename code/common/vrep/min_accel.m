function [poly_coeff,vel_coeff] = min_accel(pos,der_pos,T)
%MIN_ACCEL Summary of this function goes here
A=[0 0 0 1;...
    T^3 T^2 T 1;...
    0 0 1 0;...
    3*T^2 2*T 1 0];
B=[pos(1) ;pos(2); der_pos(1); der_pos(2)];
poly_coeff=A\B;
t = 0:0.01:T;
pos = polyval(poly_coeff, t);
vel_coeff=polyder(poly_coeff);
vel=polyval(vel_coeff,t);
accel_coeff=polyder(vel_coeff);
accel=polyval(accel_coeff,t);
hold on;
plot(t, pos,t,vel,t,accel);
legend( 'Position Profile','Velocity Profile','Accel Profile');
hold off;
end

