m = 10;
I = 1;

b1 = 10;
b2 = 10;

d1 = 0.3;
d2 = 0.3;

L1 = 10;
L2 = 3;

K1 = L1 + 2*L2;
K2 = L1*d1 - 2*L2*d2;

A = [0 1 0 0;
     -b1/m 0 L1/m 0;
     0 0 0 1;
     0 0 L2/I -b2/I];

B = [0 ; L1/m; 0; L2/I];

C = [1 0 0 0;
     0 0 1 0;];

C = eye(4);

D = 0;

Q = [1 0 0 0;
     0 10 0 0;
     0 0 80 0;
     0 0 0 100;];

R = 1;
N = 0;

sys = ss(A, B, C, D);

K = lqr(sys, Q, R, N);

lqr_sys = ss(A - B*K, B, C, D);
disp(dcgain(lqr_sys));

step(lqr_sys);

