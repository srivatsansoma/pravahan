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
     0 -b1/m L1/m 0;
     0 0 0 1;
     0 0 L2/I -b2/I];

B = [0 ; L1/m; 0; L2/I];

C = [1 0 0 0;];
D = 0;

Q = [1 0 0 0;
     0 10 0 0;
     0 0 80 0;
     0 0 0 100;];

R = 10;
N = 0;

A_aug = [A zeros(4,1);
         -C zeros(1,1);];

B_aug = [B;
         0;];
C_aug = [C zeros(1,1)];

Q_i = 5;
Q_aug = blkdiag(Q,Q_i);

sys = ss(A, B, C, D);
K = lqi(sys, Q_aug, R);

disp(K);
