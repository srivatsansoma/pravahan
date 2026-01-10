m = 10;
I1 = 1;
I2 = 0.5;

b1 = 10;
b2 = 10;

d1 = 0.3;
d2 = 0.3;
d3 = 0.1;

L1 = 10;
L2 = 3;

A = [0 1 0 0;
     0 -5 30 0;
     0 0 0 1;
     0 0 -3 -5;];

B = [0 0;
     L1/m 2*L2/m;
     0 0;
     L1*d1/I1 -2*L2*d2/I1;];

C = [1 0 0 0;
     0 0 1 0;];

D = 0;

Q = eye(4);
Q_i = eye(2);

Q_aug = blkdiag(Q, Q_i);

R = eye(2);

% Compute the continuous-time state-space representation
sys = ss(A, B, C, D);

K = lqi(sys, Q_aug, R, 0);

disp(K);

K_x = K(:, 1:4);
K_i = K(:, 5:6);


A_aug = [A zeros(4,2);
         -C zeros(2,2)];
B_aug = [B; zeros(2,2)];

Acl = A_aug - B_aug * [K_x K_i];

% Create state-space object
lqi_sys = ss(Acl, [], [], []);

% Optional: check stability
eig(Acl)

% Simulate example (initial conditions)
x0 = [0.1;0;0.05;0]; % initial states
z0 = [0;0];          % integrator states
x0_aug = [x0; z0];

t = 0:0.01:50;
[y,t,x] = initial(lqi_sys, x0_aug, t);

% Plot states
figure;
plot(t, x(:,1:4));
xlabel('Time (s)');
ylabel('States');
legend('x1','x2','x3','x4');
grid on;

figure;
plot(t, x(:,5:6));
xlabel('Time (s)');
ylabel('Integrator states');
legend('z1','z2');
grid on;

%%test
A_aug = [A_ zeros(4,2);
         -C zeros(2,2)];
B_aug = [B; zeros(2,2)];

Acl = A_aug - B_aug * [K_x K_i];

% Create state-space object
lqi_sys = ss(Acl, [], [], []);

% Optional: check stability
eig(Acl)

% Simulate example (initial conditions)
x0 = [0.1;0;0.05;0]; % initial states
z0 = [0;0];          % integrator states
x0_aug = [x0; z0];

t = 0:0.01:50;
[y,t,x] = initial(lqi_sys, x0_aug, t);

% Plot states
figure;
plot(t, x(:,1:4));
xlabel('Time (s)');
ylabel('States');
legend('x1','x2','x3','x4');
grid on;

figure;
plot(t, x(:,5:6));
xlabel('Time (s)');
ylabel('Integrator states');
legend('z1','z2');
grid on;
