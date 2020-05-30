
params = struct();

params.g = 9.81;
params.mr = 0.25;
params.ir = 0.0001;
params.d = 0.1;
params.r = 0.02;

% 1. Learn how to use the symbolic toolbox in MATLAB
% you will need the state variables, and control input to be declared as symbolic

% 2. call your "eom" function to get \ddot{q} symbolically

% 3. Linearize the system at 0 (as shown in lecture)
% You should end up with A (4x4), and b (4x1)

% 4. Check that (A,b) is  controllable
% Number of uncontrollable states should return 0

% 5. Use LQR to get K as shown in the lecture

syms th phi dth dphi u
qdd = eom(params, th, phi, dth, dphi, u);
x = [th; phi; dth; dphi];
f = [dth; dphi; qdd];

A = [diff(f, th), diff(f, phi), diff(f, dth), diff(f, dphi)];
A = subs(A, {th, phi, dth, dphi}, {0, 0, 0, 0});
A = double(A);
b = diff(f, u);
b = subs(b, {phi, u}, {0, 0});
b = double(b);

co = ctrb(A, b);
if length(A) - rank(co) == 0
    K = lqr(A, b, eye(4), 1, 0)
end


