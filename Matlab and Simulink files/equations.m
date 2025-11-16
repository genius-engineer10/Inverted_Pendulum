%% My version of Eq(8)
kcw = 1;
L = 1;
B = 1;
A = 1;
t = linspace(1, 500,100);
x = linspace(-0.5, 0.5, 100);
dt = 0.01;

v = gradient(x,dt);
arg = B * (1 ./ (1 - abs(x)/L));

a = -kcw .* x .* (abs(x)./L).^0.5 .* tanh(arg) - A .* v;
subplot(1,3,1);
plot(t,x);
grid on;



%% original Eq(8)
kcw = 1;
L = 1;

x = linspace(-0.5, 0.5, 100);

a = kcw * sign(x) .* log(1 - abs(x)/L);
subplot(1,3,2);
plot(x, a);
xlabel('x');
ylabel('a');
title('Original equation');
grid on;

%% sir's intution
kcw = 1;
L = 1;

x = linspace(-0.5, 0.5, 100);
t = linspace (1,100,100);
a = kcw * sign(x) .* abs(x).^0.5 .* log(1 - abs(x)/L);
subplot(1,3,3);
plot(t,x);

grid on;
