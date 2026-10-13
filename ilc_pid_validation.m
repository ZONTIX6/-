% ILC_PID_VALIDATION
% Numerical verification of the PID-type iterative learning law
%
%   u_{k+1}(t) = u_k(t) + Gamma*e_dot_k(t)
%                         + L*e_k(t)
%                         + Psi*int_0^t e_k(s) ds
%
% Plant: scalar linear system x_dot = a*x + b*u, y = c*x.
% The initial state is reset to the same value at every trial.

clear; close all; clc;

%% Time grid and plant
T  = 1.0;
N  = 10000;               % finer grid reduces the zero-order-hold floor
dt = T/N;
t  = linspace(0,T,N+1).';

a = -0.5;                 % x_dot = a*x + b*u
b = 1.0;
c = 1.0;                  % y = c*x
x0 = 0.0;

% Exact zero-order-hold discretization over each time interval.
Ad = exp(a*dt);
Bd = (Ad-1)/a*b;          % a ~= 0 here

%% Desired trajectory; yd(0)=y(0) so the theorem's initial-error condition holds
yd     = 0.7*sin(2*pi*t/T) + 0.2*sin(4*pi*t/T);
yd_dot = 0.7*(2*pi/T)*cos(2*pi*t/T) ...
       + 0.2*(4*pi/T)*cos(4*pi*t/T);

%% PID learning gains
Gamma = 0.60;             % D gain: rho(1-c*b*Gamma)=0.40 < 1
L     = 0.08;             % P gain
Psi   = 0.04;             % I gain

rho = abs(1-c*b*Gamma);
fprintf('Theorem check: rho(I-c*b*Gamma) = %.4f < 1\n',rho);

%% Iterations
K = 80;
u = zeros(N+1,1);         % initial input profile
err_inf = zeros(K+1,1);
err_l2  = zeros(K+1,1);
u_hist  = zeros(N+1,K+1);
y_hist  = zeros(N+1,K+1);

for k = 1:K+1
    % Run the plant once with the current input profile.
    x = zeros(N+1,1);
    x(1) = x0;
    for j = 1:N
        x(j+1) = Ad*x(j) + Bd*u(j);
    end
    y = c*x;
    e = yd-y;

    err_inf(k) = max(abs(e));
    err_l2(k)  = sqrt(trapz(t,e.^2));
    u_hist(:,k) = u;
    y_hist(:,k) = y;

    if k <= K
        % Exact continuous-time derivative on the sampled trajectory.
        y_dot = c*(a*x + b*u);
        e_dot = yd_dot-y_dot;

        % Integral term in the learning law.
        e_int = cumtrapz(t,e);

        % PID ILC update, applied pointwise in t.
        u = u + Gamma*e_dot + L*e + Psi*e_int;
    end
end

fprintf('Initial ||e_0||_inf = %.6e\n',err_inf(1));
fprintf('Final   ||e_K||_inf = %.6e\n',err_inf(end));
fprintf('Initial ||e_0||_2   = %.6e\n',err_l2(1));
fprintf('Final   ||e_K||_2   = %.6e\n',err_l2(end));

%% Figures
fig1 = figure('Color','w','Position',[100 100 900 700]);
subplot(2,1,1);
plot(t,yd,'k--','LineWidth',1.5); hold on;
plot(t,y_hist(:,1),'Color',[0.85 0.33 0.10],'LineWidth',1.1);
plot(t,y_hist(:,6),'Color',[0.00 0.45 0.74],'LineWidth',1.1);
plot(t,y_hist(:,21),'Color',[0.47 0.67 0.19],'LineWidth',1.1);
plot(t,y_hist(:,end),'r','LineWidth',1.5);
grid on; xlabel('t'); ylabel('output');
legend('y_d','trial 0','trial 5','trial 20','trial 80','Location','best');
title('PID iterative learning: output tracking');

subplot(2,1,2);
semilogy(0:K,err_inf,'b','LineWidth',1.5); hold on;
semilogy(0:K,err_l2,'r--','LineWidth',1.5);
grid on; xlabel('iteration k'); ylabel('error norm');
legend('||e_k||_\infty','||e_k||_2','Location','best');
title('Error convergence');
saveas(fig1,'pid_ilc_validation.png');

fig2 = figure('Color','w','Position',[120 120 900 500]);
plot(t,u_hist(:,1),'Color',[0.85 0.33 0.10],'LineWidth',1.1); hold on;
plot(t,u_hist(:,end),'b','LineWidth',1.4);
grid on; xlabel('t'); ylabel('u_k(t)');
legend('u_0','u_{80}','Location','best');
title('Input profile learned across trials');
saveas(fig2,'pid_ilc_input.png');
