%% 第一章 1.3：直流电机的一阶系统 D 型迭代学习控制
% 被控对象：y_dot + a*y = b*u；每次运行从 y(0)=0 开始。
% 学习律：u_{k+1} = u_k + e_dot_k/beta，注意这里使用误差导数。
% 本脚本不需要 Control System Toolbox 或 Simulink。
clear; close all; clc;

% 书中参数；N 是时间区间数，K 是最后一次迭代的编号。
a = 1; b = 1; beta = 1.25; T = 1;
N = 2000; K = 9;
t = linspace(0,T,N+1).';
dt = t(2)-t(1);
yd = t;                       % 期望轨迹 y_d(t)=t
yd_dot = ones(size(t));        % 期望轨迹导数为 1

u = zeros(N+1,1);             % 初始输入 u_0(t)=0
y = zeros(N+1,K+1);           % 第 k+1 列保存第 k 次输出
e = zeros(size(y));
u_hist = zeros(size(y));
for k = 0:K
    y(:,k+1) = plant_response(u,a,b,dt);
    e(:,k+1) = yd - y(:,k+1);
    u_hist(:,k+1) = u;
    % 理想模型中由微分方程计算输出导数，避免差分微分的数值误差。
    y_dot = -a*y(:,k+1) + b*u;
    e_dot = yd_dot - y_dot;
    if k < K
        u = u + e_dot/beta;
    end
end

err_inf = max(abs(e),[],1);
rho = abs(1-b/beta);           % 收敛条件中的参数项，不是误差范数比值
fprintf('|1-b/beta| = %.6f < 1\n',rho);
fprintf('Iteration   max(abs(error))\n');
fprintf('%9d   %.6e\n',[(0:K); err_inf]);

% 输出与脚本放在同一目录，便于从其他工作目录运行。
out_dir = fileparts(mfilename('fullpath'));
figure('Color','w','Name','Chapter 1 Section 1.3');
plot(t,yd,'k--','LineWidth',1.8,'DisplayName','y_d(t)=t'); hold on;
plot_k = [0 1 2 9]; colors = lines(numel(plot_k));
for j = 1:numel(plot_k)
    k = plot_k(j);
    plot(t,y(:,k+1),'Color',colors(j,:),'LineWidth',1.3, ...
        'DisplayName',sprintf('k=%d',k));
end
grid on; box on; xlim([0 T]); ylim([0 1.05]);
xlabel('t'); ylabel('y_k(t)'); title('D-type iterative learning: Section 1.3');
legend('Location','southeast');
exportgraphics(gcf,fullfile(out_dir,'ilc_case_1_3_trajectories.png'),'Resolution',150);

figure('Color','w','Name','Tracking error');
semilogy(0:K,err_inf,'o-','LineWidth',1.5); grid on; box on;
xlabel('iteration k'); ylabel('max_t |e_k(t)|'); title('Tracking-error convergence');
exportgraphics(gcf,fullfile(out_dir,'ilc_case_1_3_error.png'),'Resolution',150);

% 验证初始输出、第一次解析解、收敛和每次运行的初始状态。
assert(rho < 1,'Learning parameter violates the convergence condition.');
assert(abs(err_inf(1)-1) < 1e-12,'Unexpected initial error.');
y1_exact = b/(a*beta)*(1-exp(-a*t));
assert(max(abs(y(:,2)-y1_exact)) < 1e-10,'First trial disagrees with its analytic solution.');
assert(all(y(1,:)==0),'Trials must reset to y(0)=0.');
assert(err_inf(end) < 1e-3,'Tracking error at k=9 exceeds tolerance.');
assert(all(diff(err_inf) <= 1e-10),'Error failed to decrease for this example.');

function y = plant_response(u,a,b,dt)
% 每个采样区间内输入恒定时，一阶系统的精确状态递推。
    n = numel(u); y = zeros(n,1);
    alpha = exp(-a*dt);
    gain = b*(1-alpha)/a;
    for i = 1:n-1
        y(i+1) = alpha*y(i) + gain*u(i);
    end
end
