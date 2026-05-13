function [f1, varargout] = pend_plots(t, x_history, params, num_samples, refresh_rate, pause_t, u_history, ctrl_mode_history)
%Double Inverted Pendulum Animation
% INPUTS:
%   x_history: n x N

arguments
    t
    x_history
    params
    num_samples
    refresh_rate = 1
    pause_t = 0
    u_history = nan(1,length(t))
    ctrl_mode_history = ones(1, num_samples)
end

% num_samples = length(t);

% unpack parameters
g = params.g;
mc = params.mc;
mp = params.mp;
Jp = params.Jp;
bc = params.bc;
bp = params.bp;
Lp = params.Lp; Lp = 1;
r_pulley = params.r_pulley;
Ke = params.Ke;

x1_history = x_history(1,:);
x2_history = x_history(2,:);
x3_history = x_history(3,:);
x4_history = x_history(4,:);

% joint coordinates (x,y) using derived eqns
j1_x = x1_history;
j1_y = zeros(1, num_samples);

j2_x = j1_x - Lp.*sin(x3_history);
j2_y = Lp.*cos(x3_history);

% --Animation--
f1 = figure('Name', 'Double Inverted Pendulum', 'Color', 'w');
hold on; grid on; axis equal;

% axis limits
xaxlim = 2;
xlim([-xaxlim, xaxlim])
ylim([-(Lp+.1), (Lp+.1)])
xticks(-xaxlim:0.5:xaxlim)
xlabel('Position (m)'); ylabel('Height (m)')
title('Double Inverted Pendulum Cart')

% plot first frame
cart_width = 0.15;
cart_height = 0.03;

h_cart = rectangle('Position', [j1_x(1)-.5*cart_width, -.5*cart_height, cart_width, cart_height]);
h_link1 = plot([j1_x(1), j2_x(1)], [j1_y(1), j2_y(1)], 'color', [0, 0.4470, 0.7410]);
h_joints = plot(j1_x(1), j1_y(1), 'o', 'MarkerSize', 3, 'MarkerFaceColor', [0, 0.4470, 0.7410], 'Color', [0, 0.4470, 0.7410]);
% h_ctrl_mode = text(2, (L1+L2+.45), 'Tracking', 'Color', '#D95319');
time = text(1.25, Lp+0.25, 't=0.0 s');

pause(1.0)
idx = 1;
startTime = tic;
% animation loop
while idx < num_samples
    elapsed = toc(startTime);

    % Find the index in the simulation time vector 't' that matches real-world time
    % We use a simple search here; for very high-frequency data, use binary search
    while idx < num_samples && t(idx) < elapsed
        idx = idx + 1;
    end

    % if ctrl_mode_history(k) == 1
    %     set(h_ctrl_mode, 'String', 'Tracking', 'Color', '#D95319');
    % elseif ctrl_mode_history(k) == 2
    %     set(h_ctrl_mode, 'String', 'Regulation', 'Color', '#77AC30');
    % end

    % update positions
    set(h_cart, 'Position', [j1_x(idx)-.5*cart_width, -.5*cart_height, cart_width, cart_height])
    set(h_link1, 'XData', [j1_x(idx), j2_x(idx)], 'YData', [j1_y(idx), j2_y(idx)])
    % set(h_joints, 'XData', [j1_x(idx), j2_x(idx)], 'Ydata', [j1_y(idx), j2_y(idx)])
    set(h_joints, 'XData', j1_x(idx), 'Ydata', j1_y(idx))
    set(time, 'string', sprintf('t=%.1f s', t(idx)));
    drawnow
end
t_anim = toc(startTime);
fprintf('Animation time: %.2fs\n', t_anim);


if nargout > 1
    % --xc, theta, u vs time--
    varargout{1} = figure;
    subplot(3,1,1)
    hold on; grid on;
    title('$x_c$')
    plot(t, x1_history)

    subplot(3,1,2)
    hold on; grid on;
    title('$\theta_1$ and $\theta_2$')
    plot(t, [x3_history; x5_history])
    legend('$\theta_1$', '$\theta_2$')
    
    subplot(3,1,3)
    hold on; grid on;
    title('Control Input: u')
    plot(t, u_history);
    xlabel('time (s)')
    
    % states vs time
    varargout{2} = figure;
    subplot(3,1,1)
    hold on; grid on;
    title('Cart')
    plot(t, [x1_history; x2_history])
    legend('$x_c$', '$\dot{x}_c$')
    
    subplot(3,1,2)
    hold on; grid on;
    title('Pendulum 1')
    plot(t, [x3_history; x4_history])
    legend('$\theta_1$', '$\dot{\theta}_1$')
    
    subplot(3,1,3)
    hold on; grid on;
    title('Pendulum 2')
    plot(t, [x5_history; x6_history])
    legend('$\theta_2$', '$\dot{\theta}_2$')
    xlabel('time (s)')
end


end