function [] = process_bad_episode_data(episode_number, plot_flag)
close all
filepath = sprintf('./bad_data/episode_%d.mat',episode_number);
load(filepath);

% Fit spline to h and evaluate it on time points.
h_tol = 1e-7;
h_spline = spaps(Tsafe,h,h_tol);
h_spline_vals = fnval(h_spline,Tsafe);

% Take numerical derivative approximation of h
hdot = gradient(h_spline_vals)./gradient(Tsafe);
hdot_learned = hdothat + Lfh_diff + Lgh_diff.*Uint;

% Compute error in barrier derivative
hdot_r = hdot-hdothat;
hdot_r_learned = hdot-hdot_learned;

% Compile data
data_s = [Xint, Dhdx, Uint, hdot_r];

data_plotting = [Tsafe, hdot_r, hdot_r_learned, relax, h_spline_vals];
save('sim_plot_data', 'data_plotting');

% Save learning data
%%%%%%%%%%%%%%%%%% TODO %%%%%%%%%%%%%%%%%%%%%%%
% filename = sprintf("episode_%d_learning.mat", episode_number);
% cd('./learning_data');
% save(filename, 'data_s');
% %save(filename, 'data_d');
% cd ..;


if(plot_flag)    
    figure(1)
    subplot(2,2,1);
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$x$ (m)', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe,Xint(:,1), 'Linewidth', 2.5)
    subplot(2,2,2);
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$\psi$ (rad)', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe,Xint(:,3), 'Linewidth', 2.5)
    subplot(2,2,3);
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$\dot{x}$ (m/s)', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe,Xint(:,2), 'Linewidth', 2.5)
    subplot(2,2,4);
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$\dot{\psi}$ (rad/s)', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe,Xint(:,4), 'Linewidth', 2.5)
    
    figure(2)
    subplot(2,2,1);
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$\frac{\partial h}{\partial x}$', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe,Dhdx(:,1), 'Linewidth', 2.5)
    subplot(2,2,2);
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$\frac{\partial h}{\partial \psi}$', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe,Dhdx(:,3), 'Linewidth', 2.5)
    subplot(2,2,3);
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$\frac{\partial h}{\partial \dot{x}}$', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe,Dhdx(:,2), 'Linewidth', 2.5)
    subplot(2,2,4);
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$\frac{\partial h}{\partial \dot{\psi}}$', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe,Dhdx(:,4), 'Linewidth', 2.5)
    
    figure(3)
    subplot(4,1,1);
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$u$', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe,Uint, 'Linewidth', 2.5)
    subplot(4,1,2);
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$h$', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe,h, 'Linewidth', 2.5)
    subplot(4,1,3);
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$\dot{h}$', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe,hdot,'r', 'Linewidth', 2.5)
    plot(Tsafe,hdothat,'b', 'Linewidth', 2.5)
    plot(Tsafe,hdot_learned, 'g', 'Linewidth', 2.5)
    legend({'$\dot{h}$', '$\widehat{\dot{h}}$', '$\widehat{\dot{h}}+b+au$'}, 'Interpreter', 'latex')
    subplot(4,1,4);
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$\dot{h}_r$', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe,abs(hdot_r), 'b', 'LineWidth', 2.5);
    plot(Tsafe,abs(hdot_r_learned), 'g', 'LineWidth', 2.5);
    legend({'$\vert \dot{h}-\widehat{\dot{h}} \vert$','$\vert \dot{h}-\widehat{\dot{h}}-b-au \vert$'}, 'Interpreter', 'latex');
  
end