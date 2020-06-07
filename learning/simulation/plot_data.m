function [] = plot_data(episode_number, plot_flag)

close all
filepath = './sim_plot_data.mat';
load(filepath);

Tsafe = data_plotting(:,1);
hdot_r = data_plotting(:,2);
hdot_r_learned = data_plotting(:,3);
alpha = data_plotting(:,4);
h = data_plotting(:,5);

if(plot_flag)    
   
    idx = find(Tsafe>3.5,1);
    %idx = length(Tsafe);
    rng = 1:idx-1;
    delta = max(abs(hdot_r(rng))./alpha(rng)); 
    delta_l = max(abs(hdot_r_learned(rng))./alpha(rng));

    figure(1)
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$h$', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe(rng),h(rng), 'Linewidth', 2.5)
    plot([0, 1.15*Tsafe(idx)], -ones(2,1)*delta, '--r', 'LineWidth', 2.5);
    plot([0, 1.15*Tsafe(idx)], -ones(2,1)*delta_l, '--b', 'LineWidth', 2.5);
    axis([0, 1.15*Tsafe(idx),-1.1*max([delta,delta_l]),2.0*max(h(rng))]);
    legend({'$h$','$\overline{\delta}/k$','$\overline{\delta}_l/k$'}, 'Interpreter', 'latex')
    
    figure(2)
    hold on; grid on; box on;
    fig = gcf;
    ax = gca;
    ax.FontSize = 18;
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex')
    ylabel('$\vert\delta\vert/\alpha$', 'FontSize', 18, 'Interpreter', 'latex')
    plot(Tsafe(rng),abs(hdot_r(rng))./alpha(rng), 'r', 'LineWidth', 2.5);
    plot(Tsafe(rng),abs(hdot_r_learned(rng))./alpha(rng), 'b', 'LineWidth', 2.5);
    plot([0, 1.15*Tsafe(idx)],ones(2,1)*delta, '--r', 'LineWidth', 2.5);
    plot([0, 1.15*Tsafe(idx)],ones(2,1)*delta_l, '--b', 'LineWidth', 2.5);
    legend({'$\vert\delta\vert/k$','$\vert\delta_l\vert/k$','$\overline{\delta}/k$','$\overline{\delta}_l/k$'}, 'Interpreter', 'latex')
    axis([0, 1.15*Tsafe(idx), 0, 1.2*max([delta, delta_l])])
    
    

end