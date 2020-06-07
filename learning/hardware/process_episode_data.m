function [] = process_episode_data(episode_number, plot_flag)

close all
filepath = './episode_5_issf.mat';
load(filepath);

Tsafe = issf_data(:,1);
hdot_r = issf_data(:,2);
hdot_r_learned = issf_data(:,3);
alpha = issf_data(:,4);
h = issf_data(:,5);

if(plot_flag)    
   
    idx = find(Tsafe>6,1);
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
    plot([0, Tsafe(idx)], -ones(2,1)*delta, '--r', 'LineWidth', 2.5);
    plot([0, Tsafe(idx)], -ones(2,1)*delta_l, '--b', 'LineWidth', 2.5);
    axis([0,Tsafe(idx),-1.1*max([delta,delta_l]),1.8*max(h(rng))]);
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
    plot([0, Tsafe(idx)],ones(2,1)*delta, '--r', 'LineWidth', 2.5);
    plot([0, Tsafe(idx)],ones(2,1)*delta_l, '--b', 'LineWidth', 2.5);
    legend({'$\vert\delta\vert/k$','$\vert\delta_l\vert/k$','$\overline{\delta}/k$','$\overline{\delta}_l/k$'}, 'Interpreter', 'latex')
    axis([0, Tsafe(idx), 0, 1.1*max([delta, delta_l])])
    
    

end