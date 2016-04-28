% Plots L2 and Aimpoint vectors

[t dim] = size(L2Vec);
n = 1:t;

% Plot axes
for i=1:dim
    subplot(1,3,i);
    plot(n,L2Vec(n,i)); hold on;
    plot(n,AimPointVect(n,i)); hold off;
    legend('x_{L2_{Vec}}','y_{AP_{Vec}}');
end


ylabel('meters');
