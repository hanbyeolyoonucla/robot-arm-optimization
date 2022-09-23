function [state,options,optchanged] = gaoutfun(options,state,flag)
persistent h1 h2 historyPop historyBest historyScore C
optchanged = false;
disp(state.Generation+1)
switch flag
    case 'init'
        h1 = figure;
        C = 1:1e6;
        grid on; hold on; view(3); axis equal;
        title('Genetic Algorithm: P.R.F populations')
        xlabel('x','FontSize',10); ylabel('y','FontSize',10);
        zlabel('z','FontSize',10); set(gca,'FontSize',10);
        alpha = state.Population(:,6);
        x = state.Population(:,7);
        y = state.Population(:,8);
        z = state.Population(:,9);
        sizePopulation = length(alpha);
        posPRF = zeros(3,sizePopulation);
        for ii = 1:sizePopulation
            posPRF(:,ii) = rot('y',alpha(ii))*[x(ii);y(ii);z(ii)];
        end
        color = zeros(sizePopulation,1) + C(fix(state.Generation/10)+1);
        scatter3(posPRF(1,:),posPRF(2,:),posPRF(3,:),5,color);
        colormap default
        historyPop(:,:,1) = state.Population;
        historyScore(:,1) = state.Score;
        assignin('base','gapopulationhistory',historyPop);
        assignin('base','gascorehistory',historyScore);
        
        % figure of best score at each iter
        h2 = figure;
        grid on; hold on; title('Genetic Algorithm: best score at each generation')
        xlabel('Generation','FontSize',10); ylabel('Best Score','FontSize',10); set(gca,'FontSize',10);
        
    case 'iter'
        disp(state.Best(end))
        % Update the history every 10 generations.
%         if rem(state.Generation,10) == 0
            ss = size(historyPop,3);
            historyPop(:,:,ss+1) = state.Population;
            historyScore(:,ss+1) = state.Score;
            assignin('base','gapopulationhistory',historyPop);
            assignin('base','gascorehistory',historyScore);
%         end
        % Update the plot.
        figure(h1)
        alpha = state.Population(:,6);
        x = state.Population(:,7);
        y = state.Population(:,8);
        z = state.Population(:,9);
        sizePopulation = length(alpha);
        posPRF = zeros(3,sizePopulation);
        for ii = 1:sizePopulation
            posPRF(:,ii) = rot('y',alpha(ii))*[x(ii);y(ii);z(ii)];
        end
        color = zeros(sizePopulation,1) + C(fix(state.Generation/10)+1);
        scatter3(posPRF(1,:),posPRF(2,:),posPRF(3,:),5,color);
        
        figure(h2)
        plot(state.Generation, -state.Best(end),'r.')
%         pause(0.1)
        % Update the fraction of mutation and crossover after 25 generations.
%         if state.Generation == 25
%             options.CrossoverFraction = 0.8;
%             optchanged = true;
%         end
    case 'done'
        % Include the final population in the history.
        ss = size(historyPop,3);
        historyPop(:,:,ss+1) = state.Population;
        historyScore(:,ss+1) = state.Score;
        historyBest = state.Best;
        assignin('base','gapopulationhistory',historyPop);
        assignin('base','gascorehistory',historyScore);
        assignin('base','gabestscorehistory',historyBest);
end