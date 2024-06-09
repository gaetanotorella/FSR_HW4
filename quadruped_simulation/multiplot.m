function h = multiplot(t, X, Xd, x_label, y_label, legenda, pdf_name)

    set(0, 'DefaultTextInterpreter', 'latex')
    set(0, 'DefaultLegendInterpreter', 'latex')
    set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
    
    lw = 2;
    h = figure('Renderer', 'painters', 'Position', [10 10 900 350]);
    removeToolbarExplorationButtons(h)

    plot(t,X(:,1),'r',...
         t,X(:,2),'g',...
         t,X(:,3),'b',...
         t,Xd(:,1),'r--',...
         t,Xd(:,2),'g--',...
         t,Xd(:,3),'b--','linewidth',1)
    
    xlabel(x_label)
    ylabel(y_label)
    set(gcf,'color','w');
    legend(legenda);

    set(gca, 'FontSize',16);
    grid on
    box on
    exportgraphics(h, pdf_name);

    set(0, 'DefaultTextInterpreter', 'none')
    set(0, 'DefaultLegendInterpreter', 'none')
    set(0, 'DefaultAxesTickLabelInterpreter', 'none')


end

