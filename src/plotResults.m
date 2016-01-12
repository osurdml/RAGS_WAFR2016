clear variables ;
close all ;

for k = 1
    data = load(sprintf('../results/pathCosts%d.txt',k)) ;
    diffCost = zeros(size(data)) ;
    
    for i = 1:size(data,1)
        for j = 1:size(data,2)
            diffCost(i,j) = data(i,j) - data(i,end) ;
            if diffCost(i,j) > 1e3
                diffCost(i,j) = NaN ;
            elseif diffCost(i,j) < 0
                fprintf('Entry (%d,%d): %f\n',i,j,diffCost(i,j)) ;
            end
        end
    end
    
    figure
    hold on
    title('Comparison of Path Costs')
    ylabel('Path cost above optimal')
    hb = boxplot(diffCost(:,[1:3,end-1])) ;
    set(gca,'xtick',[1,2,3,4],'xticklabel',{'RAGS  ','Naive A*  ','Greedy  ','D*  '}) ;
    y_lim = get(gca,'ylim') ;
    set(gca,'ylim',[-5,y_lim(2)]) ;
    th = rotateticklabel(gca,45) ;
    set(th,'fontsize',get(gca,'fontsize')) ;
    set(gca,'xticklabel',{''}) ;
    set(gcf,'position',[2040 520 330 420])
    pos = get(gca,'position') ;
    pos(2) = 0.16 ;
    set(gca,'position',pos)
end