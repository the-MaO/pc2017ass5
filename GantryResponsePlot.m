function GantryResponsePlot(t,input,output,ul,uh,cl,ch,constId,xTarget,figureTitle)
% GantryResponsePlot(t,input,output,ul,uh,cl,ch,constId,xTarget,figureTitle)
% Plots the output of the gantry crane system together witht the usual
% input and output constraints. Calculates settling time to the xTarget
% state and plots it if it exists. 
% Plots constraints for all inputs and states indexed by constId
% Legend:
% Blue - state / input
% Black - Settling time
% Dark red - Constraint
% Light Grey - Target
allTitles=[{'X'},{'dX/dt'},{'Y'},{'dY/dt'},{'\theta'},{'d\theta/dt'},{'\psi'},{'d\psi/dt'}];
subplotid=[1 3 5 7 2 4 6 8];
info=lsiminfo(output,t,xTarget(:,1));
settlingTime=extractfield(info, 'SettlingTime');

figure('Name',figureTitle);
for ii=1:8
    subplot(5,2,subplotid(ii))
    plot(t,output(:,ii));
    h=gca;
    if any(constId==ii) % There are constraints for this state
        line([t(1) t(end)],[ch(find(constId==ii)) ch(find(constId==ii))],'Color',[0.5 0.1 0.1]);
        line([t(1) t(end)],[cl(find(constId==ii)) cl(find(constId==ii))],'Color',[0.5 0.1 0.1]);
    end    
    line([t(1) t(end)],[xTarget(ii,1) xTarget(ii,1)],'Color',[0.6 0.6 0.6],'LineStyle','--'); % target state
    if (ii == 1 || ii == 3)
           line([t(1) t(end)],[xTarget(ii,3) xTarget(ii,3)],'Color',[0.6 0.6 0.6],'LineStyle','--'); % target state 
    end
    line([settlingTime(ii) settlingTime(ii)],h.YLim,'Color',[0.3 0.3 0.3]);%settling time
    title(cell2mat(allTitles(ii)));
end
subplot(5,2,9)
stairs(t,input(:,1));
line([t(1) t(end)],[ul(1) ul(1)],'Color',[0.5 0.1 0.1]);
line([t(1) t(end)],[uh(1) uh(1)],'Color',[0.5 0.1 0.1]);
title('Input for X direction')
subplot(5,2,10)
stairs(t,input(:,2));
line([t(1) t(end)],[ul(2) ul(2)],'Color',[0.5 0.1 0.1]);
line([t(1) t(end)],[uh(2) uh(2)],'Color',[0.5 0.1 0.1]);
title('Input for Y direction')
end
