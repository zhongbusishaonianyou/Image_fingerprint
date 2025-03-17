clear;clc;
ResultsDir = './pr_result/';
%%
title_str = strcat('KITTI 08');

%% Params 
FigIdx = 2;
figure(FigIdx); clf;

%% Main 
SequenceNames = dir(ResultsDir); SequenceNames(1:2, :) = []; SequenceNames = {SequenceNames(:).name};
nSequences = length(SequenceNames);
      
    line_width = 3;
    
    LineColors = colorcube(nSequences);
    LineColors = linspecer(nSequences,'qualitative');
%     LineColors = linspecer(nSequences,'sequential');
    LineColors = flipud(LineColors);

    AUCs = zeros(1, nSequences);
    for ithSeq = 1:nSequences

        % seq info 
        ithSeqName = SequenceNames{ithSeq};
        SequenceNames{ithSeq} = string(ithSeqName);
        
        ithSeqPath = strcat(ResultsDir, ithSeqName, '/');
        ithSeqPRcurveData = dir(ithSeqPath); ithSeqPRcurveData(1:2, :) = []; ithSeqPRcurveData = {ithSeqPRcurveData(:).name};

        % load 
        Precisions = load(strcat(ithSeqPath, ithSeqPRcurveData{1}));
        Precisions = Precisions.Precisions;

        Recalls = load(strcat(ithSeqPath, ithSeqPRcurveData{2}));
        Recalls = Recalls.Recalls;
        
        AUC = 0;
        for ith = 1:length(Precisions)-1    
            small_area = 1/2 * (Precisions(ith) + Precisions(ith+1)) * (Recalls(ith+1)-Recalls(ith));
            AUC = AUC + small_area;
        end
        AUCs(ithSeq) = AUC;
              
        % draw 
        figure(FigIdx); 
        set(gcf, 'Position', [10 10 800 600]);
        
        fontsize = 10;
        p = plot(Recalls, Precisions,'-b*','LineWidth', line_width); % commonly x axis is recall
        title(title_str, 'FontSize', fontsize);
        xlabel('Recall', 'FontSize', fontsize); ylabel('Precision', 'FontSize', fontsize);
        set(gca, 'FontSize', fontsize+5)
        xticks([0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0])
        xticklabels({'0','0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1'})
        yticks([0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0])
        yticklabels({'0','0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1'})


        p(1).Color = LineColors(ithSeq, :);
        p(1).MarkerEdgeColor = LineColors(ithSeq, :);
        % axis equal;
        xlim([0, 1]); ylim([0,1]);
        grid on; grid minor;
        hold on;
        
    end

    lgd = legend(SequenceNames, 'Location', 'best');
    lgd.FontSize = fontsize + 3;
    lgd.FontWeight = 'bold';

    grid minor;

    name = 'prcurve';
    print('-bestfit', name,'-dpdf')


