clc
clear
close all


% The loaded matrix is 4 dimension, the first element (rows) is related to
% different damping values (5 cases), the second element (columns) is related to
% different inertia/end velocity (4 cases), the third element is ML/AP direction (2 cases), and
% the fourth element is different subjects (depend on the number of
% subjects)

% making plots
%figure
for gp=1:2
    str1 = ['C:\Users\fzahedi1admin\OneDrive - Arizona State University\Projects\Damping Map\Experiments\Test\SensitivityAnalysis\MV analysis\Data\Groupdata\Group',num2str(gp),'\Average_rate_total.mat'];
    load(str1);

    str2 = ['C:\Users\fzahedi1admin\OneDrive - Arizona State University\Projects\Damping Map\Experiments\Test\SensitivityAnalysis\MV analysis\Data\Groupdata\Group',num2str(gp),'\Average_time_total.mat'];
    load(str2);

    % k= 1 is ML and k=2 is AP direction
    for k=1:2
       for i=1:size(rate_total,1) % i is different damping value
            clear M Q
            M(:,:)=rate_total(i,:,k,:);
            Q=M';
            AVE_rate(i,:,k)=mean(Q,1);
            STD_rate(i,:,k)=std(Q);
       end 

       for i=1:size(tave_total,1) % i is different damping value
            clear M Q
            M(:,:)=tave_total(i,:,k,:);
            Q=M';
            AVE_tave(i,:,k)=mean(Q,1);
            STD_tave(i,:,k)=std(Q);
        end 
    end

    % ML direction

    % success rate
    if gp == 2
        Mass={'100mm/s'; '150mm/s'; '200mm/s'; '250mm/s'};
    else
        Mass={'8kg'; '10kg'; '12kg'; '14kg'};
    end
    dir1=[1 2 3 4];
%     if (gp == 1)
%         figure
%     end
    figure(1)
    subplot(4,2,1+gp*(gp-1)+2*(gp-1))
    bar(dir1,AVE_rate(:,1:4,1)',1)
    set(gca, 'XTickLabel',Mass, 'XTick',1:numel(dir1))
    ngroups = size(AVE_rate(:,1:4,1)', 1);
    nbars = size(AVE_rate(:,1:4,1)', 2);
    % Calculating the width for each bar group
    groupwidth = min(0.8, nbars/(nbars + 1.5));
    hold on
    y=AVE_rate(:,1:4,1)';
    error= STD_rate(:,1:4,1)';
    for i = 1:nbars
        x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
        errorbar(x,y(:,i),error(:,i) , '.k');
    end
    axis([0 5 0 115])
    ylabel('success rate %')
    if gp == 1
        legend('0','-10','-15','-20','-25')
        title('ML direction')
    end
    %annotation('textbox', [0, 0.95, 0, 0], 'string', 'A','fontweight','bold','fontsize',14)

    % average time
    subplot(4,2,2+gp*(gp-1)+2*gp-1)
    bar(dir1,AVE_tave(:,1:4,1)',1)
    set(gca, 'XTickLabel',Mass, 'XTick',1:numel(dir1))
    ngroups = size(AVE_tave(:,1:4,1)', 1);
    nbars = size(AVE_tave(:,1:4,1)', 2);
    % Calculating the width for each bar group
    groupwidth = min(0.8, nbars/(nbars + 1.5));
    hold on
    y=AVE_tave(:,1:4,1)';
    error= STD_tave(:,1:4,1)';
    for i = 1:nbars
        x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
        errorbar(x,y(:,i),error(:,i) , '.k');
    end
    axis([0 5 0 2500])
    %legend('0','-10','-15','-20','-25')
    if gp ==1
        xlabel('Inertia');
    else
        xlabel('End velocity of perturbation');
    end
    ylabel('Average time to regain stability (ms)')

    % AP direction---------------------------------------------------------
    % success rate
    %Mass={'100mm/s'; '150mm/s'; '200mm/s'; '250mm/s'};
    dir1=[1 2 3 4];
    %figure(2)
    subplot(4,2,1+gp*(gp-1)+2*(gp-1)+1)
    bar(dir1,AVE_rate(:,1:4,2)',1)
    set(gca, 'XTickLabel',Mass, 'XTick',1:numel(dir1))
    ngroups = size(AVE_rate(:,1:4,2)', 1);
    nbars = size(AVE_rate(:,1:4,2)', 2);
    % Calculating the width for each bar group
    groupwidth = min(0.8, nbars/(nbars + 1.5));
    hold on
    y=AVE_rate(:,1:4,2)';
    error= STD_rate(:,1:4,2)';
    for i = 1:nbars
        x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
        errorbar(x,y(:,i),error(:,i) , '.k');
    end
    axis([0 5 0 115])
    %ylabel('success rate %')
    if gp == 1
        legend('0','-20','-30','-40','-50')
        title('AP direction')
    end
    %annotation('textbox', [0, 0.95, 0, 0], 'string', 'A','fontweight','bold','fontsize',14)

    % average time
    subplot(4,2,2+gp*(gp-1)+2*gp)
    bar(dir1,AVE_tave(:,1:4,2)',1)
    set(gca, 'XTickLabel',Mass, 'XTick',1:numel(dir1))
    ngroups = size(AVE_tave(:,1:4,2)', 1);
    nbars = size(AVE_tave(:,1:4,2)', 2);
    % Calculating the width for each bar group
    groupwidth = min(0.8, nbars/(nbars + 1.5));
    hold on
    y=AVE_tave(:,1:4,2)';
    error= STD_tave(:,1:4,2)';
    for i = 1:nbars
        x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
        errorbar(x,y(:,i),error(:,i) , '.k');
    end
    axis([0 5 0 2700])
    %legend('0','-20','-30','-40','-50')
    if gp ==1
        xlabel('Inertia');
    else
        xlabel('End velocity of perturbation');
    end
    %ylabel('Average time to regain stability (ms)')
end


%%
format short g
% Making the table for difference rate from original study

for gp=1:2
    str1 = ['C:\Users\fzahedi1admin\OneDrive - Arizona State University\Projects\Damping Map\Experiments\Test\SensitivityAnalysis\MV analysis\Data\Groupdata\Group',num2str(gp),'\Average_rate_total.mat'];
    load(str1);

    str2 = ['C:\Users\fzahedi1admin\OneDrive - Arizona State University\Projects\Damping Map\Experiments\Test\SensitivityAnalysis\MV analysis\Data\Groupdata\Group',num2str(gp),'\Average_time_total.mat'];
    load(str2);
    
    % k= 1 is ML and k=2 is AP direction
    for k=1:2
       for i=1:size(rate_total,1) % i is different damping value
            clear M Q
            M(:,:)=rate_total(i,:,k,:);
            Q=M';
            AVE_rate(i,:,k)=mean(Q,1);
            STD_rate(i,:,k)=std(Q);
       end 

       for i=1:size(tave_total,1) % i is different damping value
            clear M Q
            M(:,:)=tave_total(i,:,k,:);
            Q=M';
            AVE_tave(i,:,k)=mean(Q,1);
            STD_tave(i,:,k)=std(Q);
        end 
    end
    n=0;
    if gp == 1
        origin = 2;
    else
        origin = 3;
    end
    for i=1:4 %different Inertia/velocity
        %if i~=origin
            n = n+1;
            for j=1:5 % different damping
                for f=1:2 % ML or AP direction
                     diff_rate(n,j,f,gp)= round((((abs(AVE_rate(j,i,f)-AVE_rate(j,origin,f)))/((AVE_rate(j,origin,f)+AVE_rate(j,i,f))/2))*100)*10,0)/10;
                     diff_rate(n,j+5,f,gp)= round((((abs(AVE_tave(j,i,f)-AVE_tave(j,origin,f)))/((AVE_tave(j,origin,f)+AVE_tave(j,i,f))/2))*100)*10,0)/10;
                     diff_rate2(n,j,f,gp)= round((((abs(AVE_rate(j,i,f)-AVE_rate(j,origin,f)))/(mean(AVE_rate(j,:,f))))*100)*10,0)/10;
                     diff_rate2(n,j+5,f,gp)= round((((abs(AVE_tave(j,i,f)-AVE_tave(j,origin,f)))/(mean(AVE_tave(j,:,f))))*100)*10,0)/10;
                     diff_rate4(n,j,f,gp)= round((((abs(AVE_rate(j,i,f)-mean(AVE_rate(j,:,f))))/(mean(AVE_rate(j,:,f))))*100)*10,0)/10;
                     diff_rate4(n,j+5,f,gp)= round((((abs(AVE_tave(j,i,f)-mean(AVE_tave(j,:,f))))/(mean(AVE_tave(j,:,f))))*100)*10,0)/10;
                     diff_rate3(n,j,f,gp)= round((((abs(AVE_rate(j,i,f)-AVE_rate(j,origin,f)))/(max(AVE_rate(j,origin,f),AVE_rate(j,i,f))))*100)*10,0)/10;
                     diff_rate3(n,j+5,f,gp)= round((((abs(AVE_tave(j,i,f)-AVE_tave(j,origin,f)))/(max(AVE_tave(j,origin,f),AVE_tave(j,i,f))))*100)*10,0)/10;
                end
            end
        %end
    end
end
% format short g
exceltor1 = diff_rate4(:,:,1,1);
exceltor2 = diff_rate4(:,:,1,2);
exceltor3 = diff_rate4(:,:,2,1);
exceltor4 = diff_rate4(:,:,2,2);
%%

% State Space contraction
clc
clear
close all

for gp=1:2
    str1 = ['C:\Users\fzahedi1admin\OneDrive - Arizona State University\Projects\Damping Map\Experiments\Test\SensitivityAnalysis\MV analysis\Data\Groupdata\Group',num2str(gp),'\Average_eig_total.mat'];
    load(str1);
    
    Meaneig(gp,:) = mean(Total_eig,1);
    STDeig(gp,:) = std(Total_eig);
end

for gp=1:2
 for i=1:8
     AVE_eig(i,:,gp) = Meaneig(gp,5*(i-1)+1:5*i);
     STD_eig(i,:,gp) = STDeig(gp,5*(i-1)+1:5*i);
 end
end
figure
for gp=1:2
    dir1=[1 2 3 4];
    if gp == 2
        Mass={'100mm/s'; '150mm/s'; '200mm/s'; '250mm/s'};
    else
        Mass={'8kg'; '10kg'; '12kg'; '14kg'};
    end
    for i =1:2
        subplot(2,2,i+2*(gp-1))
        bar(dir1,AVE_eig(1+4*(i-1):4*i,:,gp),1)
        set(gca, 'XTickLabel',Mass, 'XTick',1:numel(dir1))
        ngroups = size(AVE_eig(1+4*(i-1):4*i,:,gp), 1);
        nbars = size(AVE_eig(1+4*(i-1):4*i,:,gp), 2);
        % Calculating the width for each bar group
        groupwidth = min(0.8, nbars/(nbars + 1.5));
        hold on
        y=AVE_eig(1+4*(i-1):4*i,:,gp);
        error= STD_eig(1+4*(i-1):4*i,:,gp);
        for j = 1:nbars
            x = (1:ngroups) - groupwidth/2 + (2*j-1) * groupwidth / (2*nbars);
            errorbar(x,y(:,j),error(:,j) , '.k');
        end
        axis([0 5 0 1.1])
        if ((gp == 1) && (i ==1))||((gp == 2) && (i==1))
            ylabel('Eigenvalue')
        end
        if gp ==1 && i==1
            legend('0','-10','-15','-20','-25')
            title('ML direction')
        elseif gp == 1 && i==2
            legend('0','-20','-30','-40','-50')
            title('AP direction')
        end
        if gp==1
            xlabel('Inertia')
        else
            xlabel('End velocity of perturbation')
        end
    end
end

%%

% State Space contraction difference percentage
clc
clear
close all

for gp=1:2
    str1 = ['C:\Users\fzahedi1admin\OneDrive - Arizona State University\Projects\Damping Map\Experiments\Test\SensitivityAnalysis\MV analysis\Data\Groupdata\Group',num2str(gp),'\Average_eig_total.mat'];
    load(str1);
    
    Meaneig(gp,:) = mean(Total_eig,1);
    STDeig(gp,:) = std(Total_eig);
end

for gp=1:2
     for i=1:8
         AVE_eig(i,:,gp) = Meaneig(gp,5*(i-1)+1:5*i);
         STD_eig(i,:,gp) = STDeig(gp,5*(i-1)+1:5*i);
     end
end
nxt=4;
for gp=1:2
    n=0;
    if gp == 1
        origin = 2+nxt;
    else
        origin = 3+nxt;
    end
    for i=1+nxt:4+nxt
        %if i~=origin
            n = n+1;
            for j=1:5
               diff_eig(n,j,gp)= round((((abs(AVE_eig(i,j,gp)-AVE_eig(origin,j,gp)))/((AVE_eig(origin,j,gp)+AVE_eig(i,j,gp))/2))*100)*10,0)/10;
               diff_eig2(n,j,gp)= round((((abs(AVE_eig(i,j,gp)-AVE_eig(origin,j,gp)))/(mean(AVE_eig(:,j,gp))))*100)*10,0)/10;
               diff_eig3(n,j,gp)= round((((abs(AVE_eig(i,j,gp)-AVE_eig(origin,j,gp)))/(max(AVE_eig(origin,j,gp),AVE_eig(i,j,gp))))*100)*10,0)/10;
               diff_eig4(n,j,gp)= round((((abs(AVE_eig(i,j,gp)-mean(AVE_eig(1+nxt:4+nxt,j,gp))))/(mean(AVE_eig(1+nxt:4+nxt,j,gp))))*100)*10,0)/10;
            end
        %end
    end
end
new1 = diff_eig4(:,:,1);
new2 = diff_eig4(:,:,2);

%%

% lowest bound of stability
clear
clc
close all

MLinertia = [-15 -10 -25 -25; -15 -20 0 0; -15 -15 -10 -10; -15 -25 -25 -25; -25 -25 -25 -25; -25 -25 -25 -25];
APinertia = [-40 -50 -50 -50; 0 -30 -40 -20; -30 -30 -30 -50; -50 -50 -50 -50; -40 -50 -40 -50; -50 -50 -40 -40];
MLvelocity = [-25 -25 -25 -25; 0 -15 -20 0; -25 -20 -25 -25; -25 -25 -25 -20; -25 -25 -25 -25; -20 -25 -15 -15];
APvelocity = [-50 -50 -50 -50; -40 -30 -30 -40; -50 -50 -50 -50; -50 -50 -50 -50; -40 -40 -50 -30; -50 -50 -50 -50];
Average(1,:)= mean(MLinertia);
Average(2,:)= mean(APinertia);
Average(3,:)= mean(MLvelocity);
Average(4,:)= mean(APvelocity);
for i=1:4
    if i<=2
        origin = 2;
    else
        origin = 3;
    end
    n=0;
    for j=1:4 % diff condition inertia of velocity
        %if j~=origin
            n=n+1;
            diff_lowest(i,n)= round((((abs(Average(i,j)-Average(i,origin)))/abs((Average(i,origin)+Average(i,j))/2))*100)*100,0)/100;
            diff_lowest2(i,n)= round((((abs(Average(i,j)-Average(i,origin)))/abs(mean(Average(i,:))))*100)*10,0)/10;
            diff_lowest3(i,n)= round((((abs(Average(i,j)-Average(i,origin)))/abs(min(Average(i,origin),Average(i,j))))*100)*10,0)/10;
            diff_lowest4(i,n)= round((((abs(Average(i,j)-mean(Average(i,:))))/abs(mean(Average(i,:))))*100)*10,0)/10;
        %end
    end
end
