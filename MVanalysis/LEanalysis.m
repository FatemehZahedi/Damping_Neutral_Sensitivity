clear
clc
close all

gp = 2;
num_sub =6;

% w={};
% save Average_eig_total.mat w;

filename = 'C:\Users\fzahedi1admin\OneDrive - Arizona State University\Projects\Damping Map\Experiments\Test\SensitivityAnalysis\MV analysis\Data\Subject7\Group2\Subject7_1.mat';
load(filename);


m=cel{1};
M1=zeros(size(m,1),size(m,2),3,20);
M2=zeros(size(m,1),size(m,2),3,20);
clear m
dt=0.001;
dim =50;
err=0.09;
radius_e = 0.005;
d1 = designfilt('lowpassiir','FilterOrder',12,'HalfPowerFrequency',20,'DesignMethod','butter','Samplerate',1000);

for i=1:10
    
    str1 = ['C:\Users\fzahedi1admin\OneDrive - Arizona State University\Projects\Damping Map\Experiments\Test\SensitivityAnalysis\MV analysis\Data\Subject7\Group2\Subject7_',num2str(i),'.mat'];
    data = load(str1);
    
    clear cel
    load(str1);

    for j=1:size(cel,2)

      if size(cel{j},1)~= 0  
          clear m
          m=cel{j};

          M1(:,:,i,j) = m;

      end
    end
end

for i=1:size(M1,3)
    for j=1:size(M1,4)
        Vel(1,:,i,j)=diff(M1(1,:,i,j))/dt;
        Vel(1,:,i,j)=filtfilt(d1,Vel(1,:,i,j));
        Vel(2,:,i,j)=diff(M1(3,:,i,j))/dt;
        Vel(2,:,i,j)=filtfilt(d1,Vel(2,:,i,j));
    end
end

for i=1:size(M1,4)
   Ave(1,:,i)=mean(M1(1,:,:,i),3);
   Ave(2,:,i)=mean(M1(3,:,:,i),3);
end

for i=1:size(M1,4)
    Vel_av(1,:,i)=diff(Ave(1,:,i))/dt;
    Vel_av(1,:,i)=filtfilt(d1,Vel_av(1,:,i));
    Vel_av(2,:,i)=diff(Ave(2,:,i))/dt;
    Vel_av(2,:,i)=filtfilt(d1,Vel_av(2,:,i));
end

n = 1;
sn = 1;
for c=1:2
    
    if c == 1
        desired = 0;
    else
        desired = 0.76;
    end
    
    for i=1:20

        clear M
        if c==1
            for l=1:size(M1,3)

                M(l,:)=M1(1,:,l,i);
                M_v(l,:) = Vel(1,:,l,i);
            end
        else
            for l=1:size(M1,3)

                M(l,:)=M1(3,:,l,i+20);
                M_v(l,:) = Vel(2,:,l,i+20);
            end
        end

        M(~any(M,2),:)=[];
        M_v(~any(M_v,2),:)=[];
    
        clear Position Posi Position1
        for k=1:size(M,1)

            clear E ind_t Position velocity ei1 ei2 t_n Position1 Posi vel velocity1
            E = M(k,size(M,2)-3000+1:end)-desired;
            ind_t = find(abs(E) <= radius_e);
            if length(ind_t)>0  

                q = 1;
                flag = 0;
                for f=1:length(ind_t)

                    if f < length(ind_t) && flag == 0

                        if ind_t(f+1)-ind_t(f) > 1

                            e_length = f - q;

                            if e_length >= 500

                                t_n = ind_t(q);
                                flag = 1;
                            else
                                q = f+1;
                            end
                        end
                    end

                    if f == length(ind_t) && flag == 0

                        e_length = f - q;

                        if e_length >= 500

                            t_n = ind_t(q);

                        else

                            t_n = 3000; 
                        end
                    end
                end
            else
                t_n = 3000;
            end
            if t_n < 3000
                Posi = M(k,size(M,2)-3000+1:size(M,2)-3000+t_n-1+500);
                vel = M_v(k,size(M,2)-3000+1:size(M,2)-3000+t_n-1+500);
            else
               Posi = M(k,size(M,2)-3000+1:size(M,2)-3000+t_n-1);
               vel = M_v(k,size(M,2)-3000+1:size(M,2)-3000+t_n-1);
            end

            if c==2
                Posi=Posi-desired;
            end
            
            clear pos Velocity_final
            count=1;
            for h=1:dim:size(Posi,2)
                if size(Posi,2)-count*dim > dim
                    Position(1,count)=mean(Posi(1,h:h+dim-1));
                    velocity(1,count)=mean(vel(1,h:h+dim-1));
                else
                    Position(1,count)=mean(Posi(1,h:end));
                    velocity(1,count)=mean(vel(1,h:end));
                end
                count=count+1;
            end
            
            Position1(1,:)=2*((Position(1,:)-min(Position(1,:)))/(max(Position(1,:))-min(Position(1,:))))-1;
            velocity1(1,:)=2*((velocity(1,:)-min(velocity(1,:)))/(max(velocity(1,:))-min(velocity(1,:))))-1;
            

            
            q2=0;
            q=1;
            clear pos Velocity_final
            for h=1:size(Position,2)
                if h==1
                    pos(1,1)=Position(1,1);
                    Velocity_final(1,1)=velocity(1,1);
                    col=2;
                end
                flag=0;
                if h+q2 >= size(Position,2)
                    break;
                end
                    while(flag==0 || q>=2)
                        if h+q2+q < size(Position,2)
                            if sqrt((Position1(1,h+q2+q) - Position1(1,h+q2))^2+(velocity1(1,h+q2+q) - velocity1(1,h+q2))^2) <= err
                                flag=1;
                                q=q+1;
                            else 
                                pos(1,col)=Position(1,h+q2+q);
                                Velocity_final(1,col)=velocity(1,h+q2+q);
                                col=col+1;
                                q2=q2+q-1;
                                q=1;
                                flag=1;
                            end
                        else
                            pos(1,col)=Position(1,end);
                            Velocity_final(1,col)=velocity(1,end);
                            q2=size(Position,2);
                            flag=1;
                            q=1;
                        end
                    end
            end
            if (c==1) &&(i==1 || i==5 || i==6 || i==10)&& k==3
                sample_position{sn}=Posi(1,:);
                sample_velocity{sn}=vel(1,:);
                sample_pos_cons{sn}=pos(1,:);
                sample_vel_cons{sn}=Velocity_final(1,:);
                sn=sn+1;
            end
            [ei1,junk1,junk2,stats] = regress(pos(1,2:end)',[pos(1,1:end-1)', Velocity_final(1,1:end-1)']);
            [ei2,junk1,junk2,stats] = regress(Velocity_final(1,2:end)',[pos(1,1:end-1)', Velocity_final(1,1:end-1)']);
            Jacob(:,:,k) = [ei1(1) ei1(2); ei2(1) ei2(2)];
        end 
    Jf{n} = Jacob;
    n=n+1;
    end
end

sn=1;
count_pos=0;
for i=1:size(Jf,2)
    clear Q E
    Q = Jf{i};
    for j=1:size(Q,3)
        E(j)=max(abs(eig(Q(:,:,j))));
        if (i==1 || i==5 || i==6 || i==10)&& j==3
            sample_eig(sn)=E(j);
            sn=sn+1;
        end
        if E(j)>1
            count_pos=count_pos+1;
        end
    end
    Eg(i)=mean(E);
end

load('Average_eig_total.mat');
Total_eig(num_sub,:)=Eg(:);
save Average_eig_total.mat Total_eig


figure
for i=1:4
    desired=0;
    subplot(2,2,i)
    user_p=sample_position{i};
    user_v=sample_velocity{i};
    user_p_c=sample_pos_cons{i};
    user_v_c=sample_vel_cons{i};
    plot(user_p,user_v,'r');
    hold on
    plot(user_p_c,user_v_c,'b.');
    if i==3 || i==4
        xlabel('Position (m)')
    end
    if i==1 || i==3
        ylabel('Velocity (m/s)')
    end
    box off
    set(gca,'FontSize',10)
    if i==1
        title('Neutral');
        annotation('textbox', [0.01, 0.95, 0, 0], 'string', 'A','fontweight','bold','fontsize',14)
    end
    if i==2
        title('Unstable')
        annotation('textbox', [0.01, 0.5, 0, 0], 'string', 'B','fontweight','bold','fontsize',14)
    end
    %axis([-0.04 0.09 -0.5 0.4]);
    h = line(nan, nan, 'Color', 'none');
    legend(h, ['Eig=',num2str(round(sample_eig(i)*1000,0)/1000)])
    legend boxoff

end
            