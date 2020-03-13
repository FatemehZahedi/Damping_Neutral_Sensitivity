close all
clear
clc

filename = 'C:\Users\fzahedi1admin\OneDrive - Arizona State University\Projects\Damping Map\Experiments\Test\SensitivityAnalysis\Velocitycheck\Data\Subject1\KD_S1_B5.txt';
data=load(filename);


alpha = [pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0];
a = [0, 0, 0, 0, 0, 0, 0];
d = [0.36, 0, 0.42, 0, 0.4, 0, 0.126];
theta=[0 0 0 0 0 0 -0.958709];
dh = [theta' d' a' alpha'];
h=0.001;
s=2000;

% BUILD ROBOT--------------------------------------------------------------
for i = 1:length(dh(:,1))
    L{i} = Link('d', dh(i,2), 'a', dh(i,3), 'alpha', dh(i,4));
end
rob = SerialLink([L{1} L{2} L{3} L{4} L{5} L{6} L{7}]);

qm= data(:,2:7)';

ind= data(s+1:end,18);

ind_trig = data(s+1:end,22);

ind_ind = data(s+1:end,23);

ind_pert = data(s+1:end,24);

%ind_fail = data(s+1:end,25);

for i=1:length(qm)
    theta=[qm(:,i)' -0.958709]; %This should be in both ways
    [T,all] = rob.fkine(theta);
    phi_euler = atan2(T(2, 3), T(1, 3));
    theta_euler = atan2(sqrt(T(2, 3)^2 + T(1, 3)^2), T(3, 3));
    psi_euler = atan2(T(3, 2), -T(3, 1));
    xm1(:,i)=[T(1,4) T(2,4) T(3,4) phi_euler theta_euler psi_euler]';
    %[Ja,x(:,i)]=Jacobian(alpha,a,d,theta);
end
%xm1=data(:,3:8)';
xm=xm1(:,s+1:end);

d1 = designfilt('lowpassiir','FilterOrder',12,'HalfPowerFrequency',20,'DesignMethod','butter','Samplerate',1000);

for i=1:size(xm,1)
    xm(i,:)=filtfilt(d1,xm(i,:));
end

j=1;

ind_tn = find(ind_trig == 0);

j=1;
n=1;
for i=1:length(ind_tn)
    if i < length(ind_tn)
        if ind_tn(i+1)-ind_tn(i) > 1
            clear new
            new=[xm(:,ind_tn(j)-1000:ind_tn(i)+4000); ind(ind_tn(j)-1000:ind_tn(i)+4000)'; ind_ind(ind_tn(j)-1000:ind_tn(i)+4000)'; ind_pert(ind_tn(j)-1000:ind_tn(i)+4000)'];% ind(j:i)];
            content{n}=new;
            perturbation{n} = xm(1:3,ind_tn(j):ind_tn(i));
            n=n+1;
            j = i+1;
        end
    end
    if i==length(ind_tn)
        clear new
        new=[xm(:,ind_tn(j)-1000:ind_tn(i)+4000); ind(ind_tn(j)-1000:ind_tn(i)+4000)'; ind_ind(ind_tn(j)-1000:ind_tn(i)+4000)'; ind_pert(ind_tn(j)-1000:ind_tn(i)+4000)'];% ind(j:i)];
        content{n}=new;
        perturbation{n} = xm(1:3,ind_tn(j):ind_tn(i));
    end
end

for i=1:size(content,2)
    M(:,:,i)=content{i};
    Perturb(:,:,i)=perturbation{i};
end



%%

time=-1000:length(M(1,:,1))-1001;
plot(Perturb(1,:,1)-Perturb(1,1,1))
hold on
plot(Perturb(1,:,2)-Perturb(1,1,2))
plot(Perturb(1,:,3)-Perturb(1,1,3))
plot(Perturb(1,:,4)-Perturb(1,1,4))
plot(Perturb(1,:,5)-Perturb(1,1,5))
V=diff(Perturb(1,:,1))/0.001;
figure
plot(V)
hold on
V=diff(Perturb(1,:,2))/0.001;
plot(V)
V=diff(Perturb(1,:,3))/0.001;
plot(V)
V=diff(Perturb(1,:,4))/0.001;
plot(V)
V=diff(Perturb(1,:,5))/0.001;
plot(V)
xlabel('Time (ms)');
ylabel('velocity (m/s)');

%------------------------------------------------------------

figure
plot(Perturb(1,:,6)-Perturb(1,1,6))
hold on
plot(Perturb(1,:,7)-Perturb(1,1,7))
plot(Perturb(1,:,8)-Perturb(1,1,8))
plot(Perturb(1,:,9)-Perturb(1,1,9))
plot(Perturb(1,:,10)-Perturb(1,1,10))
V=diff(Perturb(1,:,6))/0.001;
figure
plot(V)
hold on
V=diff(Perturb(1,:,7))/0.001;
plot(V)
V=diff(Perturb(1,:,8))/0.001;
plot(V)
V=diff(Perturb(1,:,9))/0.001;
plot(V)
V=diff(Perturb(1,:,10))/0.001;
plot(V)
xlabel('Time (ms)');
ylabel('velocity (m/s)');

%--------------------------------------------------------------

figure
plot(Perturb(3,:,11)-Perturb(3,1,11))
hold on
plot(Perturb(3,:,12)-Perturb(3,1,12))
plot(Perturb(3,:,13)-Perturb(3,1,13))
plot(Perturb(3,:,14)-Perturb(3,1,14))
plot(Perturb(3,:,15)-Perturb(3,1,15))
V=diff(Perturb(3,:,11))/0.001;
figure
plot(V)
hold on
V=diff(Perturb(3,:,12))/0.001;
plot(V)
V=diff(Perturb(3,:,13))/0.001;
plot(V)
V=diff(Perturb(3,:,14))/0.001;
plot(V)
V=diff(Perturb(3,:,15))/0.001;
plot(V)
xlabel('Time (ms)');
ylabel('velocity (m/s)');

%----------------------------------------------------------------

figure
plot(Perturb(3,:,16)-Perturb(3,1,16))
hold on
plot(Perturb(3,:,17)-Perturb(3,1,17))
plot(Perturb(3,:,18)-Perturb(3,1,18))
plot(Perturb(3,:,19)-Perturb(3,1,19))
plot(Perturb(3,:,20)-Perturb(3,1,20))
V=diff(Perturb(3,:,16))/0.001;
figure
plot(V)
hold on
V=diff(Perturb(3,:,17))/0.001;
plot(V)
V=diff(Perturb(3,:,18))/0.001;
plot(V)
V=diff(Perturb(3,:,19))/0.001;
plot(V)
V=diff(Perturb(3,:,20))/0.001;
plot(V)
xlabel('Time (ms)');
ylabel('velocity (m/s)');

