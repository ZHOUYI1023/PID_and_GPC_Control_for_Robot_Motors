%Using CARRIMA Model to simulate the DC motor
%Yi Zhou ID:9971603 
%zhouyi1023@tju.edu.cn
%Msc Control, University of Manchester
%% 
clear
clf
addpath(genpath('Functions'))
% Simulation Parameters
Ts = 0.1;   % Sampling time (seconds)
N = 150;    % Number of time steps
% GPC setpoints
r = zeros(N,1);
r(20:50) = 1;   r(50:80) = 3;   r(80:120) = 2;
% initialization
u_all = [];
y_real = [];
clear S1;   % reset the system
for i=1:N
    % Define input signal
    u = PRBS;       % Pseudo Random Binary Signal
    % Apply to the system
    S1.u = u;
    S1 = systemXXX(S1);
    y = S1.y;  
    % save old inputs and outputs
    u_all = [u_all u];
    y_real = [y_real y];
end
%plotting
t = Ts*(1:length(u_all));
figure(1)
clf
subplot(2,1,1)
plot(t,u_all,'b-')
ylabel('Input (u)'),     xlabel('Time (sec)')

subplot(2,1,2)
plot(t,y_real,'r-')
ylabel('Output (y)'),    xlabel('Time (sec)')
%% the effect of sys order 
figure(2)
clf
for a_o=1:10
    ind=1;
for b_o=1:10
 
% System ID
sys_order = [a_o b_o];      % system order
[a,b] = systemID(y_real, u_all, sys_order);
% simulate the identified system
y_sim = simulate_system1(u_all,a,b);
% compute the mean square error (MSE)
e = y_real - y_sim;
mse(ind) = sum(e.*e)/length(e);
mse1(a_o,b_o) = sum(e.*e)/length(e);
ind=ind+1;
end
if(a_o<5)
 figure(2)
 subplot(4,1,a_o)
 plot([1:length(mse)],mse,'x-')
 ylabel('mse')
 hold on
end
end
figure(9)
[X,Y]=meshgrid([1:a_o],[1:b_o]);
contourf(X,Y,mse1)
% surf([1:a_o],[1:b_o],mse1)
%% System Identification
sys_order = [3 1];      % system order
[a,b] = systemID(y_real, u_all, sys_order);
%simulate the identified system
y_sim = simulate_system1(u_all,a,b);
%compute the mean square error (MSE)
e = y_real - y_sim;
mse = sum(e.*e)/length(e)
%ploting
figure(1)
hold on
plot(t,y_sim,'b');
legend('real','simulated')
%try the identified system against the real system with step signals.
u_all = [ones(1,30) -ones(1,10) ones(1,25) -2*ones(1,40)];  % input signals
%response from real system
y_real = [];
clear S1;   % reset the system
for i=1:length(u_all)
    %Define input signal
    u = u_all(i);
    %Apply to unkown plant
    S1.u = u;
    S1 = systemXXX(S1);
    y = S1.y;
    % save outputs
    y_real = [y_real y];
end
%response from identified system
y_sim = simulate_system1(u_all,a,b);
% plot responses from real system and identified system
t = Ts*(1:length(u_all));
figure(3)
clf
subplot(2,1,1)
plot(t,u_all,'b-')
ylabel('Input (u)'),xlabel('Time (sec)')
ylim(1.2*[min(u_all) max(u_all)])

subplot(2,1,2)
plot(t,y_real,'r-',t,y_sim,'b-')
ylabel('Output (y)'),    xlabel('Time (sec)')
legend('real','simulated')
%% Calculate some parameters
a1=conv(a,[1,-1]);
a1(1,1)=a1(1,1)-1;
Hor=3;%horizon
Parameter.p=Hor;
Parameter.n=size(a1,1);
Parameter.m=size(b,1);
t1=Ts*[1:size(r,1)]';
%% study effect of horizon 
figure(4)
clf
plot(t1,r)
ylabel('Output (y)'),xlabel('Time (sec)')
hold on
for h1=1:5
Hor=h1;
Parameter.p=Hor;
[H,P,Q]=Matrix_Carima(Parameter,a1,b);
[~,y_past]=Controllaw(r,H,P,Q,1,Parameter);
figure(4)
plot(t1,y_past);
end
%% Calculate H P Q
Hor=3;%horizon
Parameter.p=Hor;
[H,P,Q] = Matrix_Carima(Parameter,a1,b);
%% study the effect of lamda
figure(5)
clf
subplot(2,1,2)
plot(t1,r)
hold on
axis([0,max(t1),-2,4])
ylabel('Output (y)'),xlabel('Time (sec)')
figure(5)
subplot(2,1,1)
plot(t1,r);
hold on
for l=1:20:81
    lamda=(l-1)*0.125;
    [u_past,y_past]=Controllaw(r,H,P,Q,lamda,Parameter);
    figure(5)
    subplot(2,1,2)
    hold on
    plot(t1,y_past);
    figure(5)
    subplot(2,1,1)
    plot(t1,u_past);
    hold on
end
%% plot the results
[u_past,y_past]=Controllaw(r,H,P,Q,1.2,Parameter);
figure(6)
clf
subplot(2,1,1)
plot(t1,u_past,'b')
ylabel('Input (u)'),xlabel('Time (sec)')

subplot(2,1,2)
plot(t1,r,'r');
hold on
plot(t1,y_past,'b');
ylabel('Output (y)'),xlabel('Time (sec)')
legend('reference','simulated')