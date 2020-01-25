%Using  Finite Impulse Response Model to simulate the DC motor
%Yi Zhou ID:9971603 
%zhouyi1023@tju.edu.cn
%Msc Control, University of Manchester
%%
clear
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
    % (1) Define input signal
    u = 1;          % Step signal
    % (2) Apply to the system
    S1.u = u;
    S1 = systemXXX(S1);
    y = S1.y;
    % save old inputs and outputs
    u_all = [u_all u];
    y_real = [y_real y];
end
% plotting
t = Ts*(1:length(u_all));
figure(1), clf
subplot(2,1,1)
plot(t,u_all,'b-')
ylabel('Input (u)'),     xlabel('Time (sec)')

subplot(2,1,2)
plot(t,y_real,'r-')
ylabel('Output (y)'),    xlabel('Time (sec)')
%%
ind=1;
for s=1:10
% (4) System ID
h_order =s ;      % system order
[ho] = systemID_step(y_real, u_all, h_order);

% (5) simulate the identified system
y_sim = simulate_system2(u_all,ho);

% (6) compute the mean square error (MSE)
e = y_real - y_sim;
mse1(ind) = sum(e.*e)/length(e);
ind=ind+1;

end
figure(2)
plot([1:length(mse1)],mse1,'o-')
ylabel('mse')
hold on
%% System identification
h_order = 6;      % system order
[h] = systemID_step(y_real, u_all, h_order);
%simulate the identified system
y_sim = simulate_system2(u_all,h);
figure(1)
hold on
plot(t,y_sim,'b');
legend('real','simulated')
%compute the mean square error (MSE)
e = y_real - y_sim;
mse= sum(e.*e)/length(e)
%try the identified system against the real system with step signals.
u_all = [ones(1,30) -ones(1,10) ones(1,25) -2*ones(1,40)];  % input signals
%response from real system
y_real = [];
clear S1;   % reset the system
for i=1:length(u_all)
    % Define input signal
    u = u_all(i);
    %Apply to unkown plant
    S1.u = u;
    S1 = systemXXX(S1);
    y = S1.y;
    % save old outputs
    y_real = [y_real y];
end
%response from identified system
y_sim = simulate_system2(u_all,h);

% plot responses from real system and identified system
figure(3)
clf
t = Ts*(1:length(u_all));       % x-axis
subplot(2,1,1)
plot(t,u_all,'b-')
ylabel('Input (u)'),xlabel('Time (sec)')
ylim(1.2*[min(u_all) max(u_all)])

subplot(2,1,2)
plot(t,y_real,'r-',t,y_sim,'b-')
ylabel('Output (y)'),    xlabel('Time (sec)')
legend('real','simulated')
figure(7)
plot(t,y_sim)
hold on
%% Calculate some parameters
t1=Ts*[1:size(r,1)]';
m=size(h,1);
a1(1,1)=h(1);
if m>1
for i=2:m
a1(i,1)=a1(i-1,1)+h(i);
end
end
Parameter.n=1;
Parameter.m=m;
%% study effect of horizon 
figure(4)
clf
plot(t1,r)
ylabel('Output (y)'),xlabel('Time (sec)')
hold on
for h1=1:2:m-1
Hor=h1;
Parameter.p=Hor;
[H,P,Q]=Matrix_Step(Parameter,a1);
[~,y_past]=Controllaw(r,H,P,Q,1,Parameter);
figure(4)
plot(t1,y_past);
end
%% Calculate H P Q
Hor=5;
Parameter.p=Hor;
[H,P,Q]=Matrix_Step(Parameter,a1);
%% study the effect of lamda
figure(5)
clf
plot(t1,r)
hold on
axis([0,max(t1),-2,4])
ylabel('Output (y)'),xlabel('Time (sec)')
for l=1:20:101
    lamda=(l-1)*0.125;
    [~,y_past]=Controllaw(r,H,P,Q,lamda,Parameter);
    figure(5)
    hold on
    plot(t1,y_past);
end
%% plot the results
[u_past,y_past]=Controllaw(r,H,P,Q,1,Parameter);
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