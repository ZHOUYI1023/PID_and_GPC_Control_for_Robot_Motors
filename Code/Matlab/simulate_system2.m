function Y = simulate_system2(U,h)
% -------------------------------------------------------------------------
% Simulate linear dynamical system.
%
% y_{k} =
%       + h1*u_{k-1} + h2*u_{k-2} + h3*u_{k-3} + ... + h{nh}*u_{k-nh}
%       + e_{k}

% extract data
U=U';
n = length(U);
%h=h';
nh = length(h);
phi = U;
for i=1:nh-1
    t1 = [zeros(i,1);U(1:end-i)];
    phi = [phi t1];
end
Y=phi*h;
Y=Y';
sigma = 0*1e-3;
Y=Y+sigma*randn(size(Y,1),size(Y,2));
%{
% initialize
u_all = [];
y_all = [];

for i=1:n
    % extract info
    u = U(i);
    
    % Fix the length of vectors if needed
    u_all = [u_all u];
    if length(u_all)<nh
        u_all = [zeros(1,nh-length(u_all)) u_all];
    end
    
    % apply the transfer function
    y =  sum(fliplr(h).*u_all(end-nh+1:end));

    % add some noise to the output
    sigma = 0*1e-3;
    y = y + sigma*randn;

    % save old inputs and outputs
    u_all = [u_all u];
    y_all = [y_all y];
end
Y = y_all;
return
%}