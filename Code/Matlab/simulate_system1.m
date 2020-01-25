function Y = simulate_system1(U,a,b)
% -------------------------------------------------------------------------
% Simulate linear dynamical system.
%
% y_{k} + a1*y_{k-1} + a2*y_{k-2} + a3*y_{k-3} + ... + a{na}*y_{k-na} =
%       + b1*u_{k-1} + b2*u_{k-2} + b3*u_{k-3} + ... + b{nb}*u_{k-nb}
%       + e_{k}
%
% a = [a1 a2 a3 ... a{na}]
% b = [b1 b2 b3 ... b{nb}]
%
% Inputs:
%   <U>     (1 x n)     input signal, <n> number of inputs
%   <a>     (1 x na)	output parameters
%   <b>     (1 x nb)    input parameters
%
% Output:
%   <Y>     (1 x n)     output signal
%
% Mohamed Mustafa, University of Manchester, 2016
% -------------------------------------------------------------------------

% default values
if nargin<3
    error('Bad parameters: missing inputs!')
end

% make sure A and B are row vectors
if size(a,1)>1,     a = a';     end 
if size(b,1)>1,     b = b';     end

% extract data
n = length(U);
na = length(a);
nb = length(b);
a = -a;

% initialize
u_all = [];
y_all = [];

for i=1:n
    % extract info
    u = U(i);
    
    % Fix the length of vectors if needed
    u_all = [u_all u];
    if length(u_all)<nb
        u_all = [zeros(1,nb-length(u_all)) u_all];
    end
    if length(y_all)<na
        y_all = [zeros(1,na-length(y_all)) y_all];
    end
    
    % apply the transfer function
    y = sum(fliplr(a).*y_all(end-na+1:end)) + sum(fliplr(b).*u_all(end-nb+1:end));

    % add some noise to the output
    sigma = 1e-3;
    y = y + sigma*randn;

    % save old inputs and outputs
    u_all = [u_all u];
    y_all = [y_all y];
end
Y = y_all(na+1:end);
return