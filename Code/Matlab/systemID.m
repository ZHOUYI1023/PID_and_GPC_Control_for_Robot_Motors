function [a,b] = systemID(Y,U,sys_order)
% -------------------------------------------------------------------------
% System ID using least squares method.
%
% y_{k} + a1*y_{k-1} + a2*y_{k-2} + a3*y_{k-3} + ... + a{na}*y_{k-na} =
%       + b1*u_{k-1} + b2*u_{k-2} + b3*u_{k-3} + ... + b{nb}*u_{k-nb}
%       + e_{k}
%
% a = [a1 a2 a3 ... a{na}]
% b = [b1 b2 b3 ... b{nb}]
% U = [u_1 u_2 u_3 ... u_n]
% Y = [y_1 y_2 y_3 ... y_n]
%
% Mohamed Mustafa, University of Manchester, 2016
% -------------------------------------------------------------------------


if nargin<3
    sys_order = [2 2];
    if nargin<2
        error('Input and output vectors are missing!')
    end
end

% extract information
na = sys_order(1);
nb = sys_order(2);

% make sure input and output vectors are column vectors
if size(Y,2)>1,     Y = Y';     end
if size(U,2)>1,     U = U';     end

phi_1 = [];
for i=1:na
    t1 = [zeros(i,1);Y(1:end-i)];
    phi_1 = [phi_1 t1];
end

phi_2 = U;
for i=1:nb-1
    t1 = [zeros(i,1);U(1:end-i)];
    phi_2 = [phi_2 t1];
end

Phi = [phi_1 phi_2];
%Phi = [[zeros(1,1);y(1:end-1)], [zeros(2,1);y(1:end-2)], u, [zeros(1,1);u(1:end-1)]]

theta = Phi\Y;

% Extract the return vectors
a = -theta(1:na);
b = theta(na+1:end);
return