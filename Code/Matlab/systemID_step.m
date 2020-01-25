function h = systemID_step(Y,U,sys_order)
% -------------------------------------------------------------------------
% System ID using least squares method.
%
% y_{k}  = h1*u_{k-1} + h2*u_{k-2} + h3*u_{k-3} + ... + h{nb}*u_{k-nh}
%       + e_{k}

% extract information
nh = sys_order;

% make sure input and output vectors are column vectors
if size(Y,2)>1,     Y = Y';     end
if size(U,2)>1,     U = U';     end

phi = U;
for i=1:nh-1
    t1 = [zeros(i,1);U(1:end-i)];
    phi = [phi t1];
end
% Extract the return vectors
h = phi\Y;
% figure(2)
% plot(Y)
% hold on
% plot(phi*h)
return