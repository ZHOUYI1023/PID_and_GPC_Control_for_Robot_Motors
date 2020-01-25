function [u_past1,y_past1] = Controllaw(r,H,P,Q,lamda,Parameter)
p=Parameter.p;
m=Parameter.m;
n=Parameter.n;
%to avoid the excess of index ,add a series of zeros temporarily
r=[zeros(max(m-1,n),1);r];
delta_u_past=zeros(size(r,1),1);%store delta input
y_past=zeros(size(r,1),1);%store output
u_past=zeros(size(r,1),1);%store input
uu=0;%initialize input 
clear S1
for i=1+max(m-1,n):size(r,1)-p+1
delta_u_predict=(inv(H'*H+lamda*eye(p)))*H'*(r(i+1-1:i+p-1,1)-P*delta_u_past(i-1:-1:i-m+1)-Q*y_past(i-1:-1:i-n+1-1));
uu=uu+delta_u_predict(1,1);%obtain input
delta_u_past(i)=delta_u_predict(1,1);%store delta input 
%get the output from the real system
S1.u = uu;
S1 = systemXXX(S1);
yy = S1.y;
%yy=simulate_system1(u,a,b);
%update input and output
y_past(i,1)=yy;
u_past(i,1)=uu;
end
%get rid of previous added zeros
y_past1=y_past(1+max(m-1,n):end);
u_past1=u_past(1+max(m-1,n):end);
end

