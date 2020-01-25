function [H,P,Q]=Matrix_Step(Parameter,a1)
%calculate H P Q matrix
p=Parameter.p;
m=Parameter.m;
n=Parameter.n;
H=zeros(p,p);
P1=a1(m)*ones(p,m-1);
for i=1:p
    for j=1:p-i+1
    P1(i,j)=a1(i+j,1);
    end
end
for i=1:p
    for j=1:m-1
        P(i,j)=P1(i,j)-a1(j,1);
    end
end
for j=1:p
    for i=j:p
        H(i,j)=a1(i-j+1,1);
    end
end
Q=ones(p,1);
end

