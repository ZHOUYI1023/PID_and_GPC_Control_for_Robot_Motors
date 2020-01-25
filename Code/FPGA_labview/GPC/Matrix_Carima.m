function [H,P,Q] = Matrix_Carima(Parameter,a1,b)
%calculate H P Q matrix
p=Parameter.p;
m=Parameter.m;
n=Parameter.n;
Ca=eye(p);
Ha=zeros(p,n);
Cb=zeros(p);
Hb=zeros(p,m-1);
for j=1:p
    index1=1;
    index2=1;
    index3=1;
    index4=1;
    for i=j+1:min(j+n,p)
        Ca(i,j)=a1(index1,1);
        index1=index1+1;
    end
    for i=j:min(j+m-1,p)
        Cb(i,j)=b(index2,1);
        index2=index2+1;
    end
    for i=1:n-j+1
        Ha(j,i)=a1(index3+j-1);
        index3=index3+1;
    end
    for i=1:m-j
        Hb(j,i)=b(index4+j);
        index4=index4+1;
    end
end
H=Ca^(-1)*Cb;
P=Ca^(-1)*Hb;
Q=-Ca^(-1)*Ha;
end

