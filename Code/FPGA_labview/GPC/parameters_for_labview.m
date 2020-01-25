%caculate parameters
%right
% a=[1;-0.3059;0.0411];
% b=[3.1468;8.4937];
%left
a=[1;-0.2767;0.0033];
b=[2.7981;9.1688];
a1=conv(a,[1,-1]);
a1(1,1)=a1(1,1)-1;
Hor=3;
Parameter.p=Hor;
Parameter.n=size(a1,1);
Parameter.m=size(b,1);
lamda=1500;
[H,P,Q] = Matrix_Carima(Parameter,a1,b);
E=[1,zeros(1,Hor-1)];
Pr=E*(inv(H'*H+lamda*eye(Hor)))*H'
Pp=-E*(inv(H'*H+lamda*eye(Hor)))*H'*P
Pq=-E*(inv(H'*H+lamda*eye(Hor)))*H'*Q