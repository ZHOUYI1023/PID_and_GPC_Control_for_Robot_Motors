function u = PRBS(n)
% -------------------------------------------------------------------------
% Pseudo Random Binary Signal Generator
% <n> is the number of samples to generate
%
% Mohamed Mustafa, University of Manchester, 2016
% -------------------------------------------------------------------------


% default values
if nargin<1
    n = 1;
end
u = double(rand(1,n)<0.5);%0 or 1
u(u==0) = -1;%0 or -1
return