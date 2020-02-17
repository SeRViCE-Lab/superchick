data = dlmread('output_cali_translational.txt','\t',1,0);

if isa(data,'double')

    
flagpoints=data(:,24);
numpoints=max(flagpoints);
datalength=length(data);
    
for ii = 1:numpoints
    kk=1;
    for jj = 1:datalength
        if flagpoints(jj) == ii
            x(kk,ii)=data(jj,18);
            y(kk,ii)=data(jj,19);
            z(kk,ii)=data(jj,20);
            kk=kk+1;
        end
    end
    xm=mean(x(:,ii));
    ym=mean(y(:,ii));
    zm=mean(z(:,ii));
    p{ii}=[xm  ym  zm]';
end
    
% Input points - try to automate
L=([ 
   0 0 0
   1 1 1
   0 0 1
   -1 1 1
   -1 -1 1
   1 -1 1
   1 0 0
   1 1 -1
   0 1 0
   -1 1 -1
   -1 0 0
   -1 -1 -1
   0 -1 0
   1 -1 -1
   0 0 -1])*7.5;

L=L';
A = L';
T=ipdm(A);

P=[p{1} p{2} p{3} p{4} p{5} p{6} p{7} p{8} p{9} p{10} p{11} p{12} p{13} p{14} p{15}];
B = P';
Y=ipdm(B);

% rotTest = [1 0 0; 0 cos(pi()/4) -sin(pi()/4); 0 sin(pi()/4) cos(pi()/4)];
% tTest = repmat(rand(3,1),1,15)';
% B = (A+tTest) * rotTest;

mean1 = mean(A);
mean2 = mean(B);

C = A - repmat(mean1,numpoints,1);
D = B - repmat(mean2,numpoints,1);

E = C' * D;
[U,S,V] = svd(E);
R = U * [1 0 0; 0 1 0; 0 0 det(U*V')] * V';

newLocs = ((A * R')+repmat(mean1,numpoints,1)-repmat(mean2,numpoints,1));

for ii=1:numpoints
    newLocs(ii,:)=(R*B(ii,:)')'+(mean1-(R*mean2')');
end

format long
fprintf('Rotation matrix:\n');
R
fprintf('Translation:\n');
trans=mean1-(R*mean2')'


noTransTest = C' - R * D';
transTest = A' - R * B' + repmat(trans',1,15);


R * P(:,1) + (mean1-(R*mean2')')';

% R-rotTest
% tet=tTest(1,:)-trans


else
    error('The data format is not correct')
end