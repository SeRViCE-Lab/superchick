close all; clear all; clc

% Mechanical parameters
joint_space=10.95; % degree
j_posi=0.06329;
normal_leg_length=150;
nnoo= 0.1441980707446314; % z-coordinate when all lengths of leg are 150mm.

% Joints in the lower/upper panel.
tt=[0 0 120 120 240 240]+90;
t3=tt+[-1 1 -1 1 -1 1]*joint_space; t3=t3*pi/180;
t4=j_posi*cos(t3); t5=j_posi*sin(t3);
j_w=[t4; t5; zeros(1,6)]; j_ww=[j_w,j_w(:,1)];
tt=tt-60;
t3=tt+[-1 1 -1 1 -1 1]*joint_space; t3=t3*pi/180;
t3=t3([2:6,1]);
t4=j_posi*cos(t3); t5=j_posi*sin(t3);
j_u_2=[t4; t5; zeros(1,6)]; % joint positions of the top panel in the top panel frame.

u_psi_be=[0 0 0]/180*pi; u_xyz_be=[0 0 nnoo];
u_psi_de=[0 0 0]/180*pi; u_xyz_de=[0 0 nnoo];

d_xyz=[ 
   0 0 0
   1 1 1
   1 -1 1
   -1 -1 1
   -1 1 1
   -1 1 -1
   -1 -1 -1
   1 -1 -1
   1 1 -1
   0 0 0
   0 0 0]/1000*7;

% d_xyz=[ 
%    0 0 0
%    1 1 1
%    0 0 0
%    0 0 0]/1000*5;
% 
% d_psi=[
%    0 0 0
%    3 5 7
%    0 0 0
%    0 0 0]/180*pi;
d_psi=d_xyz*0;
tu=20; % time to the next point.
tg=0.2; % time step in text file;
ts=90; % time remains the same value.
NNN=round(tu/tg);
MMM=round(ts/tg);

LL=[]; 
si1=size(d_xyz,1);
tot_t=(si1-1)*tu+(si1-2)*ts;

for j=1:si1-1
   u_xyz_be=d_xyz(j,:)+[0 0 nnoo];
   u_xyz_de=d_xyz(j+1,:)+[0 0 nnoo];
   u_psi_be=d_psi(j,:);
   u_psi_de=d_psi(j+1,:);
   
   for i=1:3
       u_xyz_t(i,:)=linspace(u_xyz_be(i),u_xyz_de(i),NNN);
       u_psi_t(i,:)=linspace(u_psi_be(i),u_psi_de(i),NNN);
   end
   
   for iii=1:NNN
      psi=u_psi_t(:,iii); xyz=u_xyz_t(:,iii);
      L_aaa(iii,:)=D6_to_leg_length_123(xyz,psi,j_w,j_u_2);
   end

   tt=L_aaa*1000-normal_leg_length;
   if j~=si1-1
      tt=[tt;repmat(tt(end,:),MMM,1)];
   end
   LL=[LL;tt];
end
LL=[LL(1,:);LL];

tt=linspace(0,tot_t,size(LL,1));
ttt=[tt',LL];
dlmwrite('hex_input.txt', ttt, 'precision', '%.6f', 'delimiter', '\t', 'newline', 'pc')
t1=diff(ttt)/tg;
total_time_second=tot_t
max_length_change_mm=max(abs(ttt(:,2:end)))
max_speed_mm_per_second=max(abs(t1(:,2:end)))
