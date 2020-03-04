%% Process saved data
clc; close all; clear all
%cd('/Users/olalekanogunmolu/ros2_ws/src/superchicko/sofa/matlab')
cd('/home/lex/catkin_ws/src/superchicko/sofa/matlab')
pat_dofs = load('../python/data_dumps/patient_dofs.txt');

% organize data
x = pat_dofs(:, 1);
y = pat_dofs(:, 2);
z = pat_dofs(:, 3);

scale = 68.88;
scale_y = 308.05575;
scale_z = 471.37325;

FontSize = 20;

p = plot([x,y,z]);
xlabel('Time (mSecs)',  'FontSize', FontSize, 'FontWeight', 'bold')
ylabel('Head Displacement (mm)',  'FontSize', FontSize, 'FontWeight', 'bold')
title('Head Translation', 'FontSize', FontSize, 'FontWeight', 'bold')
ax = gca;
ax.FontSize = 28; 
ax.FontWeight = 'bold'; 
grid('on')
ylim([min(pat_dofs(:)), max(pat_dofs(:))+10]); grid('on')

p(1).LineWidth = 6;
p(2).LineWidth = 6;
p(2).LineWidth = 6;

p(1).Marker = '.';
p(2).Marker = '*';
p(3).Marker = 'o';

legend('x','y', 'z',  'FontSize', 30, 'FontWeight', 'bold','location', 'best' )