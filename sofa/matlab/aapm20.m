%% Process saved data
clc; close all
%cd('/Users/olalekanogunmolu/ros2_ws/src/superchicko/sofa/matlab')
cd('/home/lex/catkin_ws/src/superchicko/sofa/matlab')
pat_dofs = load('../python/data_dumps/patient_dofs_x.txt');
pat_dofs_y = load('../python/data_dumps/patient_dofs_y.txt');
pat_dofs_z = load('../python/data_dumps/patient_dofs_z.txt');

% organize data
[x, y, z] = deal([], [], []);
scale = 68.88;
scale_y = 308.05575;
scale_z = 471.37325;

for i=1:length(pat_dofs)
    x = [x, pat_dofs(i)-scale];
    y = [y, pat_dofs_y(i)-scale_y];
    z = [z, pat_dofs_z(i)-scale_z];    
end

FontSize = 30;
ax = gca;
ax.FontSize = FontSize;
ax.FontWeight = 'bold';

%subplot(131)
figure(1)
plot(x, 'r-*' , 'linewidth', 18 );
legend('X', 'FontSize', FontSize, 'FontWeight', 'bold' ,'location', 'best')
xlabel('Time (mSecs)',  'FontSize', FontSize, 'FontWeight', 'bold')
ylabel('Head Displacement (mm)',  'FontSize', FontSize, 'FontWeight', 'bold')
title('X-Displacement with Base Actuators', 'FontSize', FontSize, 'FontWeight', 'bold')
ax = gca;
ax.FontSize = FontSize;
ax.FontWeight = 'bold';
ylim([min(y)-5, max(y)+20])
grid('on')

%subplot(132)
figure(2)
plot(y,  'b--o' , 'linewidth', 18 ); hold on
legend('Y', 'FontSize', FontSize, 'FontWeight', 'bold', 'location', 'best');
xlabel('Time (mSecs)',  'FontSize', FontSize, 'FontWeight', 'bold')
ylabel('Head Displacement (mm)',  'FontSize', FontSize, 'FontWeight', 'bold')
title('Y-Displacement with Base Actuators', 'FontSize', FontSize, 'FontWeight', 'bold')
ax = gca;
ax.FontSize = FontSize;
ax.FontWeight = 'bold';
ax = gca;
ax.FontSize = FontSize; 
ax.FontWeight = 'bold'; 
ylim([min(y)-5, max(y)+20]); grid('on')
hold off


%subplot(133)
figure(3)
plot(z, 'g*', 'linewidth', 18 );
legend('Z',  'FontSize', FontSize, 'FontWeight', 'bold','location', 'best' )
xlabel('Time (mSecs)',  'FontSize', FontSize, 'FontWeight', 'bold')
ylabel('Head Displacement (mm)',  'FontSize', FontSize, 'FontWeight', 'bold')
title('Z-Displacement with Base Actuators', 'FontSize', FontSize, 'FontWeight', 'bold')
ax = gca;
ax.FontSize = FontSize; 
ax.FontWeight = 'bold'; 
ylim([min(y)-5, max(y)+20]); grid('on')