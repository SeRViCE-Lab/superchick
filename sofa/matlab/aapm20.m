%% Process saved data
clc; close all
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

subplot(311)
plot(x, 'k*' );
legend('X', 'FontSize', 20, 'FontWeight', 'bold' ,'location', 'best')
ylabel('Head Displacement (mm)',  'FontSize', 12, 'FontWeight', 'bold')
ylim([min(y)-5, max(y)+20])
grid('on')

subplot(312)
plot(y,  'b*' );
legend('Y', 'FontSize', 20, 'FontWeight', 'bold', 'location', 'best');
ylabel('Head Displacement (mm)',  'FontSize', 12, 'FontWeight', 'bold')
ylim([min(y)-5, max(y)+20]); grid('on')

subplot(313)
plot(z, 'g*' );
legend('Z',  'FontSize', 20, 'FontWeight', 'bold','location', 'best' )
xlabel('Time (mSecs)',  'FontSize', 20, 'FontWeight', 'bold')
ylabel('Head Displacement (mm)',  'FontSize', 12, 'FontWeight', 'bold')
ylim([min(y)-5, max(y)+20]); grid('on')