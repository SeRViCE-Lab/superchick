clc; clear all
cd('~/Documents/IAB8/')

%% Container forthe geometry, structural material properties, 
% body and boundary loads, boundary constraints, and mesh.
model = createpde('structural','static-solid');
% importGeometry(model,fullfile('compoz', 'sphere.stl'));
importGeometry(model,'BracketWithHole.stl');
% plot
close all
figure
pdegplot(model,'FaceLabels','on')
view(30,30);
title('Sphere with Face Labels')

figure
pdegplot(model,'FaceLabels','on')
view(-134,-32)
title('Bracket with Face Labels, Rear View')