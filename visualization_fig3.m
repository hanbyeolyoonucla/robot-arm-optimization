clc; clear; close all;

% planstl = stlread('data/test43/Jayne_tb_14_arch_final_cut_result_mw.stl');
planstl = stlread('data/test38/Jayne_tb_14_arch_final_cut_result.stl');

axial1 = readmatrix('data/test38/new_axial_path_1.csv');
axial2 = readmatrix('data/test38/new_axial_path_2.csv');
axial3 = readmatrix('data/test38/new_axial_path_3.csv');
axial4 = readmatrix('data/test38/new_axial_path_4.csv');
occ = readmatrix('data/test38/new_occlusal_path.csv');
plan = [axial1; axial2; axial3; axial4; occ];

for ii = 1:length(plan)
%     cutpath(:,:,ii) = rt2tr(oa2r([0;1;0],-plan(ii,4:6)), plan(ii,1:3));
    cutpath(:,:,ii) = rt2tr(oa2r([0;1;0],[0;0;-1]), plan(ii,1:3));
end

figure()
view(3)
trimesh(planstl,'Facecolor','k','FaceAlpha',0.3,'Edgecolor','none')
colormap gray
hold on
grid on
axis equal
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]')
xlim([-15 13]); ylim([-10 10]); zlim([-5 6]);
% ax = axes();
% xlim(ax, [-10 10]); ylim(ax, [-10 10]); zlim(ax, [-10 10]);
% view(ax, 3); hold(ax, 'on'); grid(ax, 'on')
for ii = 1:length(plan)
    if mod(ii,10) ==0
        trplot(cutpath(:,:,ii),'rgb','notext','length',2,'thick',3);
    end
end
