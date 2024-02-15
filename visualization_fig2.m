clc; close all; clear;
addpath('functions');

% solution
x_GA_OPT = [0.226	0.14	0.256	0.09	1.345	0.714	0.05	0.193	0.052];  % GA OPT
x_PSO_OPT = [0.279	0.213	0.230	0.090	-1.310	-0.802	0.210	0.020	0.056];  % PSO OPT
x_GA_MED = [0.261	0.211	0.083	0.133	-1.441	-0.335	0.079	0.317	0.115];  % GA MED
x_PSO_MED = [0.257	0.204	0.098	0.123	-1.407	-0.316	0.071	0.318	0.118];  % PSO MED
x_GA_STIFF = [0.189	0.16	0.116	0.09	1.571	-0.198	-0.05	0.232	0.047];  % GA STIFF
x_PSO_STIFF = [0.149	0.147	0.153	0.090	1.188	1.052	0.144	0.109	-0.004];  % PSO STIFF
x_GA_MECA_all = [1.125	-1.053	-0.093	0.209	0.139];  % MECA
x_PSO_MECA_all = [1.505	0.878	0.218	0.005	0.091];  % MECA
x_MECA_maxilla = [1.199	-0.399	0.129	-0.158	0.102];  % MECA_maxilla

% stiffness
% (X, mandibleOn, weight_m, weight_r, weight_s)
stiffness_GA = -PerformanceIndexFunction_fig(x_GA_OPT, 1, 0, 0, 1);
stiffness_PSO = -PerformanceIndexFunction_fig(x_PSO_OPT, 1, 0, 0, 1);
stiffness_GA_MED = -PerformanceIndexFunction_fig(x_GA_MED, 1, 0, 0, 1);
stiffness_PSO_MED = -PerformanceIndexFunction_fig(x_PSO_MED, 1, 0, 0, 1);
stiffness_GA_STIFF = -PerformanceIndexFunction_fig(x_GA_STIFF, 1, 0, 0, 1);
stiffness_PSO_STIFF = -PerformanceIndexFunction_fig(x_PSO_STIFF, 1, 0, 0, 1);
stiffness_MECA = -PerformanceIndexFunction_MECA_fig(x_GA_MECA_all, 0, 0, 0, 1);

% ISO avg - manipulability
manipulability_GA = -PerformanceIndexFunction_fig(x_GA_OPT, 1, 1, 0, 0);
manipulability_PSO = -PerformanceIndexFunction_fig(x_PSO_OPT, 1, 1, 0, 0);
manipulability_GA_MED = -PerformanceIndexFunction_fig(x_GA_MED, 1, 1, 0, 0);
manipulability_PSO_MED = -PerformanceIndexFunction_fig(x_PSO_MED, 1, 1, 0, 0);
manipulability_GA_STIFF = -PerformanceIndexFunction_fig(x_GA_STIFF, 1, 1, 0, 0);
manipulability_PSO_STIFF = -PerformanceIndexFunction_fig(x_PSO_STIFF, 1, 1, 0, 0);
manipulability_GA_MECA = -PerformanceIndexFunction_MECA_fig(x_GA_MECA_all, 1, 1, 0, 0);
manipulability_PSO_MECA = -PerformanceIndexFunction_MECA_fig(x_PSO_MECA_all, 1, 1, 0, 0);
manipulability_MECA_maxilaonall = -PerformanceIndexFunction_MECA_fig(x_MECA_maxilla, 1, 1, 0, 0);

% ISO min - singularity
singularity_GA = -PerformanceIndexFunction_fig(x_GA_OPT, 1, 0, 1, 0);
singularity_PSO = -PerformanceIndexFunction_fig(x_PSO_OPT, 1, 0, 1, 0);
singularity_GA_MED = -PerformanceIndexFunction_fig(x_GA_MED, 1, 0, 1, 0);
singularity_PSO_MED = -PerformanceIndexFunction_fig(x_PSO_MED, 1, 0, 1, 0);
singularity_GA_STIFF = -PerformanceIndexFunction_fig(x_GA_STIFF, 1, 0, 1, 0);
singularity_PSO_STIFF = -PerformanceIndexFunction_fig(x_PSO_STIFF, 1, 0, 1, 0);
singularity_GA_MECA = -PerformanceIndexFunction_MECA_fig(x_GA_MECA_all, 1, 0, 1, 0);
singularity_PSO_MECA = -PerformanceIndexFunction_MECA_fig(x_PSO_MECA_all, 1, 0, 1, 0);
singularity_MECA_maxillaonall = -PerformanceIndexFunction_MECA_fig(x_MECA_maxilla, 1, 0, 1, 0);

% maxilla_ ISO avg - manipulability
manipulability_GA_maxilla = -PerformanceIndexFunction_fig(x_GA_OPT, 0, 1, 0, 0);
manipulability_PSO_maxilla = -PerformanceIndexFunction_fig(x_PSO_OPT, 0, 1, 0, 0);
manipulability_GA_MED_maxilla = -PerformanceIndexFunction_fig(x_GA_MED, 0, 1, 0, 0);
manipulability_PSO_MED_maxilla = -PerformanceIndexFunction_fig(x_PSO_MED, 0, 1, 0, 0);
manipulability_GA_STIFF_maxilla = -PerformanceIndexFunction_fig(x_GA_STIFF, 0, 1, 0, 0);
manipulability_PSO_STIFF_maxilla = -PerformanceIndexFunction_fig(x_PSO_STIFF, 0, 1, 0, 0);
manipulability_GA_MECA_allonmaxilla = -PerformanceIndexFunction_MECA_fig(x_GA_MECA_all, 0, 1, 0, 0);
manipulability_PSO_MECA_allonmaxilla = -PerformanceIndexFunction_MECA_fig(x_PSO_MECA_all, 0, 1, 0, 0);
manipulability_MECA_maxilla = -PerformanceIndexFunction_MECA_fig(x_MECA_maxilla, 0, 1, 0, 0);

% ISO min - singularity
singularity_GA_maxilla = -PerformanceIndexFunction_fig(x_GA_OPT, 0, 0, 1, 0);
singularity_PSO_maxilla = -PerformanceIndexFunction_fig(x_PSO_OPT, 0, 0, 1, 0);
singularity_GA_MED_maxilla = -PerformanceIndexFunction_fig(x_GA_MED, 0, 0, 1, 0);
singularity_PSO_MED_maxilla = -PerformanceIndexFunction_fig(x_PSO_MED, 0, 0, 1, 0);
singularity_GA_STIFF_maxilla = -PerformanceIndexFunction_fig(x_GA_STIFF, 0, 0, 1, 0);
singularity_PSO_STIFF_maxilla = -PerformanceIndexFunction_fig(x_PSO_STIFF, 0, 0, 1, 0);
singularity_GA_MECA_allonmaxilla = -PerformanceIndexFunction_MECA_fig(x_GA_MECA_all, 0, 0, 1, 0);
singularity_PSO_MECA_allonmaxilla = -PerformanceIndexFunction_MECA_fig(x_PSO_MECA_all, 0, 0, 1, 0);
singularity_MECA_maxilla = -PerformanceIndexFunction_MECA_fig(x_MECA_maxilla, 0, 0, 1, 0);


%% plot
figure()
hold on
grid on
p1 = plot([stiffness_GA, stiffness_GA],[singularity_GA, manipulability_GA],'-o', 'LineWidth',2);
p2 = plot([stiffness_PSO, stiffness_PSO],[singularity_PSO, manipulability_PSO],'-o', 'LineWidth',2);
p3 = plot([stiffness_GA_MED, stiffness_GA_MED],[singularity_GA_MED, manipulability_GA_MED],'-o', 'LineWidth',2);
p4 = plot([stiffness_PSO_MED, stiffness_PSO_MED],[singularity_PSO_MED, manipulability_PSO_MED],'-o', 'LineWidth',2);
p5 = plot([stiffness_GA_STIFF, stiffness_GA_STIFF],[singularity_GA_STIFF, manipulability_GA_STIFF],'-o', 'LineWidth',2);
p6 = plot([stiffness_PSO_STIFF, stiffness_PSO_STIFF],[singularity_PSO_STIFF, manipulability_PSO_STIFF],'-o', 'LineWidth',2);
p7 = plot([stiffness_MECA, stiffness_MECA],[singularity_GA_MECA, manipulability_GA_MECA],'-o', 'LineWidth',2);
p8 = plot([stiffness_MECA, stiffness_MECA],[singularity_PSO_MECA, manipulability_PSO_MECA],'-o', 'LineWidth',2);
% p8 = plot([stiffness_MECA, stiffness_MECA],[singularity_MECA_maxillaonall, manipulability_MECA_maxilaonall],'--^');
xlabel('Stiffness')
ylabel('Manipulability & Singularity')
title('Stiffness vs Isotropy')
legend([p1 p2 p3 p4 p5 p6 p7 p8],{'A_{GA}','A_{PSO}','B_{GA}','B_{PSO}','C_{GA}', 'C_{PSO}', 'MECA_{GA}', 'MECA_{PSO}'})
% xlim([0.7 1])
ylim([0 1])

figure()
hold on
grid on
p1 = plot([stiffness_GA, stiffness_GA],[singularity_GA_maxilla, manipulability_GA_maxilla],'--o', 'LineWidth',2);
p2 = plot([stiffness_PSO, stiffness_PSO],[singularity_PSO_maxilla, manipulability_PSO_maxilla],'--o', 'LineWidth',2);
p3 = plot([stiffness_GA_MED, stiffness_GA_MED],[singularity_GA_MED_maxilla, manipulability_GA_MED_maxilla],'--o', 'LineWidth',2);
p4 = plot([stiffness_PSO_MED, stiffness_PSO_MED],[singularity_PSO_MED_maxilla, manipulability_PSO_MED_maxilla],'--o', 'LineWidth',2);
p5 = plot([stiffness_GA_STIFF, stiffness_GA_STIFF],[singularity_GA_STIFF_maxilla, manipulability_GA_STIFF_maxilla],'--o', 'LineWidth',2);
p6 = plot([stiffness_PSO_STIFF, stiffness_PSO_STIFF],[singularity_PSO_STIFF_maxilla, manipulability_PSO_STIFF_maxilla],'--o', 'LineWidth',2);
p7 = plot([stiffness_MECA, stiffness_MECA],[singularity_GA_MECA_allonmaxilla, manipulability_GA_MECA_allonmaxilla],'--o', 'LineWidth',2);
p8 = plot([stiffness_MECA, stiffness_MECA],[singularity_PSO_MECA_allonmaxilla, manipulability_PSO_MECA_allonmaxilla],'--o', 'LineWidth',2);
% p8 = plot([stiffness_MECA, stiffness_MECA],[singularity_MECA_maxilla, manipulability_MECA_maxilla],'-^');
xlabel('Stiffness')
ylabel('Manipulability & Singularity')
title('Stiffness vs Isotropy [Maxilla]')
legend([p1 p2 p3 p4 p5, p6, p7, p8],{'A_{GA}','A_{PSO}','B_{GA}','B_{PSO}','C_{GA}', 'C_{PSO}', 'MECA_{GA}', 'MECA_{PSO}'})
% xlim([0.7 1])
ylim([0 1])