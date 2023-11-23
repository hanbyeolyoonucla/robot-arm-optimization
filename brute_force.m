clc; clear; close all;
addpath('functions')
%% constraints and interval
lb = [0.082 0.100 0.040 0.090 -pi/2 -pi/2 -1 -1 -1];
ub = [0.3 0.3 0.3 0.3 pi/2 pi/2 1 1 1];
interval_link = 0.01;
interval_ang = 0.1;
interval_pos = 0.05;
%% discretization of the range
x1 = [lb(1) ceil(lb(1)*100)/100:interval_link:ub(1)];  % l2
x2 = lb(2):interval_link:ub(2);  % l3
x3 = lb(3):interval_link:ub(3);  % l4
x4 = lb(4):interval_link:ub(4);  % l5
x5 = [lb(5) ceil(lb(5)*10)/10:interval_ang:floor(ub(5)*10)/10 ub(5)];  % alpha
x6 = [lb(6) ceil(lb(6)*10)/10:interval_ang:floor(ub(6)*10)/10 ub(6)];  % beta
x7 = lb(7):interval_pos:ub(7);  % x
x8 = lb(8):interval_pos:ub(8);  % y
x9 = lb(9):interval_pos:ub(9);  % z
    
%% combination of each point
X = combinations(x5,x6,x7,x8,x9);

%% combination of each point
% Fval = zeros(height(X));
for ii =1:height(X)
    Fval(ii) = PerformanceIndexFunction_MECA(table2array(X(ii,:)));
    if mod(ii,1000) == 0
        fprintf('%d out of %d \n',ii, height(X))
    end
end

% Fval = zeros(length(x5),length(x6),length(x7),length(x8),length(x9));
% for i1 = 1:length(x1)
%     for i2 = 1:length(x2)
%         for i3 = 1:length(x3)
%             for i4 = 1:length(x4)
%                 for i5 = 1:length(x5)
%                     for i6 = 1:length(x6)
%                         for i7 = 1:length(x7)
%                             for i8 = 1:length(x8)
%                                 for i9 = 1:length(x9)
%                                     Fval(i1,i2,i3,i4,i5,i6,i7,i8,i9) = PerformanceIndexFunction(x1(i1),x2(i2),x3(i3),x4(i4),x5(i5),x6(i6),x7(i7),x8(i8),x9(i9));
%                                 end
%                             end
%                         end
%                     end
%                 end
% 
%             end
%         end
%     end
% end