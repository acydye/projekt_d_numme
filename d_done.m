%Vill hitta skillnad i tid för dumma och smarta roboten
%Importerar datan 'data_smart' och 'data_dum'
clear all; clc;

if ~exist('data_smart.mat') || ~exist('data_dum.mat')
    error('Filerna saknas')
end

load("data_smart.mat") %Data från fil c_done.m
load("data_dum.mat") %Data från fil b_done.m

%Tid för träff samt fel för dumma roboten
t_dum_err = t_dum(2); t_dum_storn = t_dum(3); t_dum = t_dum(1);

%Höjd vid träff samt fel för dumma roboten
y_dum_err = y_dum(2); y_dum_storn = y_dum(3); y_dum = y_dum(1);

%Tid för träff samt fel för smart roboten
t_smart_err = t_smart(2); t_smart_storn = t_smart(3); t_smart = t_smart(1);

%Höjd vid träff samt fel för smart roboten
y_smart_err = y_smart(2); y_smart_storn = y_smart(3); y_smart = y_smart(1);

%Ortsvektorerna för dumma och smarta roboten
x_smart_r = plot_smart(:,1); y_smart_r = plot_smart(:,2);
%
%Dumma
x_dum_r = plot_dum(:,1); y_dum_r = plot_dum(:,2);

%Vilken är snabbast?
t_diff = abs(t_smart-t_dum); % smarta roboten är snabbare
y_diff = abs(y_smart-y_dum); 

%Felet blir summan av båda felen i t_smart och t_dum
t_diff_err = t_smart_err + t_dum_err;
y_diff_err = y_smart_err + y_dum_err;

%Felet där indatan har en osäkerhet på 1%
t_diff_storn = t_dum_storn + t_smart_storn;
y_diff_storn = y_dum_storn + y_smart_storn;

disp([newline 'Smarta roboten är snabbare än dumma roboten där skillnaden vid träff är: '])
disp(['Δt = ' num2str(t_diff) ' ± ' ...
    num2str(t_diff_err) '(s), och Δy = ' num2str(y_diff) ' ± ' num2str(y_diff_err) ...
     ' (m)' newline])
disp(['Med störd indata blir felet istället: '])
disp(['Δt = ' num2str(t_diff,1) ' ± ' ...
    num2str(t_diff_storn) '(s), och Δy = ' num2str(y_diff,1) ' ± ' num2str(y_diff_storn) ...
     ' (m)' newline])
disp([newline 'Relativa felet vid osäkerhet i indata blir: '])
disp(['Δt/t(osäkherhet) = ' num2str(t_diff_storn/t_diff*100) ' %' newline ...
    'Δy/y(osäkherhet) = ' num2str(y_diff_storn/y_diff*100) ' %'])


%plot
width = 1;
figure;
hold on
plot([0,0],[0,y_dum_r(end)-0.3],'Blue','DisplayName','Kulans lägeskurva', ...
    'linestyle','--', 'LineWidth', 1)
plot(x_dum_r,y_dum_r,'red', 'DisplayName', 'Dumma roboten', 'lineWidth', width)
plot(x_smart_r,y_smart_r, 'green','DisplayName', 'Smarta roboten','lineWidth',width)

plot([0,0],[y_smart,y_dum],'X','MarkerSize',10,'Color','Black','DisplayName', ...
    'Träff')
errorbar(0,y_smart,y_smart_storn,'LineWidth',width)

xlabel('x (m)'); ylabel('y (m)');
title('Plot över kulan och roboternas rörelse')
axis('equal')
xlim([-0.5,0.5])
ylim([y_dum-0.2,y_smart+0.2])

grid on



legend('Location','southeast')






