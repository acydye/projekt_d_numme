%Vill hitta skillnad i tid för dumma och smarta roboten
%Importerar datan 'data_smart' och 'data_dum'
clear all; clc; close all;

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

%Ortsvektorerna för dumma-, smarta roboten, samt kulan
x_smart_r = plot_smart(:,1); y_smart_r = plot_smart(:,2);

y_kula = plot_dum(:,1);
%Dumma
x_dum_r = plot_dum(:,3); y_dum_r = plot_dum(:,4);


%Hur mycket snabbare är den smarta roboten?
t_diff = t_dum-t_smart;
y_diff = abs(y_smart-y_dum); 

%Då båda t_dum och t_smart har en osäkerhet och Δt är en linjär
%funktion av dem. Felet i Δt blir summan av bägge fel i indatan.

t_diff_err = t_smart_err + t_dum_err;
y_diff_err = y_smart_err + y_dum_err;

%Felet där indatan har en osäkerhet på 1%
t_diff_storn = t_dum_storn + t_smart_storn;
y_diff_storn = y_dum_storn + y_smart_storn;

disp([newline 'Smarta roboten är snabbare än dumma roboten där skillnaden vid träff är: '])
disp(['Δt = ' num2str(t_diff) ' ± ' ...
    num2str(t_diff_err) '(s)'])
disp(['Med störd indata blir felet istället: '])
disp(['Δt = ' num2str(t_diff,1) ' ± ' ...
    num2str(t_diff_storn) '(s)'])
disp([newline 'Relativa felet vid osäkerhet i indata blir: '])
disp(['ΔΔt/Δt(osäkherhet) = ' num2str(t_diff_storn/t_diff*100) ' %' newline])



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

xlabel('x (m)'); ylabel('y (m)');
title('Plot över kulan och roboternas rörelse')
axis('equal')
xlim([-0.5,0.5])
ylim([y_dum_r(end)-0.2,y_smart_r(end)+0.2])

grid on



legend('Location','southeast')







% Setup animation
figure;
hold on;
axis equal;
xlim([-5,5]);
ylim([min([y_dum_r; y_smart_r])-0.2, max([y_dum_r; y_smart_r])+0.2]);
xlabel('x (m)');
ylabel('y (m)');
title('Animation: Rörelse av robotar och kula');
grid on;

% Preallocate objects

h_smart = plot(NaN, NaN, 'go', 'MarkerSize', 8, 'DisplayName', 'Smarta roboten');
h_kula = plot(NaN, NaN, 'bo', 'MarkerSize', 8, 'DisplayName', 'Kula');

legend('Location', 'southeast');

% Find minimum number of frames (in case different lengths)
numFrames = min([length(x_dum_r), length(x_smart_r), size(plot_smart, 1)]);

for k = 1:numFrames
    % Update robot positions

    set(h_smart, 'XData', x_smart_r(k), 'YData', y_smart_r(k));

    % Assuming ball is in plot_smart(:,1:2) – adjust if needed
    x_kula = plot_smart(k, 1); 
    y_kula = plot_smart(k, 2);
    set(h_kula, 'XData', x_kula, 'YData', y_kula);

    drawnow;
    pause(0.03); % Adjust speed of animation
end

