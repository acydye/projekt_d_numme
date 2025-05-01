%Secant för att hitta då avståndet är noll mellan robotarna
close all; close all; clc;

u0 = [0,0,-4.98,0]; %Start vektor
h_t = 1e-3; %Initial steglängd för att beräkna tiden
tol = 1e-11;
tend = 1.21; %Avslut

max_iter = 10; %Max antal halveringar


%förallokerar

fel = zeros([max_iter,5]); dy = 1; %För allokering för fel

%Loopen halverar steglängden vid varje iteration tills felet
%i y-värdet för roboten är under en felgränsen
err_t = 1;
for i = 1:max_iter

    [ti,yi] = rkf(@dy_func,[0,tend],u0,h_t); %runge-kutta 4
    si = get_distance(yi); %Avståndet mellan robot och kulan

    i_hit = find_time(ti,yi,h_t,0.1); %Index för punkt innan träff
    
    [t_hit, t_err_int] = interp_time(ti,si,i_hit); %Interpolerar för att hitta nollstället
    
    if i > 1

        dt = abs(t_hit - t_prev); %Trunkerings fel.
        err_t = dt + t_err_int;
        fel(i,:) = [h_t, t_hit, dt,t_err_int,err_t];
    else
        fel(i,:) = [h_t,t_hit,0,0,0];
    end
    if err_t < tol
        disp(['Roboten träffar kulan vid t = ' num2str(abs(t_hit),8) ...
    '  ± ' num2str(err_t) ' (s)' newline]);
       break 
    end
   h_t = h_t/2;
   t_prev = t_hit;

end
t_deci = floor(abs(log10(err_t)));
round(t_hit,t_deci);
t_err_pres = abs(t_hit - round(t_hit,t_deci)); %presentations fel

%totala felet
err_t_tot = t_err_pres + err_t
t_hit_pres = round(t_hit,t_deci)
h_t


% Felet avtar relativt regelbundet men noggranhets ordningen verkar inte
% vara 4. Men vi använder flera metoder samtidigt
loglog(fel(:,1), fel(:,end), '-o')
xlabel('Steglängd h')
ylabel('Fel i t_{träff}')
grid on
title('Konvergens av träfftiden')
t_hit = fel(end,2);
err = fel(:,end);


p = log2(err(2:max_iter-1)./err(3:max_iter));
mean(p)

%% Söker y värdet för kollision
%t_hit ger rätt kollision, använder bara rungekutta för att hitta
%korrespondernade y-värde


y_iter = 20; %Maximal iterationer
fel = zeros([20,3]); dy = 1; %För allokering för fel
h_y = 1e-2; %Steglängd för att beräkna träff punkt y värde

%Loopen halverar steglängden vid varje iteration tills felet
%i y-värdet för roboten är under en felgränsen
for i = 1:y_iter
    [t_n, u_n] = rkf(@dy_func,[0,t_hit],u0,h_y);
    y_hit = u_n(end,1);
    if i > 1

        dy = abs(yPrev-y_hit);
        fel(i,1:3) = [y_hit, h_y, dy];
    else
        fel(i,1:3) = [y_hit,h_y,0];
    end
    if dy < tol

        %Då t_hit har osäkerhet err_t beräknar jag störning
        [~,y_storn_1] = rkf(@dy_func,[0,t_hit+err_t],u0,h_y); %Stör upp
        [~,y_storn_2] = rkf(@dy_func,[0,t_hit-err_t],u0,h_y); %Stör ner

        error_y_storn = max( [abs(y_hit-y_storn_1(end,1)), abs(y_hit-y_storn_2(end,1))]); %Maximal skillnad
        err_y = dy + error_y_storn; %Lägger ihop för att få total fel



       break 
    end
   h_y = h_y/2;
   yPrev = y_hit;
end
y_deci = floor(abs(log10(err_y)));
round(y_hit,y_deci);
err_y_pres = abs(y_hit - round(y_hit,y_deci)); %presentations fel

%totala felet
err_y_tot = err_y_pres + err_y
y_hit_pres = round(y_hit,y_deci)
h_y
disp(['Runge-kutta 4 get att vid t = ' num2str(abs(t_hit_pres),11) ...
'  ± ' num2str(err_t_tot,1) ' (s)' newline 'har bollen rullat ' num2str(y_hit_pres,11) ...
'  ± ' num2str(err_y_pres,1) ' (m)' newline]);

%% störningsräkning
%Stör indata med 1%
%Indata:
%Start vektorn - u
%Robotens hastighet - v
%Koefficenterna i differential ekvationen - (a,b)

%Standard värden och osäkerhet
v = 5; err_v = 5*0.01;
u = [0,0,-4.98,0]; err_u = abs(0.01*u0);
a = -3; err_a = 0.03; b = 0.1; err_b = 1e-3;


n_storn = 5; %Antal gånger att störa indatan
fel_storn = zeros([n_storn,2]);
storn_list = diag([1,1,1,1])

for i = 1:4

    %Störd indata
    a_storn = a + (2*rand(1)-1)*err_a; b_storn = b + (2*rand(1)-1)*err_b;
    u_storn = u + (2*rand(1)-1)*err_u; v_storn = v + (2*rand(1)-1)*err_v;

    %Störd funktion för runge-kutta 4
    dy_func_storn = @(t,y) dy_func(t,y,v_storn,a_storn,b_storn);

    %Tid vid störning
    [t_storn,y_storn] = rkf(dy_func_storn,[0,tend],u_storn,h_t); %runge-kutta 4
    s_storn = get_distance(y_storn); %Avståndet mellan robot och kulan
    i_storn = find_time(t_storn,y_storn,h_t,0.1); %Index för punkt innan träff
    [t_hit_storn, ~] = interp_time(t_storn,s_storn,i_storn); %Interpolerar för att hitta nollstället

    %Ortsvektorns y-värde vid störning
    [~,y_storn] = rkf(dy_func_storn, [0,t_hit_storn],u_storn,h_y);
    y_hit_storn = y_storn(end,1);
    
    %Lista med felet i y, samt t.
    fel_storn(i,:) = [abs(y_hit_storn-y_hit),abs(t_hit-t_hit_storn)];



end

storn = max(fel_storn,[],1); %Max felet i t och y,
err_storn_y = storn(1); err_storn_t = storn(2); %Störnings felet

disp([newline 'Med störning får vi y = ' num2str(y_hit) ' ± ' num2str(err_storn_y) ...
    '(m). t = ' num2str(t_hit,4) ' ± ' num2str(err_storn_t) ' (s).' newline])





%%
%Plotta roboten och kulan
figure();

% subplot(1,2,1)
% plot(zeros(length(yi(:,1)),1),yi(:,1),'Blue','DisplayName','Kulans lägeskurva', 'LineWidth', 1)
% hold on
% plot(yi(:,3),yi(:,4),'red', 'DisplayName', 'Robotens lägeskurva', 'lineWidth', 1)
% plot(0,y_hit,'X', 'MarkerSize',10, 'displayName','Träff', 'Color','Black')
% xlim([-5,1])
% grid('on')
% ylabel('Y')
% xlabel('X')
% hold off
% legend(Position=[0.2,0.2,0.2,0.2])

% subplot(1,2,2)
dist = get_distance(yi);
plot(ti(end-1050:end-990),dist(end-1050:end-990), 'black', 'LineWidth', 1, 'MarkerSize', 10, 'DisplayName','d = d(t)')
xlim([1.20586166365831,1.20613])
ylim([-0.00003062231693386406,0.00009222365663980835])
xlabel('tid (s)')
ylabel('Avstånd mellan robot och kulan (m)')

legend()

matlab2tikz('Plot_distance_b.tex')

%Sparar informationen
deltat = 1e-2; %Steglängd i tid
[~,y_save] = rkf(@dy_func,[0,tend],u0,deltat); %Gör en mindre version för att plotta
y_dum = [y_hit,err_y,err_storn_y]; %y och felet i y
t_dum = [t_hit, err_t, err_storn_t]; %t och felet i t
plot_dum = y_save;

save('data_dum', 't_dum','y_dum','plot_dum','deltat')

%%
u0 = [0,0,-4.98,0]; %Start vektor
h = 1e-4/7; %Initial steglängd
tol = 1e-10;
tend = 1.21; %Avslut

[ti,yi] = rkf(@dy_func,[0,tend],u0,h); %runge-kutta 4
si = get_distance(yi); %Avståndet mellan robot och kulan

j = find_time(ti,yi,h); %Index för punkt innan träff

[t_hit, t_err_int] = interp_time(ti,si,j);



function distance = get_distance(y_values)
    r_r = y_values(:, 3:4);
    r_b = [zeros(length(y_values(:,1)),1),y_values(:,1)];
    distance = sqrt(sum((r_b - r_r).^2, 2));
end

function [t_meet,err_trunk] = interp_time(t,s,i)
    %indata:
    %t - tids vektor
    %s - avstånds vektor
    %
    %utdata
    %t_meet - tid för träff
    %t_err - osäkerhet i t_meet
    tt = t(i-1:i);
    ss = s(i-1:i);

    %Linjär interpolation
    lin = poly(tt,ss,1);
    t_1 = -lin(2)/lin(1); %Noll ställe

    %Söker trunkerings felet i interpolationen
    %Andra grads polynom
    pp2 = poly(tt,ss,2);

    sum(pp2.*[t_1^2;t_1;1]);
    
    %Hittar nollställen
    b = pp2(2)/pp2(1); c = pp2(3)/pp2(1);
    rot1 = -b/2 + sqrt((b/2)^2 - c); 
    rot2 = -b/2 - sqrt((b/2)^2 - c);
    
    %Det som är närmast t_1 blir det vi använder för att räkna trunkerings
    %felet
    [~,n] = max(abs([rot1-t_1,rot2-t_1])); %Minsta skillnad mellan t_1 blir korrekt
    if n == 2,  t_2 = rot1; else, t_2 = rot2; end
    

    t_meet = t_1; %Tid punkt för träff
    err_trunk = abs(t_1-t_2); %Trunkerings felet från interpolation
 
end

function index = find_time(t,y,h,limit)
    %Vi har d (avstånd) = d(t), avstånd är en funktion av tiden.
    %Denna function är en absolut värd funktion dvs d(t) > 0, det betyder
    %att derivatan kommer vara kontinuerlig framtill där roboten och kulan
    %möts. Målet med denna funktion är att hitta denna punkt genom att
    %apprxoimera derivatan med centraldifferenser. 
    %
    %Indata
    %(t,y) - värden av t samt position från rungekutta-4f
    %h - steglängden som användes i rungekutta-4
    %limit - gräns för skillnad i derivatan m.a.p största skillnaden
    %Utdata:
    %Index - index för punkten precis innan träff
    if nargin == 3
        limit = 0.11;
    end

    r_r = y(:, 3:4); %Robot ortsvektor
    r_b = [zeros(length(y(:,1)),1),y(:,1)]; %Kulans orts vektor
    s = sqrt(sum((r_b - r_r).^2, 2)); %Avstånd mellan dem
    
    %Central differans metod
    ds_dt = (s(3:end)-s(1:end-2))./(2*h);
    
    %differens av derivatan
    diff_ds = ds_dt(2:end)-ds_dt(1:end-1);
    

    %Gräns för ändring i derivatan
    tol_max = limit*max(diff_ds);


    for i = 1:length(diff_ds)
        if abs(diff_ds(i)) > tol_max
            index = i + 2; %omvandling pga central differans
            plot(t(index-10:index+4),s(index-10:index+4), 'blue', t(index),s(index),'x')
            return
        end
    end
    disp(['Error, ingen dicontinuitet hittades i find_time()'])


    %Vill nu interpolera för punkterna innan och hitta t där d(t) = 0
    plot(t(index-5:index+2),s(index-5:index+2), 'blue', t(index),s(index),'x')
    
end

function c = poly(x,y,n)
    if n == 1
        A = [x, x.^0];
        c = A\y;
    end
    if n == 2
        A = [x.^2,x, x.^0];
        c = A\y;
    end
    if n == 3
        A = [x.^3,x.^2,x,x.^0];
        c = A\y;
    end
    
end