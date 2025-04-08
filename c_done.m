%Använda runge-kutta 4, inskjutning och sekant metoden för att lösa
%systemet. 
clear all; clc; close all;

h = 1e-5; %Steglängd för rk-4
tol_sec = 1e-15; %Tolerans för sekant funktionen


phi_1 = 3.4; phi_2 = 3.6; %Start gissning för sekant metoden

%Vill hitta fel från runge-kutta 4. Håller allt förutom steglängden
%konstant, och varierar den.

x0 = 4.99; v = 5;

h = 1; %start steglängden
n = 10; %antal gånger att halvera steglängden

%Förallokering
fel_y = zeros([n,3]);
fel_t = zeros([n,3]);

%Vid varje iteration halveras steglängd och m.h.a zerof funktionen
%hittar man vinkel (psi) samt felet från secant (error_psi_secant),
for i = 1:n

    [psi,err_psi_secant,iter_err,psi_list,iter] = sekant(@(guess) errFunc(guess,h), phi_1, phi_2,tol_sec,50);
    
    t_collision = -x0/(v*cos(psi)); %Tiden för collision
    y_ball = x0*tan(-psi); %Position för y

    %Fortplantning av felet från theta till tid (t) samt höjd (y)
    error_y_secant = abs(x0/(sec(psi).^2))*err_psi_secant;
    error_t_secant = abs(x0/(v*cos(psi).^2))*err_psi_secant;
    %Felen är under 1e-15 och är då försumbara


    if i > 1
        %Trunkerings felet från runge-kutta 4
        error_y_rk = abs(y_ball_prev - y_ball); %y-position
        error_t_rk = abs(t_collision - t_col_prev); %tid
        
        %Total felet, summering av felet från rk-4 och sekant.
        err_total_y = error_y_rk + error_y_secant; 
        err_total_t = error_t_secant + error_t_rk;

        %Lägger in felen, samt steglängd i en lista
        fel_y(i,:) = [h,y_ball, err_total_y];
        fel_t(i,:) = [h,t_collision,err_total_t];
    else
        %Vid första iterationen kan man ej räkna ett trunkerings fel
        fel_y(i,:) = [h, y_ball,0];
        fel_t(i,:) = [h,t_collision,0];
    end
 
    y_ball_prev = y_ball; 
    t_col_prev = t_collision;
    psi_prev = psi;

    h = h/2; %Halverar steglängden
end


table(fel_y);
table(fel_t);

%Vill kolla konvergensordningen
p = (1+sqrt(5))/2; %korrekt
p_obs = (log(iter_err(3:end))- log(iter_err(2:end-1))) ./ ...
        (log(iter_err(2:end-1)) - log(iter_err(1:end-2)))

konv = iter_err(3:end)./(iter_err(1:end-2).*iter_err(2:end-1));


subplot(2,2,[3,4])
semilogy(konv,'o')
title('Konvergens plot för sekant metoden')
ylabel('log(C)')
xlabel('i (iteration)')


%Plottar felet för att se noggranhets ordningen
%Ser ungefär ut som 4 vilket stämmer med runge-kutta 4 och felet avtar
%regelbundet vilket tyder att vår metod är korrekt

subplot(2,2,1);
plot(log(fel_y(2:end,1)),log(fel_y(2:end,3))) %Höjden
title('Error y (meter) gentemot steglängd')

xlabel('log(h)'); ylabel('log(dy)');
subplot(2,2,2);
plot(log(fel_t(2:end,1)),log(fel_t(2:end,3))) %Tiden
xlabel('log(h)'); ylabel('log(dt)');
title('Fel i t (tid) gentemot steglängd')





%Visar värdena och fel för användaren
disp([newline 'Vinkeln theta = ' num2str(psi,10) ' ± ' num2str(error_y_secant,3) ...
    ' (rad) ger korrekt start vinkel' newline ...
    'Smarta roboten och kulan träffas då vid t = ' num2str(t_collision,15) ' ± ' ...
    num2str(err_total_t,3) '(s)' newline ...
    'och vid y = ' num2str(y_ball,10) ' ± ' num2str(err_total_y) ' (m)' newline ])




%%
%Störnings beräkning
%All indata ska störas med 1%
%Indata: 
%(0,0),(0,x0) start position för robot och kula.
%(a,b) - koefficenter inom differential ekvationen
%v - hastighet för smarta roboten

%Vill nu lösa ut samma y och t med störda värden
%Vi har totalt 4 störd indata ty då 0 störd med 1% är fortfarande 0

x0 = 4.99; a = -3; b = 0.1; v = 5; %Ostörda värden
err_x = 0.01*4.99; err_a = 0.01*3; err_b = 0.01*0.1; err_v = 5*0.01; %Osäkerhet i indata

n = 100; %Antal störningar
fel_storn = zeros(n,2); %Förallokering

%Varje iteration av loopen stör indatan slumpmässigt inom osäkerheten för
%att sedan beräkna t (collision), y (höjd vid collision), och sedan beräkna
%skillnaden från de störda värden och de ostörda värden.

for i = 1:n
    % 2*rand(1)-1;
    a_storn = a + (2*rand(1)-1)*err_a; b_storn = b + (2*rand(1)-1)*err_b;
    x_storn = x0 + (2*rand(1)-1)*err_x; v_storn = v + (2*rand(1)-1)*err_v;



    psi_storn = sekant(@(guess) ... %Störd vinkel
        errFunc(guess,h,x_storn,a_storn,b_storn,v_storn), ...
        psi, psi+0.1,tol_sec); 

    t_storn = -x_storn/(v*cos(psi_storn)); %Tiden för collision
    y_storn = x_storn*tan(-psi_storn); %Position för y
    fel_storn(i,:) = [abs(y_storn-y_ball), abs(t_storn-t_collision)]; %Fel pga osäkerhet
end

%Total felet från störning blir maximum av alla fel
fel_storn;
y_error_storn = max(fel_storn(:,1)); %Fel i träff värdet för y
t_error_storn = max(fel_storn(:,2)); %Fel i träff tiden för t;

disp([newline 'Med en osäkerhet av 1% i indatan får vi följande värden:' newline ...
    'y = ' num2str(y_ball,3), ' ± ' num2str(y_error_storn,2) ' (m)' newline ...
    't = ' num2str(t_collision, 3) ' ± ' num2str(t_error_storn,2) ' (s)' newline ])





%%
%Skapar en plot

orts_smart = @(x) y_ball - (y_ball/x0)*x; %y = y(x) för smarta roboten
yy = linspace(0,y_ball+0.5,100);
xx = linspace(-0.5, x0,100);


%Vill spara informationen
y_smart = [y_ball,err_total_y,y_error_storn];
t_smart = [t_collision,err_total_t, t_error_storn];
plot_smart = [xx',orts_smart(xx)',];

save('data_smart', 't_smart','y_smart',"plot_smart")

figure;

plot([0,0],[0,y_ball], 'blue', 'displayName','Kulan')
hold on
plot(xx,orts_smart(xx),'red', 'displayName','Smarta roboten')
plot(0,y_ball, 'kX','MarkerSize',10,'MarkerFaceColor','k', 'DisplayName', ...
    'Träff')
errorbar(0,y_ball,y_error_storn, 'displayName','Fel i y')

annotation('textbox',[0.7,0.35,0.2,0.3], ...%Position
     'String', {['Träff:' newline 'y = ' num2str(y_ball,4) ' (m)' newline ...
     't = ' num2str(t_collision,4) ' (s)']})


xlabel('x')
ylabel('y')
title('Plot över kulan och smarta roboten i xy-planet')

hold off
legend('Location','southeast')




function error = errFunc(guess,h, x0,a,b,v)
    %Funktion använder inskjutnings metoden och hittar felet
    %mellan gissningen och riktiga värdet
    %Indata:
    %Guess - gissning på theta (vinkel)
    %h - stegindelning för rk4
    %x0, a, b, v - frivilligt val av indatan används vid störnings kalkyl
    %Utdata:
    %error - hur långt ifrån robotarna är från varandra i y-led dvs felet i
    %y

    u0 = [0,0];
    if nargin < 3
        v = 5;
        x0 = 4.99;
    else
        f = @(t,y) f(t,y,a,b);
    end
    t0 = -x0/(v*cos(guess));
    [t,y] = rkf(@f,[0,t0],u0,h);
    y_korr = y(end,1);
    y_get = t0*v*sin(guess);

    error = abs(y_korr-y_get);
end

function [t,y,error] = errFunc_y(guess,h)
    %Funktion använder inskjutnings metoden och hittar felet
    %Samma som ovan men returnerar u-vektorn som ger rätt svar
    %mellan gissningen och riktiga värdet
    %Indata:
    %Guess - gissning på theta
    %h - stegindelning för rk4
    %Utdata:
    %error - hur långt det är från faktiska värdet
    v = 5;
    u0 = [0,0];
    x0 = 4.99;
    t0 = -x0/(v*cos(guess));
    [t,y] = rkf(@f,[0,t0],u0,h);
    y_korr = y(end,1);
    y_get = t0*v*sin(guess);
    error = abs(y_korr-y_get);
end

% function t = newton(g,dg,t0,tol)
%     dt = 1;
%     for i = 1:40
%         dt = g(t0)/dg(t0);
%         t0 = t0 - dt
%         if abs(dt) < tol
%             t = t0;
%         end
%     end
% end


function dydt = f(t,y,a,b)
    %Delar differential ekvationen i vektor form
    %indata
    %t - vilka t värden man vill utvärdera på
    %y - [y, y'] y värden man vill utvärdera
    %a,b - frivillig insättning av andra koefficenter
    %Utdata
    %dydt - Approximativa derivatan för insatt y värden
    if nargin == 2
        a = -3;
        b = 0.1;
    end
    dydt = zeros([2,1]);
    dydt(1) = y(2);
    dydt(2) = a + b*(y(2).^2);

end