clear all, close all, clc

%Lägger till funktionerna som jag behöver


%start indelning 
%Vill hitta y(t) där t = 0.89 s
h = 0.89; %start indelningen
tol = 1e-10; %Tolerans
t_end = 0.89; %Slut tid
u0 = [0,0]; %Start vektor


fel = zeros([20,3]); dy = 1; %För allokering för fel

%Loopen halverar steglängden vid varje iteration tills felet
%i y-värdet för roboten är under en felgränsen
for i = 1:20
    [t_n, u_n] = rkf(@f,[0,t_end],u0,h);
    y_ball = u_n(end,1);
    if i > 1

        dy = abs(yPrev-y_ball);
        fel(i,1:3) = [y_ball, h, dy];
    else
        fel(i,1:3) = [y_ball,h,0];
    end
    if dy < tol
        disp(['Runge-kutta 4 get att vid t = 0.89 s har bollen rullat ' num2str(abs(y_ball),11) ...
    '  ± ' num2str(dy) ' (m)' newline])
       break 
    end
   h = h/2;
   yPrev = y_ball;

end
decimaler = floor(abs(log10(dy)));
round(y_ball,decimaler);
err_pres = abs(y_ball - round(y_ball,decimaler)); %presentations fel

%totala felet
y_err = err_pres + dy;
y_ball = round(y_ball,decimaler);
disp(['h = ' num2str(h)])
disp(['Med perfect indata blir felet  ' num2str(y_err,1) ...
    ' (m)' newline 'får vi att y(0.89) = ' num2str(y_ball,decimaler+1) ' ± ' ...
    num2str(y_err,1) ' (m)'] )


%Ser att felet avtar med lutning 1 i loglog plot
%stämmer överens med framåt euler noggranhetsordning
subplot(1,2,2);
plot(log(fel(:,2)),log(fel(:,3)))
title('Loglog plot dy = dy(h)');
grid("on");
ylabel('log(dy)');
xlabel('log(h)');
table(fel(:,1),fel(:,2),fel(:,3),'VariableNames',{'y', 'h','dy'})

%Plot över t m.a.p y
subplot(1,2,1);
plot(t_n, u_n(:,1), "blue", 0.89,y_ball,'X');
title('plot y = y(t)');
grid("on");
ylabel('y (m)');
xlabel('t (s)');

%% osäkerhet i indata
%Anta osäkerhet i indata på 1%
%Indatan vi har
%koefficenterna i differential ekvationen
%startvärdena på y och y' men då båda är noll blir felet i indatan noll
%Utför störnings kalkyl
%Kalla koefficenterna a och b
a = -3; b = 0.1;
err_a = abs(-3*0.01);
err_b = abs(0.1*0.01); %Osäkerhet i indata

%Störnings räkning


%Varierar a och b med ett slumpmässigt tal inom osäkerheten
a_storn = a + err_a; b_storn = b + err_b;

%Räknar ut y värdena med de störda värden
f_storn_a = @(t,y) f(t,y,a_storn,b); % stör a
f_storn_b = @(t,y) f(t,y,a,b_storn); % stör b

[~,y_storn_a] = rkf(f_storn_a, [0,t_end], u0,h);
[~,y_storn_b] = rkf(f_storn_b, [0,t_end], u0,h);
storn_err_tab = abs(y_ball - y_storn_b(end,1)) + abs(y_ball - y_storn_a(end,1));
y_ball_storn = round(y_ball,2, "significant");
storn_err_pres = abs(y_ball_storn-y_ball); %presentations fel

%totala felet blir summan
storn_err_pres + storn_err_tab;
storn_err_pres = ceil((storn_err_pres + storn_err_tab)*100)/100;





%Felet på grund av osäkerhet blir max värdet av storning_fel vektorn

disp(['Med 1 % osäkerhet i indatan får vi störnings fel på ' num2str(storn_err_pres) ...
    ' (m)' newline 'Med störd indata får vi att y(0.89) = ' num2str(y_ball_storn) ' ± ' ...
    num2str(storn_err_pres,1) ' (m)'] )


function dydt = f(t,y,a,b)
    %Delar differential ekvationen i vektor form
    %indata
    %t - vilka t värden man vill utvärdera på
    %y - [y, y'] y värden man vill utvärdera
    %a,b - frivillig insättning av andra koefficenter
    %Utdata
    %dydt - Approximativa derivatan för insat y värden
    
    if nargin == 2
        a = -3;
        b = 0.1;
    end
    dydt = zeros([2,1]);
    dydt(1) = y(2);
    dydt(2) = a + b*(y(2).^2);
end



