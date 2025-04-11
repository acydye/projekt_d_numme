%Använd ode-45 för att lösa samma system


tol_ode = 1e-12; %Tolerans för ode-45
opts = odeset('AbsTol', tol_ode); %Inställningar

t_span = [0,0.89]; %Span som ska lösas
u0 = [0,0]'; %Start vektor


[t,y] = ode45(@dy_f, t_span,u0, opts); %Värdena

y_end = y(end,1); %Höjden vid t=0.89 s blir vid slutet



%Störnings beräkning
a = -3; b = 0.1;
err_a = abs(-3*0.01);
err_b = abs(0.1*0.01); %Osäkerhet i indata
n = 40; %Antal gånger att störa

storn_fel = zeros([n,1])
for i = 1:size(storn_fel)
    %Varierar a och b med ett slumpmässigt tal inom osäkerheten
    a_storn = a + (2*rand(1)-1)*err_a; b_storn = b + (2*rand(1)-1)*err_b;
    
    %Räknar ut y värdena med de störda värden
    

    [t_storn,y_storn] = ode45(@(t,y) dy_f(t,y,a_storn,b_storn), t_span, u0,opts);
    storn_fel(i,1) = abs(y_storn(end,1)-y_end); %Störnings fel
end
%Störnings felet blir maximal värdet från storn_fel
y_storn_error = max(storn_fel);
disp(['Ode45 funktionen ger att vid t = 0.89 (s) är y =  ' num2str(abs(y_end),12) ...
    ' (m), där felet är under 1e-12 ' newline])
disp(['Osäkerhet med 1% ger ett absolut fel på ' num2str(abs(y_storn_error),5) ...
     ' (m) ',newline])





function dydt = dy_f(t,y,a,b)
    %Delar differential ekvationen i vektor form
    %indata
    %t - vilka t värden man vill utvärdera på
    %y - [y, y'] y värden man vill utvärdera
    %Utdata
    %dydt - Approximativa derivatan för insat y värden
    if nargin == 2
        a = -3;
        b = 0.1;

    end

    dydt = zeros(2,1);
    dydt(1) = y(2);
    dydt(2) = a + b*(y(2).^2);
end

