%Gissar start vinkel
initial_guess = [3.2,3.6]; %Gissning

tol_abs =1e-5; %Absolut tolerans för ode45
tol_rel_high = 1e-6; %Hög gräns
tol_rel_low = 1e-9; %lägre gräns
opts_high = odeset('RelTol',tol_rel_high,'AbsTol',1e-3); %Testa olika toleranser
opts_low = odeset('RelTol',tol_rel_low, 'AbsTol', 1e-11);

x0 = 4.99; v = 5; %Indata, robot pos och hastighet



%Använder sekant för att hitta rätt vinkel start vinkel
[t_high,y_high, ~,~,n_high] = get_time_height(opts_high,initial_guess); %Hög relativ tolerans

[t_low,y_low, t_err,y_err,n_low] = get_time_height(opts_low,initial_guess); %Låg relativ tolerans 

%Felet i runge kutta blir skillnaden mellan svaren från 'opts_high' och
%'opts_low'
error_t_ode = abs(t_high-t_low);
error_y_ode = abs(y_high-y_low);

total_error_t = error_t_ode + t_err;
total_error_y = error_y_ode + y_err;

disp(['Trunkerings felet från ode45 får vi genom att testa olika toleranser och subtrahera dessa värden,' newline ...
    'vi hade en där n1 = ' num2str(n_high) ', och n2 = ' num2str(n_low) newline ...
    'Error y Trunkering = ' num2str(error_y_ode) newline ...
    'Error t trunkering = ' num2str(error_t_ode) newline]);



disp(['Med ode45 och f-zero får vi att kulan och roboten träffas vid:' newline ...
    'y = ' num2str(y_low,11) ' ± ' num2str(total_error_y) ' (m)' newline ...
    't = ' num2str(t_low,11) ' ± ' num2str(total_error_t) ' (s)' newline]) 
    

%Störnings beräkning
%Vi har indatan a,b,x0,v
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



    [t_storn,y_storn, ~,~] = get_time_height(opts_low,initial_guess, ...
        x_storn,v_storn,a_storn,b_storn);
    fel_storn(i,:) = [abs(y_storn-y_low), abs(t_storn-t_low)];
end

%Total felet från störning blir maximum av alla fel
fel_storn;
y_error_storn = max(fel_storn(:,1)); %Fel i träff värdet för y
t_error_storn = max(fel_storn(:,2)); %Fel i träff tiden för t;

disp([newline 'Med en osäkerhet av 1% i indatan får vi följande värden:' newline ...
    'y = ' num2str(y_ball,3), ' ± ' num2str(y_error_storn,2) ' (m)' newline ...
    't = ' num2str(t_collision, 3) ' ± ' num2str(t_error_storn,2) ' (s)' newline ])







%Nu har vi trunkerings felet från secant
%Vad blir felet från ode45?

function [tr,yr,t_err,y_err,n] = get_time_height(opts,start,x0,v,a,b)
    if nargin == 2
        x0 = 4.99;
        v = 5;
        a = -3;
        b = 0.1;
    end
    [angle,angle_error] = fzero(@(guess) errFunc(guess,opts,x0,a,b,v),start);
    [tr,yr,t_err,y_err] = get_from_angle(angle,angle_error,x0,v);

    %Hur många indelningar är det i ode45?
    [t, ~] = ode45(@f,[0,tr],[0,0],opts);
    n = length(t);

end
function [t_hit,y_hit,t_err,y_err] = get_from_angle(phi,phi_err,x0,v)
    %Funktion get ut tid och y värde för kollision mellan dumma roboten och
    %bollen då man vet korrekt start vinkel samt returnerar en fel term
    %genom fortplantnings kalkyl
    t_hit = -x0/(v*cos(phi)); %Tiden för collision
    y_hit = x0*tan(-phi); %Position för y
    
    t_err = abs(x0/(sec(phi_err).^2))*phi_err;
    y_err = abs(x0/(v*cos(phi_err).^2))*phi_err;
end



function error = errFunc(guess,opts, x0,a,b,v)
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
    [~,y] = ode45(@f,[0,t0],u0,opts);
    y_korr = y(end,1);
    y_get = t0*v*sin(guess);

    error = y_korr-y_get;
end


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