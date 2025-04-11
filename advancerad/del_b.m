%Löser med ode-45 samt interpolation

u0 = [0,0,-4.98,0]; %Start vektor
h_t = 1e-3; %Initial steglängd för att beräkna tiden
tol = 1e-10;
tend = 1.22; %Avslut

opts = odeset('AbsTol', 1e-6); %Inställningar för ode45

[t,y] = ode45(@dy_func,[0,tend],u0,opts);

find_time_adv(t,y)

function index = find_time_adv(t,y,limit)
    %Vi har d (avstånd) = d(t), avstånd är en funktion av tiden.
    %Denna function är en absolut värd funktion dvs d(t) > 0, det betyder
    %att derivatan kommer vara kontinuerlig framtill där roboten och kulan
    %möts. Målet med denna funktion är att hitta denna punkt genom att
    %apprxoimera derivatan med centraldifferenser. 
    %
    %Indata
    %(t,y) - värden av t samt position från rungekutta-4
    %h - steglängden som användes i rungekutta-4
    %limit - gräns för skillnad i derivatan m.a.p största skillnaden
    %Utdata:
    %Index - index för punkten precis innan träff
    if nargin == 2
        limit = 0.1;
    end

    r_r = y(:, 3:4); %Robot ortsvektor
    r_b = [zeros(length(y(:,1)),1),y(:,1)]; %Kulans orts vektor
    s = vecnorm(r_r-r_b,2,2);
    %Central differans metod
    ds_dt = (s(3:end)-s(1:end-2))./(t(3:end)-t(1:end-2));
    
    %differens av derivatan
    diff_ds = ds_dt(2:end)-ds_dt(1:end-1);
    
    %Gräns för ändring i derivatan
    tol_max = limit*max(diff_ds);


    for i = 1:length(diff_ds)
        if abs(diff_ds(i)) > tol_max
            index = i + 2; %omvandling pga central differans
            subplot(2,1,1)
            plot(t(index-10:index+2),s(index-10:index+2), 'blue', t(index),s(index),'x')
            subplot(2,1,2)
            plot(t,s, 'red',t(index),s(index),'X')
            return
        end
    end
    disp(['Error, ingen dicontinuitet hittades i find_time()'])


    %Vill nu interpolera för punkterna innan och hitta t där d(t) = 0
    
    
end