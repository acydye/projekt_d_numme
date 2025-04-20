; %Inställningar för ode45

[t,y] = ode45(@dy_func,[0,tend],u0,opts);

find_time_adv(t,y)

function [t_meet,t_err_trunk] = find_time_adv(t,y,limit)
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
    robot_ball = r_b-r_r;
    s = vecnorm(robot_ball,2,2);
    
   
    %hitta index
    x = robot_ball(:,1); y = robot_ball(:,2);
    i_x = find(x < 0, 1);
    i_y = find(abs(y) <= abs(y(i_x)));
    i = i_x
    



    plot(robot_ball(:,1),robot_ball(:,2))
    


    % 
    % subplot(2,2,1)
    % plot(t(1:length(ds_diff)),ds_diff)
    % subplot(2,2,2)
    % plot(t(1:length(ds_2_diff)),ds_2_diff)
    % 
    % 
    % subplot(2,2,3)
    % plot(t(i-5:i+2),s(i-5:i+2), 'blue', t(i),s(i),'x')
    % subplot(2,2,4)
    % plot(t,s, 'red',t(i),s(i),'X')



    % Vill nu interpolera för punkterna innan och hitta t där d(t) = 0

    span = i-2:i %Index av punkter som ska interpoleras
    dd = s(span); tt = t(span); %avståndet (dd) och tiden (tt)
    t_ca = t(i); %Ungefärliga värdet

    second_koef = polyfit(tt,dd,2); %Linjär interpolation
    roots_sec = roots(second_koef)

    [~, index_root] = min(abs(roots_sec-t_ca));
    root_sec = roots_sec(index_root);


    third_koef = polyfit(tt,dd,3); %Andra grads interpolation
    roots_trd = roots(third_koef);

    [~, index_root] = min(abs(roots_trd-t_ca));
    root_trd = roots_trd(index_root);




    t_err_trunk = abs(root_sec-root_trd) %Trunkerings fel
    t_meet = root_sec;


    
    
end