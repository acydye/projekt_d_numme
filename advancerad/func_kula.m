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

