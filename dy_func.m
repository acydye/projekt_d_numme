function dydt = dy_func(t,y,v,a,b)
    %Indata:
    %t värden
    %y vektor värden 
    %y(1) = kulans y värde
    %y(2) = y' för kulan
    %(y(3),y(4)) = (x,y) positionen för roboten
    %Utdata:
    %dydt vektor värden
    if nargin == 2

        v = 5;
        a = -3;
        b = 0.1;

    end
    dydt = zeros(4,1);
    dist = sqrt(y(3).^2 + (y(1)-y(4)).^2);
    dydt(1) = y(2);
    dydt(2) = a + b*(y(2).^2);
    dydt(3) = (-y(3)*v)./dist;
    dydt(4) = (v*(y(1)-y(4)))./dist;
end