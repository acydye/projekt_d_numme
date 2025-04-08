function [x,y] = rkf(odfunc,xspan,y0,h)
    %Runge kutta 4
    %Indata:
    %func - Function, f(x,y)
    %xspan - [x0, xend] start och slut x värden
    %y0 - start vektorn
    %h - steglängd
    %Utdata
    %x - vektor av x värden
    %y - vektor av y värden

    %Slut- och start punkter
    x0 = xspan(1);
    xend = xspan(2);

    %Säkerställer att slutpunkten är med
    N = round(abs(xend-x0)/h);
    h = (xend-x0)/N;

    x = linspace(x0,xend,N+1);
    y = zeros([length(x),length(y0)]);
    y(1,:) = y0;


    for i = 1:length(x)-1
        xn = x(i);
        yn = y(i,:)';
        k1 = odfunc(xn,yn(:));
        k2 = odfunc(xn + h/2, yn(:) + h*0.5*k1);
        k3 = odfunc(xn + h/2, yn(:) + h*0.5*k2);
        k4 = odfunc(xn + h, yn(:) + h*k3);
        K = (k1 + 2*k2 + 2*k3 + k4)/6;
        y(i+1,:) = (yn(:) + h*K(:))';


    end
    x = x';

end