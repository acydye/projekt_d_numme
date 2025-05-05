function [y, e_pres] = avrunda(x,error)
    %Indata:
    %x: värde
    %error: absolut fel
    %utdata
    %y: Avrundat värde
    %e_pres: Presentations fel
    digit = floor(-log10(abs(error / x))); %Värde siffror
    y = round(x, digit, 'significant');
    e_pres = abs(y-x);

end