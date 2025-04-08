function [rot,error,error_list,t_list,iter] = sekant(func, t0, t1, tol, max_n)
    %Secant metoden
    %Indata:
    %func - functionen man vill hitta rot till
    %t0, t1 - start gissning
    %tol - tolerans
    %max_n - max iterationen
    %Utdata:
    %rot - hittat rot
    %error_list - fel vid varje iteration
    %iter - antal iterationer

    error_list = [];
    t_list = [t0,t1];
    iter = 0;

    if nargin == 3, tol = 1e-10; end
    if nargin == 4, max_n = 20; end
    

    for i = 1:max_n
        iter = iter + 1;
        f1 = func(t1); f0 = func(t0);
        df = f1 - f0;
    
        if abs(df) < eps
            disp('Sekant funktion error - division med noll')
        end
    
        dt = f1*(t1-t0)/(f1-f0);
        t0 = t1; t1 = t1 - dt; 

        error_list(end+1) = abs(dt);
        t_list(end+1) = t1;
        
        if abs(dt) < tol
            rot = t1;
            error = abs(dt);
            return;
        end

        
    end
    
    warning('Sekant hittade ej rot efter %d iterationer', max_n)
    rot = t1;
end