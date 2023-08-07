        function [p, v, a, t] = motion_profile(app,p_init, p_max, v_max, a_max)
            if v_max > sqrt(a_max*p_max)
                f = warndlg("v_max > " + num2str(sqrt(a_max*p_max)),'Warning');
           
                app.Stop_animation = 1;

            end
            t1 = v_max/a_max; 
            T = p_max/v_max + t1;   
            t2 = T - t1; 
            t = linspace(0,T,100); 
            p = (1/2*a_max*t.^2 + p_init).*(t < t1) + (v_max*(t-t1) + 1/2*a_max*t1^2+p_init).*(t>=t1 & t<t2) + (v_max*(t2-t1) + 1/2*a_max*t1^2 - 1/2*a_max*(t-t2).^2 + v_max*(t-t2)+p_init).*(t>=t2); 
            v = (a_max*t).*(t<t1) + v_max.*(t>=t1 & t<t2) + (-a_max*(t-t2)+v_max).*(t>=t2); 
            a = a_max.*(t<t1) + 0.*(t>=t1 & t<t2) + (-a_max).*(t>=t2); 
        end                     
