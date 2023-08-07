    
        function [theta1, theta2, theta3, theta4] = inverse_kinematics(app, px, py, pz, pitch)
            d1 = 7.7; 
            a2 = 13.0; 
            a3 = 12.4; 
            a4 = 12.6; 
            Phi = -pitch;
            delta_theta = atan(12.8/2.4)*180/pi;
            theta1 = atan2d(py, px); 
            pr = sqrt(px^2+py^2);    
            r3 =  pr;
            z3 = pz - d1; 
            
            r2 = r3 - a4*cosd(Phi);
            z2 = z3 - a4*sind(Phi);
            c3 = (r2^2 + z2^2 - a2^ 2 - a3^2)/(2*a2*a3);
            s3 = [-sqrt(1-c3^2), sqrt(1-c3^2)];
            try
                theta3_ = atan2d(s3, [c3,c3]);
                theta3_list = theta3_ + delta_theta;
            
                c2 = ((a2 + a3*cosd(theta3_))*r2 + (a3*sind(theta3_))*z2)/(r2^2+z2^2);
                s2 = ((a2 + a3*cosd(theta3_))*z2 - (a3*sind(theta3_))*r2)/(r2^2+z2^2);
                theta2_ = atan2d(s2, c2); 
                theta2_list = theta2_ - delta_theta;
                theta4_list = Phi - theta2_ - theta3_;
                
                index = []; 
                for i =1:2
                    if (theta2_list(i) + theta3_list(i) + theta3_list(i)) >= -90  || (theta2_list(i) + theta3_list(i) + theta3_list(i)) <= 90
                        index = [index, i];
                    end
                end
                if length(index) == 0
                    f = warndlg('out of work space','Warning');
                    theta1 = app.theta1; 
                    theta2 = app.theta2; 
                    theta3 = app.theta3; 
                    theta4 = app.theta4;
                    app.Stop_animation = 1;
                      
                end
                theta2_list = theta2_list(index);
                theta3_list = theta3_list(index);
                theta4_list = theta4_list(index);
                [~, index_resturn] = min(abs(theta2_list-app.theta2) + abs(theta3_list-app.theta3) + abs(theta4_list-app.theta4));
                theta2 = theta2_list(index_resturn); 
                theta3 = theta3_list(index_resturn);
                theta4 = theta4_list(index_resturn);

            catch 
                f = warndlg('out of work space','Warning');
                theta1 = app.theta1; 
                theta2 = app.theta2; 
                theta3 = app.theta3; 
                theta4 = app.theta4;
                app.Stop_animation = 1;
            end
        end
        