     
        function [] = path_planning_circular(app,P0, P1, P2)
            
            v_max = app.V_maxEditField.Value;
            a_max = app.a_maxEditField.Value; 
            
            
            P01 = P1-P0; 
            P02 = P2-P0;
            X = P01/norm(P01);
            Z = cross(P01, P02)/norm(cross(P01, P02));
            Y = cross(Z,X);
            T = [X, Y, Z, P0;0,0,0,1];
            P1_ = (T^-1)*[P1;1];
            P1_ = P1_(1:3);
            
            P2_ = (T^-1)*[P2;1];
            P2_ = P2_(1:3);
            C_ = [P1_(1)/2;((P2_(1) - P1_(1)/2)^2 + P2_(2)^2 -(P1_(1)/2)^2)/(2*P2_(2));0];
            C = T*[C_;1];
            C = C(1:3); 
            R = norm(C_);
            
            p_init = atand(C_(2)/C_(1)); 
            p_max = acosd(-(P2_ - C_)'*C_/R^2) ;
            [Pos, Vel, ACCE,time] = app.motion_profile(p_init,p_max , v_max, a_max);
            L = length(Pos);
            path = zeros(3, L);
            for i=1:L
                P_ = R*[-cosd(Pos(i));-sind(Pos(i));0] + C_; 
                P = T*[P_;1]; 
                P = P(1:3);
                path(1,i) = P(1);
                path(2,i) = P(2);
                path(3,i) = P(3);
            end
             
            delta_t = time(2) - time(1);
            x_dot = diff(path(1,:))/delta_t;
            y_dot = diff(path(2,:))/delta_t;
            z_dot = diff(path(3,:))/delta_t;

            x_dot = [0,x_dot];
            y_dot = [0,y_dot,];
            z_dot = [0,z_dot,];
            
          
            pitch_dot = zeros(size(x_dot));
            [~, point_num] =  size(path);

            theta1_list = [];
            theta1dot_list = [];
            
            theta2_list = [];
            theta2dot_list = [];
            
            theta3_list = [];
            theta3dot_list = [];
            
            theta4_list = [];
            theta4dot_list = [];
            R_ee_Pre = zeros(3,3);
            for i  = 1:point_num
                cla(app.UIAxes); 

                [theta1, theta2, theta3, theta4] = app.inverse_kinematics(path(1, i), path(2, i), path(3, i), app.pitchAEditField.Value);
                app.theta1 = theta1; 
                app.theta2 = theta2; 
                app.theta3 = theta3; 
                app.theta4 = theta4; 
                

                app.DrawRobot(); 
                p = plot3(app.UIAxes, path(1, 1:i), path(2, 1:i), path(3, 1:i),'Color','red'); 
                p.LineWidth = 3;
                scatter3(app.UIAxes, P0(1), P0(2), P0(3),'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75]);
                scatter3(app.UIAxes, P1(1), P1(2), P1(3),'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75]);
                scatter3(app.UIAxes, P2(1), P2(2), P2(3),'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75]);

                theta1_list = [theta1_list, theta1];
                p = plot(app.UIAxesTheta1,time(1:i), theta1_list,'Color','blue'); 
                p.LineWidth =2;
                
                    
                if (i>1)
                    theta1_dot2 = diff(theta1_list)/delta_t;
                    p = plot(app.UIAxesTheta1dot_2,time(1:i-1), theta1_dot2,'Color','blue'); 
                    p.LineWidth =2;
                end
                
                theta2_list = [theta2_list, theta2];
                if (i>1)
                    theta2_dot2 = diff(theta2_list)/delta_t;
                    p = plot(app.UIAxesTheta2dot_2,time(1:i-1), theta2_dot2,'Color','blue'); 
                    p.LineWidth =2;
                end
                
                
                p = plot(app.UIAxesTheta2,time(1:i), theta2_list,'Color','blue'); 
                p.LineWidth =2;
                
                theta3_list = [theta3_list, theta3];
                if (i>1)
                    theta3_dot2 = diff(theta3_list)/delta_t;
                    p = plot(app.UIAxesTheta3dot_2,time(1:i-1), theta3_dot2,'Color','blue'); 
                    p.LineWidth =2;
                end
                p = plot(app.UIAxesTheta3,time(1:i), theta3_list,'Color','blue'); 
                p.LineWidth =2;
                
                theta4_list = [theta4_list, theta4];
                if (i>1)
                    theta4_dot2 = diff(theta4_list)/delta_t;
                    p = plot(app.UIAxesTheta4dot_2,time(1:i-1), theta4_dot2,'Color','blue'); 
                    p.LineWidth =2;
                end
                p = plot(app.UIAxesTheta4,time(1:i), theta4_list,'Color','blue'); 
                p.LineWidth =2;
                R_dot = (app.R_EE - R_ee_Pre)/delta_t; 
                S = R_dot*app.R_EE';
                wx = S(3,2);
                wy = S(1,3);
                wz = S(2,1); 
                temp = pinv(app.Jacobian_matrix)*[x_dot(i);y_dot(i);z_dot(i);wx;wy;wz]; 

                
                theta1dot = temp(1);
                theta1dot_list = [theta1dot_list, theta1dot*180/pi];
                p = plot(app.UIAxesTheta1dot,time(1:i), theta1dot_list,'Color','red'); 
                p.LineWidth =2; 
                
                theta2dot = temp(2);
                theta2dot_list = [theta2dot_list, theta2dot*180/pi];
                p = plot(app.UIAxesTheta2dot,time(1:i), theta2dot_list,'Color','red'); 
                p.LineWidth =2; 
                
                theta3dot = temp(3);
                theta3dot_list = [theta3dot_list, theta3dot*180/pi];
                p = plot(app.UIAxesTheta3dot,time(1:i), theta3dot_list,'Color','red'); 
                p.LineWidth =2; 
                
                theta4dot = temp(4);
                theta4dot_list = [theta4dot_list, theta4dot*180/pi];
                p = plot(app.UIAxesTheta4dot,time(1:i), theta4dot_list,'Color','red'); 
                p.LineWidth =2; 
                
                p = plot(app.UIAxesP, Pos(1:i), 'Color','blue');
                p.LineWidth =2;
                
                p = plot(app.UIAxesV, Vel(1:i), 'Color','blue');
                p.LineWidth =2;
                
                p = plot(app.UIAxesA, ACCE(1:i), 'Color','blue');
                p.LineWidth =2;
                app.EditField.Value = num2str(app.theta1);
                app.EditField_2.Value = num2str(app.theta2);
                app.EditField_3.Value = num2str(app.theta3);
                app.EditField_4.Value = num2str(app.theta4);
                pause(1/3000);
                if app.Stop_animation == 1
                    break;
                end
                R_ee_Pre = app.R_EE;
            end

            
        end