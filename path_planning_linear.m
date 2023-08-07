     function [] = path_planning_linear(app, point1, point2)
            v_max = app.V_maxEditField.Value;
            a_max = app.a_maxEditField.Value
            [Pos, Vel, Acce, time] = app.motion_profile(0, norm(point2-point1), v_max, a_max); 
            path = point1 + Pos.*(point2-point1)/norm(point2-point1);
            delta_t = time(2) - time(1);
            x_dot = diff(path(1,:))/delta_t;
            y_dot = diff(path(2,:))/delta_t;
            z_dot = diff(path(3,:))/delta_t;
                        
            x_dot = [x_dot,0];
            y_dot = [y_dot,0];
            z_dot = [z_dot,0];
                        
            T = (length(x_dot))*0.01; 
            
            [~, point_num] =  size(path);
                
            theta1_list = [];
            theta1dot_list = [];
            
            theta2_list = [];
            theta2dot_list = [];
            
            theta3_list = [];
            theta3dot_list = [];
            
            theta4_list = [];
            theta4dot_list = [];
            theta1_dot2 = [];
            theta2_dot2 = [];
            theta3_dot2 = [];
            theta4_dot2 = [];
            R_ee_Pre = zeros(3,3);
            for i  = 1:point_num
                cla(app.UIAxes);

                [theta1, theta2, theta3, theta4] = app.inverse_kinematics(path(1, i), path(2, i), path(3, i), path(4, i));
                app.theta1 = theta1; 
                app.theta2 = theta2; 
                app.theta3 = theta3; 
                app.theta4 = theta4; 
                

                app.DrawRobot(); 
                
                
                p = plot3(app.UIAxes, path(1, 1:i), path(2, 1:i), path(3, 1:i),'Color','red'); 
                p.LineWidth = 3;
                scatter3(app.UIAxes, point1(1), point1(2), point1(3),'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75]);
                scatter3(app.UIAxes, point2(1), point2(2), point2(3),'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75]);

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
                
                p = plot(app.UIAxesP,time(1:i) ,Pos(1:i), 'Color','blue');
                p.LineWidth =2;
                
                p = plot(app.UIAxesV,time(1:i)  ,Vel(1:i), 'Color','blue');
                p.LineWidth =2;
                
                p = plot(app.UIAxesA,time(1:i) , Acce(1:i), 'Color','blue');
                p.LineWidth =2;
                app.EditField.Value = num2str(app.theta1);
                app.EditField_2.Value = num2str(app.theta2);
                app.EditField_3.Value = num2str(app.theta3);
                app.EditField_4.Value = num2str(app.theta4);
                pause(5/1000);
                R_ee_Pre = app.R_EE;
                if app.Stop_animation == 1
                    break;
                end
            end
        end