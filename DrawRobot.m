      function [] = DrawRobot(app)
                
            A01 = app.DHmatrix(7.7, app.theta1, 0, 90);
            n1 = A01(1:3,1); 
            o1 = A01(1:3,2); 
            a1 = A01(1:3,3); 
            p1 = A01(1:3,4);
            
            app.Joint1_0 = A01*[app.Base_2;1];
            app.Joint1_0 = app.Joint1_0(1:3);
            app.Joint1_1 = A01*[app.Base_1;1];
            app.Joint1_1 = app.Joint1_1(1:3);
            
            delta_theta = atan(12.8/2.4)*180/pi;            
            A12 = app.DHmatrix(0, app.theta2+delta_theta, 13.0, 0); 
            A20 = A01*A12;
            
            n2 = A20(1:3,1); 
            o2 = A20(1:3,2); 
            a2 = A20(1:3,3); 
            p2 = A20(1:3,4);
            
        
            app.Joint2_0 = A20*[app.Base_2;1];
            app.Joint2_0 = app.Joint2_0(1:3);
            app.Joint2_1 = A20*[app.Base_1;1];
            app.Joint2_1 = app.Joint2_1(1:3);
            
            A23 = app.DHmatrix(0, app.theta3-delta_theta, 12.4, 0); 
            A30 = A01*A12*A23;
            
            n3 = A30(1:3,1); 
            o3 = A30(1:3,2); 
            a3 = A30(1:3,3); 
            p3 = A30(1:3,4);
            
            
            app.Joint3_0 = A30*[app.Base_2;1];  
            app.Joint3_0 = app.Joint3_0(1:3);
            app.Joint3_1 = A30*[app.Base_1;1];
            app.Joint3_1 = app.Joint3_1(1:3);
            
            A34 = app.DHmatrix(0, app.theta4, 12.6, 0); 
            A40 = A01 *A12*A23*A34; 
            
            n4 = A40(1:3,1); 
            o4 = A40(1:3,2); 
            a4 = A40(1:3,3); 
            p_ee = A40(1:3,4);
            p4 = A40(1:3,4);
            
            app.Joint4_0 = A40*[app.Base_2;1];
            app.Joint4_0 = app.Joint4_0(1:3);
            app.Joint4_1 = A40*[app.Base_1;1];
            app.Joint4_1 = app.Joint4_1(1:3);
            
            hold(app.UIAxes, 'on');
            
            

            
            app.DrawCylinder(app.Base_0,app.Base_1,3,[1 0 0],app.alpha); 
            
            app.DrawCylinder(app.Base_0,(app.Joint1_0+app.Joint1_1)/2,0.5,[0.9290 0.6940 0.1250],app.alpha); 
            
            app.DrawCylinder(app.Joint1_0,app.Joint1_1,1,[0 0 1],app.alpha); 
            
            app.DrawCylinder((app.Joint1_0+app.Joint1_1)/2,(app.Joint2_0+app.Joint2_1)/2,0.5,[0.9290 0.6940 0.1250],app.alpha); 
            
            app.DrawCylinder(app.Joint2_0,app.Joint2_1,1,[0 0 1],app.alpha);
            
            app.DrawCylinder((app.Joint2_0+app.Joint2_1)/2,(app.Joint3_0+app.Joint3_1)/2,0.5,[0.9290 0.6940 0.1250],app.alpha); 
            
            app.DrawCylinder(app.Joint3_0,app.Joint3_1,1,[0 0 1],app.alpha);
            
            app.DrawCylinder((app.Joint3_0+app.Joint3_1)/2,(app.Joint4_0+app.Joint4_1)/2,0.5,[0.9290 0.6940 0.1250],app.alpha); 
            
            %[X,Y,Z] = sphere;
            %S = surf(app.UIAxes,X+p_ee(1),Y+p_ee(2),Z+p_ee(3),'facecolor','#A2142F','LineStyle','none');
            app.DrawCylinder(p_ee+a4,p_ee-a4,0.4,[1 0 1],app.alpha);
            app.DrawCylinder(p_ee+a4+1.5*n4,p_ee+a4-n4,0.5,[1 0 1],app.alpha);
            app.DrawCylinder(p_ee-a4+1.5*n4,p_ee-a4-n4,0.5,[1 0 1],app.alpha);
            %S.FaceAlpha = app.alpha;
           
            app.PxEditField.Value = num2str(p_ee(1));
            app.PyEditField.Value = num2str(p_ee(2));
            app.PzEditField.Value = num2str(p_ee(3));
            
         
            
            R_ee = A40(1:3,1:3);
            app.R_EE = R_ee;
            Pitch_angle = atan2(-R_ee(3,1),sqrt(R_ee(3,2)^2+R_ee(3,3)^2)); 
            if Pitch_angle == pi/2
                Yaw_angle = 0; 
                Roll_angle = atan2(R_ee(1,2), R_ee(2,2));
            
            elseif Pitch_angle == -pi/2
                Yaw_angle = 0; 
                Roll_angle = -atan2(R_ee(1,2), R_ee(2,2));
            else
                Yaw_angle = atan2(R_ee(2,1)/cos(Pitch_angle), R_ee(1,1)/cos(Pitch_angle));
                Roll_angle = atan2(R_ee(3,2)/cos(Pitch_angle), R_ee(3,3)/cos(Pitch_angle));
            end
      
            p0 = [0,0,0]';
            z0 = [0,0,1]'; 
            
            z1 = A01(1:3,3);
            
            z2 = A20(1:3,3);
         
            z3 = A30(1:3,3);
          
            z4 = A40(1:3,3);
            
            J = [cross(z0,(p4-p0)), cross(z1,(p4-p1)), cross(z2,(p4-p2)),cross(z3,(p4-p3));z0,z1,z2,z3]; 
            app.Jacobian_matrix = J; 

            
            
            app.RollEditField.Value = num2str(Roll_angle*180/pi);
            app.PitchEditField.Value = num2str(Pitch_angle*180/pi);
            app.YawEditField.Value = num2str(Yaw_angle*180/pi);
            

            if app.show_axis ~= 0
                app.DrawLine([0,0,0], [2,0,0],[1 0 0]);
                app.DrawLine([0,0,0], [0,2,0],[0 1 0]);
                app.DrawLine([0,0,0], [0,0,2],[0 0 1]);
                

                
                app.DrawLine(p1, n1*2+p1,[1 0 0]);
                app.DrawLine(p1, o1*2+p1,[0 1 0]);
                app.DrawLine(p1, a1*2+p1,[0 0 1]);
                
            

                
                app.DrawLine(p2, n2*2+p2,[1 0 0]);
                app.DrawLine(p2, o2*2+p2,[0 1 0]);
                app.DrawLine(p2, a2*2+p2,[0 0 1]);
                
                

                
                app.DrawLine(p3, n3*2+p3,[1 0 0]);
                app.DrawLine(p3, o3*2+p3,[0 1 0]);
                app.DrawLine(p3, a3*2+p3,[0 0 1]);
                
          
     
                app.DrawLine(p4, n4*2+p4,[1 0 0]);
                app.DrawLine(p4, o4*2+p4,[0 1 0]);
                app.DrawLine(p4, a4*2+p4,[0 0 1]);
                
            end
     
        end