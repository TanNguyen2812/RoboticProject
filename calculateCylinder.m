   function [X, Y, Z] = calculateCylinder(app, p0, p1, R)

            v = p1 - p0;
            mag = norm(v);
            v = v/mag;
            not_v = [1, 0.1, 0.1232]'; 
            if isequal(v, not_v)
                not_v = [-0.1,1,0]';
            end
            n1 = cross(v, not_v);
            n1 = n1/norm(n1);
            n2 = cross(v,n1);
            t = linspace(0, mag, 100);
            theta = linspace(0,2*pi, 100);
            [t, theta] = meshgrid(t, theta);
            X = p0(1) + v(1) * t + R * sin(theta) * n1(1) + R * cos(theta) * n2(1);
            Y = p0(2) + v(2) * t + R * sin(theta) * n1(2) + R * cos(theta) * n2(2);
            Z = p0(3) + v(3) * t + R * sin(theta) * n1(3) + R * cos(theta) * n2(3);
        end 
        
        function [] = DrawLine(app, p1, p2, color)
            p = plot3(app.UIAxes, [p1(1);p2(1)], [p1(2);p2(2)],[p1(3);p2(3)]);
            p.LineWidth = 3;
            p.Color = color;
        end