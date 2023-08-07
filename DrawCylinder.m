   function [] = DrawCylinder(app, p1 , p2, R, color, alpha)
           
            [X, Y, Z] = app.calculateCylinder(p1, p2, R);
            S1 = surf(app.UIAxes, X, Y, Z,'LineStyle','none');
            S2 = fill3(app.UIAxes, X(:,1),Y(:,1),Z(:,1),'r'); 
            S3 = fill3(app.UIAxes, X(:,100),Y(:,100),Z(:,100),'r');
            S1.FaceColor = color;
            S2.FaceColor = color;
            S3.FaceColor = color;
            
            S1.FaceAlpha = alpha;  
            S2.FaceAlpha = alpha;
            S3.FaceAlpha = alpha;
        end