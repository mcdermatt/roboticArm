%generates force ellipses for arm at each point xyz due to fx fy fz
%assumes zero initial velocity

j0vi = 0;
j1vi = 0;
j2vi = 0;
%lengths of links (m)
l0 = 0.0;
l1 = 0.40625;
l2 = 0.59375;

xmin = -0.5;
xmax = 0.5;
ymin = 0.25;
ymax = 0.75;
zmin = 0.2;
zmax = 0.5;
fidelity = 0.1;

numPts = (xmax-xmin)*(ymax-ymin)*(zmax-zmin)/(fidelity^3)
%theta0 = zeros(1,numPts);
theta0 = zeros(1,100);

%loop through workspace generating in xyz getting joint angles for each pos
count = 1;
for x = xmin:0.1:xmax
    for y = ymin:0.1:ymax
        for z = zmin:0.1:zmax
            r = sqrt((x*x)+(y*y)+(z*z));
            phi = atan((sqrt((x  * x) + (z * z)))/ y );
            beta = atan(z/ x);
            
            theta0(count) = beta;
            theta1(count) = phi + acos(((l1^2)-(l2^2)+(r*r))/(-2*l1*r));
            theta2(count) = acos(((l1*l1)+(l2*l2)-(r*r))/(-2*l1*l2));
            
            count = count + 1;
        end
    end
end

%loop through xyz points using input angles
for j0 = 1:length(theta0)
    j0pi = theta0(j0);
    for j1 = 1:length(theta1)
        j1pi = theta1(j1);
        for j2 = 1:length(theta2)
            j2pi = theta2(j2)            
            
             %adjust joint limits as initial conditions change
            j1ll = j1pi - 105;
            j1ul = j1pi + 45;
            j2ll = j2pi - 20;
            j2ul = j2pi + 120;

            %x
            fx = [0 10];
            fy = [0 0];
            fz = [0 0];
            simOut = sim('threeLinkEEForce.slx');
            
            %y
            fx = [0 0];
            fy = [0 10];
            fz = [0 0];
            simOut = sim('threeLinkEEForce.slx');
            
            %z
            fx = [0 0];
            fy = [0 0];
            fz = [0 10];
            simOut = sim('threeLinkEEForce.slx');
                        
                        
        end
    end
end

return
