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
ymax = 0.85;
zmin = 0.2;
zmax = 0.5;
fidelity = 0.1;

numPts = (xmax-xmin)*(ymax-ymin)*(zmax-zmin)/(fidelity^3)

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

%get rid of imaginary components and convert to deg
theta0 = rad2deg(real(theta0));
theta1 = 180 - rad2deg(real(theta1));
theta2 = rad2deg(real(theta2));

%loop through xyz points using input angles
i = 1;

for i = 1:length(theta0)
    j0pi = theta0(i);
    j1pi = theta1(i);
    j2pi = theta2(i);
    
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
    ax = simOut.ax(end);
    Ix = fx(2)/ax;

    %y
    fx = [0 0];
    fy = [0 10];
    fz = [0 0];
    simOut = sim('threeLinkEEForce.slx');
    ay = simOut.ay(end);
    Iy = fy(2)/ay;
    
    %z
    fx = [0 0];
    fy = [0 0];
    fz = [0 10];
    simOut = sim('threeLinkEEForce.slx');
    az = simOut.az(end);
    Iz = fz(2)/az;
 
    ellAxis(:,i) = [Ix, Iy, Iz];

end

csvwrite('ellAxis.txt',ellAxis)

return
