%Generates state prediction table using threeLinkEEForce simulation

j0pi = 0;
j0vi = 0;
j1vi = 0;
j1pi = 0;
j2pi = 0;
j2vi = 0;

count = 0;
while count < 10
    
    j1vi = count * 10;
    j2vi = count * 10; 
    
    %adjust joint limits as initial conditions change
    
    simOut = sim('threeLinkEEForce.slx');
    j0vf = simOut.j0vf(end)
    j1vf = simOut.j1vf(end)
    j2vf = simOut.j2vf(end)
    
    count = count + 1;

end