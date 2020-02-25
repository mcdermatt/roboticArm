%Generates state prediction table using threeLinkEEForce simulation

j0pi = 0;
j0vi = 0;
j1vi = 0;
j1pi = 0;
j2pi = 0;
j2vi = 0;

count = 1;
while count < 100
    
    j0vi = count * 0.1;
    j1vi = count * 0.1;
    j2vi = count * 0.1; 
    
    %adjust joint limits as initial conditions change
    
    %run simulation again with updated runtime vars
    simOut = sim('threeLinkEEForce.slx');
    j0vf = simOut.j0vf(end);
    j1vf = simOut.j1vf(end);
    j2vf = simOut.j2vf(end);
    
    j0pf = simOut.j0pf(end);
    j1pf = simOut.j1pf(end);
    j2pf = simOut.j2pf(end);
    
    prediction = [j0pf, j1pf, j2pf, j0vf, j1vf, j2vf];
    
    simOutput(:,count) = prediction
    
    count = count + 1;

end

csvwrite('predictionTable.txt',simOutput)

return