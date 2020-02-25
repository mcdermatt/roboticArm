%Generates state prediction table using threeLinkEEForce simulation

j0pi = 0;
j0vi = 0;
j1vi = 0;
j1pi = 0;
j2pi = 0;
j2vi = 0;

sim('threeLinkEEForce.slx')

ans = out.j1vf(end)

count = 0;
while count < 5
    
    j1vi = count * 10
    j2vi = count * 10 
    sim('threeLinkEEForce.slx')
    ans = out.j1vf(end)
    
    count = count + 1

end