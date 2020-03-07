%Generates NONLINEAR state prediction table using threeLinkEEForce simulation

fx = [0 0];
fy = [0 0];
fz = [0 0];

j0pi = 0;
j0vi = 0;
j1vi = 0;
j1pi = 0;
j2pi = 0;
j2vi = 0;

j0PosRes = 3;
j1PosRes = 7;
j2PosRes = 7;
j0PosPoints = linspace(-180,180,j0PosRes);
%j1 is backwards, I should probably fix this at some point
j1PosPoints = linspace(-105,45,j1PosRes);
j2PosPoints = linspace(-20,120,j2PosRes);

% j0VelRes = 4;
% j1VelRes = 4;
% j2VelRes = 4;
% j0VelPoints = linspace(-120,120,j0VelRes);
% j1VelPoints = linspace(-120,120,j1VelRes);
% j2VelPoints = linspace(-120,120,j2VelRes);

j0VelPoints = [-100 -20 -10 0 10 20 100];
j1VelPoints = [-100 -20 -10 0 10 20 100];
j2VelPoints = [-100 -20 -10 0 10 20 100];

j0VelRes = length(j0VelPoints);
j1VelRes = length(j1VelPoints);
j2VelRes = length(j2VelPoints);

count = 1;

for j0PosCount = 1:j0PosRes
    j0pi = j0PosPoints(j0PosCount);
    for j1PosCount = 1:j1PosRes
        j1pi = j1PosPoints(j1PosCount);
        for j2PosCount = 1:j2PosRes
            j2pi = j2PosPoints(j2PosCount);
            for j0VelCount = 1:j0VelRes
                j0vi = j0VelPoints(j0VelCount);
                for j1VelCount = 1:j1VelRes
                    j1vi = j1VelPoints(j1VelCount);
                    for j2VelCount = 1:j2VelRes
                        j2vi = j2VelPoints(j2VelCount);

                        %adjust joint limits as initial conditions change
                        j1ll = j1pi - 105;
                        j1ul = j1pi + 45;
                        j2ll = j2pi - 20;
                        j2ul = j2pi + 120; 

                        %run simulation again with updated runtime vars
                        simOut = sim('threeLinkEEForce.slx');
                        j0vf = simOut.j0vf(end);
                        j1vf = simOut.j1vf(end);
                        j2vf = simOut.j2vf(end);

                        j0pf = simOut.j0pf(end);
                        j1pf = simOut.j1pf(end);
                        j2pf = simOut.j2pf(end);
%                         j0vf = out.j0vf(end);
%                         j1vf = out.j1vf(end);
%                         j2vf = out.j2vf(end);
%                         j0pf = out.j0pf(end);
%                         j1pf = out.j1pf(end);
%                         j2pf = out.j2pf(end);

                        prediction = [j0pf, j1pf, j2pf, j0vf, j1vf, j2vf];

                        simOutput(:,count) = prediction;
                        count = count + 1
                    end
                end
            end
        end
    end
    
end

csvwrite('predictionTable377777.txt',simOutput)

return