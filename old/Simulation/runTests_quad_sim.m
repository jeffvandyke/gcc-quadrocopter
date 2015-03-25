% function [rkp,rki,rkd] = runTests_quad_sim()


bestgains = [0;0;0];
step = .25;
max = 5;

rolltotalerror = ones(1,1+(max/step)^3);
rolltotalerror = 10e5*rolltotalerror;
h = waitbar((max/step)^3,'Progress');
for i = 0:step:max
    for d = 0:step:max
        for p = 0:step:max
            waitbar((1 + max^2*i/step^3 + max*d/step^2 + p/step)/(max/step)^3,h)
            fprintf('kP: %g\tkI: %g\tkD: %g\n',p,i,d)
            
            [quadc,history] = simulate_quad_testRuns(p,i,d);

            rolltotalerror(round(1 + max^2*i/step^3 + max*d/step^2 + p/step)) = history.rolld;
            
            if min(rolltotalerror) == rolltotalerror(round(1 + max^2*i/step^3 + max*d/step^2 + p/step))
                bestgains = [p i d];
            end
            
        end
    end
end

plot(rolltotalerror)
min(rolltotalerror)