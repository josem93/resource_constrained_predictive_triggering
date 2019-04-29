function getExitProbPDE(sim,file1,file2,folder)
% intermediate function to run the numerical solutions of the 1,2,3D PDE
%
% sim:      structure containing simulation parameters
% file1:    filename for lookup table [string]
% file2:    filename for reset probability [string]
% folder:   save location folder name
%
%--------------------------------------------------------------------------

setGlobalParams(sim.A,sim.Q,sim.delta,sim.dt);
z = sim.Z0_set;

runtime = tic;
if(sim.nx == 1)
    
    if(strcmp(sim.TRIG_TYPE,'ST'))          % for ST
        t = 0:sim.dt:30*sim.dt;
        Sprob = solve_PDE_1D(z,t,sim.TRIG_TYPE);
        prob = 1 - Sprob;
        
        figure;
        plot(0:length(t)-1,prob);
        xlabel('M');
        ylabel('P(\tau < M\Delta t | Z(0)=0)');
        
    elseif(strcmp(sim.TRIG_TYPE,'PT'))      % for PT
        t = linspace(0,2*sim.M*sim.dt,3);    
        Sprob = solve_PDE_1D(z,t,sim.TRIG_TYPE);
        prob = 1 - Sprob;
        prob = prob';
        reset_prob = interp1(z,prob,0);
%         figure;
%         plot(z,prob);
%         xlabel('Error Z');
%         ylabel('P(\tau < M\Delta t | Z(0)=z)');
    end
    

elseif(sim.nx == 2)
 
    if(strcmp(sim.TRIG_TYPE,'ST'))
        t = 0:sim.dt:30*sim.dt;
        Sprob = solve_PDE_2D(z,t,sim.TRIG_TYPE);
        prob = 1 - Sprob;
        
%         figure;
%         plot(0:length(t)-1,prob);
%         xlabel('M');
%         ylabel('P(\tau < M\Delta t | Z(0)=0)');
        
    elseif(strcmp(sim.TRIG_TYPE,'PT'))
        t = linspace(0,2*sim.M*sim.dt,3);
        Sprob = solve_PDE_2D(z,t,sim.TRIG_TYPE);
        prob = 1 - Sprob;

%         figure;
%         surf(z,z,prob);
%         xlabel('Error Z');
%         ylabel('Error Z');
%         zlabel('P(\tau < M\Delta t | Z(0)=z)');     
    end

elseif(sim.nx == 3)
    
%     t = 0:sim.dt/2e1:2*sim.M*sim.dt;
    t = linspace(0,2*sim.M*sim.dt,3);
    Sprob = solve_PDE_3D(sim.Z0_set,t);
    prob = 1 - Sprob;
    
%     xslice = [-0.01,0,0.01];
%     lvls = 0:0.1:1;
%     [X,Y,Z] = meshgrid(sim.Z0_set,sim.Z0_set,sim.Z0_set);
%     figure;
%     contourslice(X,Y,Z,prob,xslice,[],[],lvls)
%     colorbar
%     view(3)
%     grid on
else
   
end
toc(runtime);

% Save lookup table
fprintf('\tSaving Numerical lookup table...');
cd(folder);
save(file1,'prob');
save(file2,'reset_prob');
cd('../');
fprintf('Done\n');
clear global;

end


function setGlobalParams(Ad,Qd,delta,dt)
    global w x y z
    w = Ad;
    x = Qd;
    y = delta;
    z = dt;
end

