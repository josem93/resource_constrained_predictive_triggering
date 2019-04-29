function survival_prob = solve_PDE_3D(z,t)
% Funtion to solve the 3D PDE
%
% Inputs:
%   z       vector of spatial values at which solution should be computed
%   t       vector of time values at which solution should be computed
%
% Output:
%   survival_prob   solution of PDE (ie. survival probability)
%
%--------------------------------------------------------------------------

[Ad,Qd,delta,dt] = getModelParams;
Ac = (1/dt)*(Ad - eye(size(Ad,1)));
Qc = sqrt(Qd)/sqrt(dt);

model = createpde(1);

gm = multisphere(delta);
model.Geometry = gm;
% pdegplot(model,'CellLabels','on');

% PDE coefficients [TODO]
c1 = 0.5*(Qc(1,1)^2 + Qc(1,2)^2 + Qc(1,3)^2);
c2 = Qc(1,1)*Qc(2,1);
c3 = Qc(1,1)*Qc(3,1);
c4 = Qc(1,2)*Qc(2,2) + Qc(1,3)*Qc(2,3);
c5 = 0.5*(Qc(2,1)^2 + Qc(2,2)^2 + Qc(2,3)^2);
c6 = Qc(2,1)*Qc(3,1);
c7 = Qc(1,2)*Qc(3,2) + Qc(1,3)*Qc(3,3);
c8 = Qc(2,2)*Qc(3,2) + Qc(2,3)*Qc(3,3);
c9 = 0.5*(Qc(3,1)^2 + Qc(3,2)^2 + Qc(3,3)^2);
c = [c1 c2 c3 c4 c5 c6 c7 c8 c9]';
f = @(location,state) location.x.*(Ac(1,1).*state.ux + Ac(2,1).*state.uy + Ac(3,1).*state.uz) ...
                    + location.y.*(Ac(1,2).*state.ux + Ac(2,2).*state.uy + Ac(3,2).*state.uz) ...
                    + location.z.*(Ac(1,3).*state.ux + Ac(2,3).*state.uy + Ac(3,3).*state.uz);
                    
specifyCoefficients(model,'m',0,...
                          'd',1,...
                          'c',c,...
                          'a',0,...
                          'f',f);
                      
% Initial Conditions
setInitialConditions(model,1);

% Boundary Conditions
applyBoundaryCondition(model,'dirichlet','Face',1,'u',0);

% Mesh
hmax = delta/20; % element size
mesh = generateMesh(model,'Hmax',hmax);
% pdeplot(model);

% Solver options
model.SolverOptions.RelativeTolerance = 1.0e-3;
model.SolverOptions.AbsoluteTolerance = 1.0e-4;

f = exist('pde3D.mat','file');
if(~f)
    tPDE = tic;
    fprintf('Starting PDE Solver\n');

    sol = solvepde(model,t);
    runtime = toc(tPDE);
else
    load('pde3D.mat');
end

[X,Y,Z] = meshgrid(z,z,z);
% get value at Mdt
survival_prob = interpolateSolution(sol,X,Y,Z,2);
survival_prob = reshape(survival_prob,size(X));

fprintf(['\tNumerical completed in ',num2str(runtime),' seconds\n\n']);

end



function [Ad,Qd,delta,dt] = getModelParams
    global w x y z
    Ad = w;
    Qd = x;
    delta = y;
    dt = z;
end