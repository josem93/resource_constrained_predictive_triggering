function survival_prob = solve_PDE_2D(z,t,type)
% Funtion to solve the 2D PDE
%
% Inputs:
%   z       vector of spatial values at which solution should be computed
%   t       vector of time values at which solution should be computed
%   type    trigger type
%
% Output:
%   survival_prob   solution of PDE (ie. survival probability)
%
%--------------------------------------------------------------------------

[Ad,Qd,delta,dt] = getModelParams;
Ac = (1/dt)*(Ad - eye(size(Ad,1)));
Qc = sqrt(Qd)/sqrt(dt);

model = createpde(1);
geometryFromEdges(model,@circlefunction);

% pdegplot(@circlefunction,'EdgeLabels','on','FaceLabels','on');

% PDE coefficients [TODO]
c1 = 0.5*(Qc(1,1)^2 + Qc(1,2)^2);
c2 = Qc(1,1)*Qc(2,1);
c3 = Qc(1,2)*Qc(2,2);
c4 = 0.5*(Qc(2,1)^2 + Qc(2,2)^2);
c = [c1 c2 c3 c4]';
f = @(location,state) location.x.*(Ac(1,1).*state.ux + Ac(2,1).*state.uy) ...
                    + location.y.*(Ac(1,2).*state.ux + Ac(2,2).*state.uy);

specifyCoefficients(model,'m',0,...
                          'd',1,...
                          'c',c,...
                          'a',0,...
                          'f',f);
                      
% Initial Conditions
setInitialConditions(model,1);

% Boundary Conditions
applyBoundaryCondition(model,'dirichlet','Edge',1:4,'u',0);

% Mesh
hmax = delta/20; % element size
mesh = generateMesh(model,'Hmax',hmax);
% pdeplot(model);                   

% Solver options
model.SolverOptions.RelativeTolerance = 1.0e-3;
model.SolverOptions.AbsoluteTolerance = 1.0e-4;

sol = solvepde(model,t);


if(strcmp(type,'ST'))
    % get solution for t = M*dt for M in (0,20) at z=0    
    survival_prob = 1;

elseif(strcmp(type,'PT'))
    % get value at Mdt
    [X,Y] = meshgrid(z,z);
    survival_prob = interpolateSolution(sol,X,Y,2);
    survival_prob = reshape(survival_prob,size(X));
end





% u = sol.NodalSolution;
% figure;
% pdeplot(model,'XYData',u(:,2),'Contour','on','ColorMap','jet');
% title(sprintf('Temperature In The Plate, Transient Solution( %d seconds)\n', ...
%   t(1,2)));
% xlabel 'X-coordinate, meters'
% ylabel 'Y-coordinate, meters'
% axis equal;

end



% Create circular boundary condition using four segments.
function [x,y] = circlefunction(bs,s)
    [~,~,r] = getModelParams;
    switch nargin
        case 0
            x = 4; % four edge segments
            return
        case 1
            A = [0,pi/2,pi,3*pi/2; % start parameter values
                 pi/2,pi,3*pi/2,2*pi; % end parameter values
                 1,1,1,1; % region label to left
                 0,0,0,0]; % region label to right
            x = A(:,bs); % return requested columns
            return
        case 2
            x = r*cos(s);
            y = r*sin(s);
    end
end

function [Ad,Qd,delta,dt] = getModelParams
    global w x y z
    Ad = w;
    Qd = x;
    delta = y;
    dt = z;
end