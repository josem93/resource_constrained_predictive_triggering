function survival_prob = solve_PDE_1D(z,t,type)
% Funtion to solve the 1D PDE
%
% Inputs:
%   x       vector of spatial values at which solution should be computed
%   t       vector of time values at which solution should be computed
%   type    trigger type
%
% Output:
%   survival_prob   solution of PDE (ie. survival probability)
%
%--------------------------------------------------------------------------

m = 0;

% computes solution for all T in [0,2*Mdt]
% pde_sol = pdepe(m,@pdeEqn,@pdeIC,@pdeBC,x,t);
% 
% figure;
% surf(x,t,1-pde_sol);
% xlabel('Distance x');
% ylabel('Time t');

mstep_sol = pdepe(m,@pdeEqn,@pdeIC,@pdeBC,z,t);

if(strcmp(type,'ST'))
    % get solution for t = M*dt for M in (0,20) at z=0    
    survival_prob = mstep_sol(:,ceil(end/2));

elseif(strcmp(type,'PT'))
    % get solution for t = M*dt
    [survival_prob,~] = pdeval(m,z,mstep_sol(2,:),z);
end

end


% --------------------------------------------------------------
function [c,f,s] = pdeEqn(x,t,u,DuDx)
    [Ad,Qd,~,dt] = getModelParams;
    Ac = (1/dt)*(Ad - eye(size(Ad,1)));
    Qc = sqrt(Qd(1,1)/dt);
    c = 1;
    f = 0.5*Qc^2*DuDx;
    s = Ac*x*DuDx;
end
% --------------------------------------------------------------
function u0 = pdeIC(x)
    u0 = 1;
end
% --------------------------------------------------------------
function [pl,ql,pr,qr] = pdeBC(xl,ul,xr,ur,t)
    pl = ul;
    ql = 0;
    pr = ur;
    qr = 0;
end


function [Ad,Qd,delta,dt] = getModelParams
    global w x y z
    Ad = w;
    Qd = x;
    delta = y;
    dt = z;
end