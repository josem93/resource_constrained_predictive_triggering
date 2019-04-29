function net = network_setup(varargin)
% Funtion to set up the network
%
% Inputs:
%   sim     structure containing simulation parameters
%   net     structure containing network parameters
%   N       number of system agents
%
% Output:
%   net     updated network structure with bandwidth, capcity and number of
%           slots
%--------------------------------------------------------------------------

sim = varargin{1};
net = varargin{2};

if(nargin < 3)
    Np = sim.N;
else
    Np = varargin{3};
end

% capacity per time step [bytes]
net.capacity = sim.dt*net.b;

% number of communication slots available
if(sim.P_CONSTRAINT)
    net.K = floor((net.capacity - (4+2)*Np)/(4*sim.nx));
else
    net.K = floor((net.capacity - 4*Np)/(4*sim.nx));    
end