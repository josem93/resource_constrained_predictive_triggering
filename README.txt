====================================================================================================
Predictive Triggering Framework for Distributed Control of Resource Constrained Multi-agent Systems
====================================================================================================

Source code for the paper "Predictive Triggering for Distributed Control of Resource Contrained
Multi-agent Systems", submitted to the 2019 Distributed Estimation and Control in Networked Systems
conference (NecSys 2019). The code (as is curremtly set up) simulates a Cooperative Adaptive Cruise 
Control (CACC) system for on a 5 lane highway, where each lane follows a reference vehicle.

To run the CACC simulations, the user requires Matlab version R2010a or higher (due to class 
definitions) and have the Statistics and Machine Learning toolbox installed. A single simulation may
be performed by running Multi_Agent_Simulation.m. The following are key files needed to modify/analyze
the simulations:

Multi_Agent_Simulation.m:	
	Runs a single simulation using the parameters defined in params.m and
	the CACC vehicle model defined in multi_agent_setup.m

params.m:	
	Defines the simulation, trigger and network parameters needed to run the simulations.

multi_agent_setup.m: 	
	Defines the multi-agent system parameters (ie., number of agents, models)

Agent.m: 	
	Contains the class definition for a single agent, where the methods represnt the function
	blocks (ie., process, predictors, trigger)

/Autonomous Vehicles:
	Folder which contains the lookup tables (currently for sim.delta = 0.01, 
	sim.Q = diag([9e-6,9e-6,9e-6,9e-6])) and reference velocity generator function.
	
**NOTE: if the sim.delta or the sim.Q parameters are changed, new lookup tables need to be computed.



José Mario Mastrangelo
MPI-IS, ICS
josema(at)student.ethz.ch; mastrangelo(at)is.mpg.de

Copyright 2019 Max Planck Society. All rights reserved.
====================================================================================================