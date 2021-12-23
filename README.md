# Spatial Consensus-Prevention in Robotic Swarms - Implementation in the MASON Flockers domain
Code for implementation of the consensus-prevention settings in the <a href="https://cs.gmu.edu/~eclab/projects/mason/">MASON simulator</a>, demonstrating the impact of the number of diverting agents on the disagreement measure in different scenarios. If any part of this code is used, the following paper must be cited: 

Saar Cohen and Noa Agmon. Spatial Consensus-Prevention in Robotic Swarms. <em>In AAMAS'21: Proceedings of the 20th International Conference on Autonomous Agents and Multiagent Systems, 2021</em>.

## Dependencies
The code is written in Java version 1.8.0_131, and requires:
- [MASON simulator](https://cs.gmu.edu/~eclab/projects/mason/)

## Simulation Environment
This simulator encodes all the dynamics as they are described in the previous sections, where each agent points and moves in the direction of its current velocity vector. We made a few alterations to the the MASON Flockers domain, such that they will fit our needs. It was initially altered to also contain <em>diverting agents</em>. Another modification was making the <em>flocking agents</em> update their orientation according to the average orientation of all agents in their respective neighborhood at time step t. For more realistic implications of the simulator, its toroidal feature was removed. That is, if an agent moves off of an edge of our domain, it will not reappear and will remain <em>"lost"</em> forever.

### Placement Methods
For guaranteeing that the <em>flocking</em> neighbors graph does indeed consist of a specific number of connected components, we consider the <em>grid placement method</em> and the <em>random placement method</em> proposed in the paper ["Converging to a Desired Orientation in a Flock of Agents"](https://arxiv.org/abs/2010.04686), according to which each pair of successive flocking agents are within a radius of at most R from each other.

### Concept
The three main files are as follows:
- <code>Flocker.java</code> - Encodes a single agent (either a flocking agent or a diverting agent) by implementing the `sim.engine.Steppable` interface.
- <code>Flockers.java</code> - Each model situated in the MASON simulator is entirely encapsulated in a special object called `sim.engine.SimState`. As such, this module sets up the flockers' field, in accordance to the user's inputs provided through the console.
- <code>FlockerWithUI.java</code> - All GUI elements in a visualized MASON model are also encapsulated by an instance of a subclass of `sim.display.GUIState`. The `GUIState` object knows about the `SimState` model object, but not the other way around, and ultimately encapsulates all the elements mentioned in the 'SimState' model object to visualize and control the model.

### Execution
1. Import [MASON](https://cs.gmu.edu/~eclab/projects/mason/) into Eclipse (or any other IDE of your choice).
2. Replace all three files, situated under the `sim.app.flockers` class, which can be founde under the found under the following sub-directory: `/mason/src/main/java/sim/app/flockers/`.
3. Execute MASON's console and select the `Flockers` simulation environement.
4. Under the `Flockers` simulation environement, the following can be performed:
	1. Under the `Console` tab, various parameters regarding the speed of the simulation's execution can be altered in regard with the user's choice.
	2. Under the `Model` tab, the baseline settings for variables can also be altered, including:
		1. Domain height and width
		2. Agent velocity
		3. Visibility radius
		4. Placement method, which can be either `grid` or `random`.
		5. Number of consequent executions
		6. Number of flocking agents per each connected component - the simulation supports three different abstract scenarios:
			1. `equal` - All connected components consist of an equal nubmer of flocking agents. Reagarding this scenario, the number of flocking agents shall be also specified.
			2. `i X m` - The i-th connected component consists of `i X m` flocking agents, where `m` denotes the number of diverting agents.
			3. `i X 10` (Default) - The i-th connected component consists of `i X 10` flocking agents.
