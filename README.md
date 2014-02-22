agentSchool
========

Description:
-------------

agentSchool is a c++/python project which aims to reproduce the schooling/boids behavior of certains animals, typically fishes. The output of the project is a 2D view of an bounded box containing multiple agents and a predator. Each agent's behavior is driven by a multi-target command corresponding to the solution of a quadratic problem.

For now, the predator is initially set with a random direction and only moves forward with a constant speed.
Agents are initially placed in random positions in the aquarium, with random directions. They are able to rotate around their yaw axis up to a maximum angular speed limit, and they can move forward up to a maximum speed limit. They constantly know where is the predator, and they see the other agents up to a maximum distance and up to a maximum angle of view. The agents that one agent can see are called its 'neighbors'. 
Each agent moves are determined by a quadratic problem which models the following target:

1. Fleeing from the predator 
  * Turning agent's back away from predator
  * Maximize the distance between the agent and the predator

2. Schooling/Boids behavior
  * Minimizing the distance between the agent and its neighbors' barycenter
  * Minimizing the difference between the agent direction and its neighbors' mean direction
  * Keeping a minimum distance with the other agents

3. Natural behavior
  *  Wandering around

This model, inspired by [Craig Reynolds's work](http://www.red3d.com/cwr/boids/), is quite rough for now, but I would like to make it work before going any further. Also, each target has a weight in the quadratic QP. I expect the seek for these coefficients to be particulary hard, as they probably will not be constant. For instance, when an agent is far from the predator, it should not give as much importance to the target 'Fleeing from the predator' as if the predator was very close.

I think about implementing a genetic algorithm to determine these coefficients. This could be done by giving the predator the ability to 'eat' agents for instance. Hopefully it will make it easier to converge to a wanted global behavior.

Understand that this project is some kind of prototype and of course I am not done finishing it.

Dependencies:
-------------
1. System tools:
  * Python (>=2)
  * CMake (>=2.8)
  * Usual compilation tools (GCC/G++, make, etc.)
2. Libraries:
  * [Boost](http://www.boost.org/) (>=1.55.0): a set of libraries for the C++ programming language that provide support for a huge number of tasks and structures. Here I only use it for python bindings
  * [Pygame](http://pygame.org/): a set of Python modules designed for writing games. I use it for its 2D animation modules.

Install and Start:
------------------
Make sure you have all the dependencies.
Go to your agentSchool folder, then 'mkdir build' and 'cd build'.
Then 'cmake .. && make && python agentSchool.py'




