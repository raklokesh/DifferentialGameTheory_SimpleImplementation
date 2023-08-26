# Differential Game Theory: Illustrative Implementation

Implementation of differential game theoretic solution for a simple spatial negotiation problem between two agents.

## Problem Statement
<p align="justify"> 
Two agents have to move from a start position to an end position in two dimensional space (Lateral (x) and Forward (y)). Each agent has to also pass through a via-point before reaching the end target. The agents have to reach the end target and pass through the via-point at prespecified time instants.
</p>
<p align="center"> 
<img src="https://github.com/raklokesh/DifferentialGameTheory_SimpleImplementation/blob/main/Task%20Illustration.jpg" width="400px">
</p>

<p align="justify"> 
Each agent has a cost function thatvit aims to minimize: The cost function consists of the control effort, distance from its via-point at the specified time instant, and distance from the end target at the specified time instant. However, the cost functions of the two agents will be tied to one another when the agents are connected through a spring between them. The agents have to negotiate movement paths to minimize their control effort.
</p>

## Script Descriptions
Here are the scripts needed to run the simulations.
* model_params - common parameters of the task and models.
* models_clean - model classes for different agent models.
* run_models - jupyter notebook used to run simulations.

## Differential Game Theoretic Model Simulation
<p align="justify"> 
The agents are controlled using a feedback control approach. The control feedback gains are obtained using recursive equations derived from Chapter 6, Dynamic Non-Cooperative Game Theory, Basar and Olsder. In short, each agent does not gain anything by deviating from their optimal control policy.
</p>

Running the simulation yields the following solution trajectories.
<p align="center"> 
<img src="https://github.com/raklokesh/DifferentialGameTheory_SimpleImplementation/blob/main/UnsharedCosts_.png" width="400px">
</p>

<p align="justify"> 
It is also possible to share the control effort costs of each agent with another by designing the cost functions appropriately. The script has options to specify whether control costs are shared for each agent. When control costs are shared the agents move in a more cooeprative manner.
</p>

<p align="center"> 
  <img src="https://github.com/raklokesh/DifferentialGameTheory_SimpleImplementation/blob/main/SharedCosts_.png" width="400px">
</p>

<p align="justify"> 
The same movement behavior would be observed if a single entity with a single cost function controlled both agents. Such a behavior can be modelled using a single Linear Quadratic Regulator (See class 'TwoHands_Model'). Running the simulation of the single LQR yields the following trajectories.
</p>

<p align="center"> 
  <img src="https://github.com/raklokesh/DifferentialGameTheory_SimpleImplementation/blob/main/SingleLQR.png" width="400px">
</p>



 


