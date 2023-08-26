# Differential Game Theory Illustrative Implementation

Implementation of differential game theoretic solution for a simple spatial negotiation problem between two agents.

# Problem Statement
<p align="justify"> 
Two agents have to move from a start position to end position in two dimensional space (Lateral (x) and Forward (y)). Each agent has to pass through a via-point before reaching the end target. The agents have to reach the end target and pass through the via-point at specified time instants.
</p>
<p align="center"> 
<img src="https://github.com/raklokesh/DifferentialGameTheory_SimpleImplementation/blob/main/Task%20Illustration.jpg" width="400px">
</p>

<p align="justify"> 
Each agent has its own cost function that aims to minimize the control effort, distance from its via-point at the specified time instant, and distance from the end target at the specified time instant. However, the cost functions of the two agents will be tied to one another when the agents are connected through a spring between them. The agents have to negotiate movement paths to minimize control effort.
</p>



