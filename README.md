# Multiagent_VO
Decentralised Multiagent path planning using velocity obstacles

The growing use of Multi-Agent Path Planning (MAPP) in various fields marks a significant advancement in the realm of autonomous systems and artificial intelligence. MAPP refers to the computational process of determining optimal paths for multiple agents, such as robots, vehicles, or drones, to navigate through a shared environment while avoiding collisions and efficiently reaching their respective goals. The surge in interest and application of MAPP is attributed to its capability to address complex scenarios where traditional path planning methods fall short. As industries increasingly integrate autonomous systems into their operations, MAPP emerges as a pivotal technology, offering solutions for dynamic environments, cooperative tasks, and real-world challenges. From logistics and transportation to surveillance and search and rescue missions, MAPP's versatility underscores its transformative impact, ushering in a new era of intelligent and collaborative agent-based systems.

This project aims to develop a decentralised method for multiagent systems by using the concept of velocity obstacles. A decentralised way suggests that the vehicle communicates with each other directly and finds paths to avoid collision by negotiation based on their prioritites to cross. Instead of avoiding the obstalce like in a traditional velocity obstacle method, the robot decides whether to stop or slow down while approaching near each other. This method reduces the latency time and any dependency on a central server. 

This method works efficiently with any number of vehicles as long as the collision is not between more than 5 vehicles crossing at the same time from a junction.

The messaging format for data communication betwen any local vehicle includes:
1. Current position of the vehicle
2. Current velocity of the vehicle
3. Current direction(yaw) of the vehicle
4. Assigned Priority value.

The algorithm running on each vehicle takes in these values and finds a new velocity of the vehicle needed to solve this scenario.

## Media
![video](https://github.com/saksham18kukreja/Multiagent_VO/blob/main/media/fms.gif)

The blue vehicle has the highest priority, followed by red and black.

## Future Work
1. Develop a high level planner which assigns the priority autonomously based on the goal locations of the vehicles.
2. Develop a learning based approach to assign priority to generalize over every scenario of collision and deadlock.
