## Distributed Collaboration of Connected Autonomous Vehicles at Unsignalized Intersections using Parallel Monte Carlo Tree Search

### MCS Dissertation Project

This project uses the [Simulated Urban Mobility (SUMO)](https://sumo.dlr.de/docs/) package to simulate Connected Autonomous Vehicle traversal of Unsignalized Intersections.
On approach to the intersection, the vehicles communicate data to each other pertaining to their arrival time to the intersection and their desired trajectory through it.
The vehicles then use Root Parallel Monte Carlo Tree Search to find the optimal passing order.
If two vehicles are on a collision trajectory, the one which is lower in the passing order will slow down to give way.
The optimal passing order is that in which the least amount of adjustments need to be made by vehicles to avoid collisions, thereby minimising traffic delay at the intersection while avoiding collisions.

#### [Link to dissertation](https://www.scss.tcd.ie/publications/theses/diss/2021/TCD-SCSS-DISSERTATION-2021-075.pdf)

![demo](img/demo.gif)
