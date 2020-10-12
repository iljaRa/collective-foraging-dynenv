# collective-foraging-dynenv
A C++ simulation of the collective foraging behaviour with scale-free communication and dynamic changes of item avaialability using ARGoS.

Please see https://osf.io/48b9h/ and https://doi.org/10.3389/frobt.2020.00086 for more details.

If our projects sparks your interest in swarm robotic simulation, please visit the ARGoS webpage for more instructions on how to easily get started.
http://argos-sim.info/

Our simulation setup is strongly inspired by the foraging example provided here: http://argos-sim.info/examples.php

However, we significantly extended the scenario by scaling up the swarm to almost 1000 robots. Moreover, we included an algorithm that allows the swarm to grow a scale-free network based on proximity information at each simulation time step. Additionally, we implemented a time-varying availability of food items inside the foraging area.

If you wish to run these simulations on your computer to reproduce our results, or just for fun, you can use this repository or go to https://osf.io/48b9h/ where you find everything you need - including a short tutorial on how to install the simulator (e.g. using our Docker image) and get started with ARGoS.
