# PX4_SIH_QuadX
Simulator in Hardware for PX4 Quadrotor X by Coriolis g Coporation

The Simulator in Hardware (SIH) is an alternative to the [Hardware In The Loop (HIL) simulation](https://dev.px4.io/en/simulation/hitl.html) provided by PX4. In this setup, the controller, the state estimator, and the simulator are running on the embedded hardware. Basically, everything is running on the embedded hardware.

The SIH provides two benefits:
- It ensures real time simulation by avoiding the bidirectionnal connection to the computer. It means the user does not have to worry about having an expensive/real time desktop computer.
- It provides the code in C++ to developers and researchers willing to incorporate their own aerodynamics into the simulator.

The SIH can be used by new PX4 users to get familiar with PX4 and the different modes and features, and of course to learn to fly a quadrotor.

## About us
Coriolis g Corporation is a Canadian company specialized in Vertical Takeoff and Landing (VTOL) Unmanned Aerial Vehicles (UAV). 

The company is focusing on devloping a new type of VTOL aircraft using a passive coupling system.
We are specialized in dynamics and control and real time simulation.

Discover our current platform at www.vogi-vtol.


