# PX4_SIH_QuadX
Simulator in Hardware for PX4 Quadrotor X by Coriolis g Coporation

![Simulator in Hardware diagram](https://github.com/romain-chiap/PX4_SIH_QuadX/blob/master/Documentation/SIH_diagram.png)

The Simulator in Hardware (SIH) is an alternative to the [Hardware In The Loop (HIL) simulation](https://dev.px4.io/en/simulation/hitl.html) provided by PX4. In this setup, the controller, the state estimator, and the simulator are running on the embedded hardware. Basically, everything is running on the embedded hardware.

The SIH provides two benefits over the HITL:
- It ensures real time simulation by avoiding the bidirectionnal connection to the computer. It means the user does not have to worry about having an expensive/real time desktop computer.
- It provides the code in C++ to developers and researchers willing to incorporate their own aerodynamics into the simulator. They can for instance modify the noise level of the sensors or even add a sensor to be simulated.

The SIH can be used by new PX4 users to get familiar with PX4 and the different modes and features, and of course to learn to fly a quadrotor with the real Radio.

## User manual and documentation
For information on seting up the Coriolis SIH, visit the [wiki page](https://github.com/romain-chiap/PX4_SIH_QuadX/wiki)
For additional documentation, check the [Documentation](https://github.com/romain-chiap/PX4_SIH_QuadX/tree/master/Documentation) folder.

## About us
Coriolis g Corporation is a Canadian company specialized in Vertical Takeoff and Landing (VTOL) Unmanned Aerial Vehicles (UAV). 

The company is focusing on devloping a new type of VTOL aircraft using a passive coupling system.
We are specialized in dynamics and control and real time simulation. The SIH is a simple simulator for quadrotors realeased for free and without any conditions.

Discover our current platform at www.vogi-vtol.com


