# :rocket: autorotation
Autorotation Task for CanSat Competition.

This repository contains the source code and files needed for the **CanSat Challenge** by the **White Noise Team**. 



# Team

TBA



# Contents 

## :robot: Control Systems

The `control-systems` directory contains the control simulations for controlling the hub. The dynamics of the system are modeled as a rigid body falling along the longitudinal axis. The states are the angles made with the x and y axis which can also be measured by the IMU. The inputs to the system are the torques Mx, My along the x and y axes. Near its equilibrium point the system is linearized and linear control methods are applied. In our approach we have used linear quadratic regulation control  of the form u[n] = -  Kx[n] to send the states to zero. 