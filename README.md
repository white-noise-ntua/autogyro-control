# :rocket: autorotation - Control Systems

Autorotation Task for CanSat Competition. Control Systems Repository.
This repository contains the source code and files needed for the **CanSat Challenge** by the **White Noise Team**. 

# Contents 

## :robot: Control Systems

The dynamics of the system are modeled as a rigid body falling along the longitudinal axis. The states are the angles made with the x and y axis which can also be measured by the IMU. The inputs to the system are the torques Mx, My along the x and y axes. Near its equilibrium point the system is linearized and linear control methods are applied. In our approach we have used linear quadratic regulation control  of the form u[n] = -  Kx[n] to send the states to zero. 

The **control mechanism** of the payload are three control fins that are attached to the payload. We have developed a theoretical model for the fins based on aerodynamics, which we used as a simulation black box. We have sampled the resulting moments for various fins configurations in order to invert the function that gives the moments wrt to the fins. Because the data we have gathered were almost impossible to fit to the microcontroller we have used **K-Means Clustering** in order to compress the points to a very small subset (cluster centers which range from 20 to 50 points) and then fit a **hyperplane** on the data. The resulting angles are calculated by a simplified linear model developed with **Pytorch**. 

# Authors

 * George Rapakoulias 
 * Marios Papachristou
