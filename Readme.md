# Design and Implementation of a Digital Controller for a Self-Balancing Car Using LQR

## 1. Introduction

The objective of this project is to design and implement a digital controller for a self-balancing car using state-space control techniques. Specifically, we focus on the **Linear Quadratic Regulator (LQR)** for achieving stability. The report details the mathematical modeling, system identification, control design, simulation, and experimental validation using an STM32-based self-balancing car.

## 2. System Modeling

### 2.1 Equations of Motion
The dynamics of the self-balancing car can be derived using Newton-Euler equations. The primary equations governing the system are:

$$\dot{x} = Ax + Bu$$

$$y = Cx + Du$$

where:
- $A$ represents the system dynamics matrix,
- $B$ is the input matrix,
- $C$ is the output matrix,
- $D$ is the feedthrough matrix.

The state-space model is derived based on the system parameters:

$$A = \begin{bmatrix} 
0 & 1 & 0 & 0 & 0 & 0 \\ 
0 & 0 & A_{23} & 0 & 0 & 0 \\ 
0 & 0 & 0 & 1 & 0 & 0 \\ 
0 & 0 & A_{43} & 0 & 0 & 0 \\ 
0 & 0 & 0 & 0 & 0 & 1 \\ 
0 & 0 & 0 & 0 & 0 & 0 
\end{bmatrix}$$

$$B = \begin{bmatrix} 
0 & 0 \\ 
B_{21} & B_{22} \\ 
0 & 0 \\ 
B_{41} & B_{42} \\ 
0 & 0 \\ 
B_{61} & B_{62} 
\end{bmatrix}$$

where the values of $A_{23}$, $A_{43}$, $B_{21}$, $B_{22}$, $B_{41}$, $B_{42}$, $B_{61}$, $B_{62}$ are computed based on system parameters.

### 2.2 Parameter Identification

The physical parameters of the self-balancing car are:
- Wheel mass: $m = 0.035$ kg
- Wheel radius: $r = 0.0672/2$ m
- Vehicle body mass: $M = 1.000-2m$
- Distance from center of mass to chassis: $L = 0.5 \times 0.0766$
- Moment of inertia: $J_{centroid}$

Using these values, we computed the **state-space matrices A and B** in MATLAB.

## 3. LQR Controller Design

The LQR controller minimizes the cost function:

$$J = \int (x^TQx + u^TRu) dt$$

where $Q$ and $R$ are weighting matrices. The MATLAB script computes the **optimal gain matrix K**:

```matlab
Q = [7700 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 1600 0 0; 0 0 0 0 500 0; 0 0 0 0 0 0];
R = [1 0; 0 1];
K = lqr(A,B,Q,R);
```

The computed **gain matrix K** is:

$$K = \begin{bmatrix} 
-62.0484 & -73.3232 & -361.4617 & -35.9024 & 15.8114 & 3.9517 \\ 
-62.0484 & -73.3232 & -361.4617 & -35.9024 & -15.8114 & -3.9517 
\end{bmatrix}$$

## 4. Simulation and Analysis

### 4.1 Simulink Model
A Simulink model was developed for simulation, incorporating:
- **State-space representation** of the system
- **LQR gain implementation** as feedback control
- **Disturbance rejection analysis**

### 4.2 Results
The simulation results show:
- **Stable balancing** of the car
- **Effective disturbance rejection**
- **Smoother response compared to PID control**

A screenshot of the Simulink simulation output is attached.

## 5. Experimental Implementation

The computed LQR controller was implemented on the STM32-based car. The control algorithm was embedded using **Keil uVision & STM32CubeIDE**. Key observations:
- **Real-time angle estimation** using Kalman Filtering
- **PWM-based motor control** for dynamic balance
- **Successful real-world validation** of the LQR controller

## 6. Conclusion

This project successfully designed and implemented an LQR-based **digital controller** for a self-balancing car. The results demonstrate **improved stability** and **disturbance rejection** compared to other control methods. Future work includes:
- **Fine-tuning Kalman Filter for better sensor fusion**
- **Extending control for obstacle avoidance**
- **Deploying advanced optimization techniques (e.g., MPC)**
