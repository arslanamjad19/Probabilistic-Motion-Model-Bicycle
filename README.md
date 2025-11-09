# Probabilistic Motion Model of Kinematic Bicycle

# 🚲 Bicycle Motion Simulation and State Estimation

This project involves deriving the deterministic motion model for an idealized bicycle and implementing a probabilistic sampling function to simulate its movement under noisy controls.

## 📝 Problem Overview

Consider an **idealized bicycle** with a **wheelbase $l$**. The bicycle's state is described by the state vector $\mathbf{x} = [x, y, \theta]^T$, where:
* $(x, y)$ is the coordinate of the front tire's center.
* $\theta$ is the frame's orientation (heading).

The bicycle is controlled by a **forward velocity $v$** and a **steering angle $\alpha$**. The rear wheel is assumed to move parallel to the frame (i.e., **no sideways slip**). We assume the controls are constant over a time interval $\Delta t$.

The control input vector is $\mathbf{u}_t = [v_t, \alpha_t]^T$.

## 🎯 Tasks

### a. Deterministic State Update Equations

We have derived the deterministic state update equations, $\mathbf{x}_{t+1} = g(\mathbf{x}_t, \mathbf{u}_t)$, which predict the new pose given the current pose $\mathbf{x}_t$ and controls $\mathbf{u}_t$.

* **General Case:** A solution is provided for the general case of turning ($\alpha \neq 0$).
* **Special Case:** A solution is provided for the special case of moving straight ($\alpha = 0$).

### b. Implementation of Sampling Function

Implemented a sampling function based on our derived model to simulate the bicycle's motion under **noisy controls**. The true controls, $\hat{v}$ and $\hat{\alpha}$, were sampled from a Gaussian distribution centered on the commanded values:

$$
\hat{v} = v + \epsilon_v, \quad \text{where } \epsilon_v \sim \mathcal{N}(0, \sigma_v^2)
$$

$$
\hat{\alpha} = \alpha + \epsilon_\alpha, \quad \text{where } \epsilon_\alpha \sim \mathcal{N}(0, \sigma_\alpha^2)
$$

## ⚙️ Simulation Parameters

Use the following parameters for implementation and simulation:

* **Wheelbase:** $l = 150 \text{ cm}$
* **Time step:** $\Delta t = 1 \text{ sec}$
* **Variance of steering angle:** $\sigma_\alpha^2 = 16 \text{ (degrees)}^2$ (i.e., standard deviation $\sigma_\alpha = 4^\circ$)
* **Variance of velocity:** $\sigma_v^2 = 42 \cdot v^2 \text{ in units of }(\text{cm}/\text{sec})^2$

For a bicycle starting at the origin, $x_0 = [0, 0, 0]^T$, we used the above function to generate at least $N = 240$ posterior pose samples for a single time step. Create a separate plot of the $(x, y)$ sample cloud for each of the control parameter sets listed in the table below.

| Problem Number | Steering Angle ($\alpha$) | Velocity ($v$) |
| :------------: | :-----------------------: | :--------------: |
| 1              | $35^\circ$                | $22 \text{ cm/sec}$ |
| 2              | $-35^\circ$               | $33 \text{ cm/sec}$ |
| 3              | $25^\circ$                | $85 \text{ cm/sec}$ |
| 4              | $62^\circ$                | $10 \text{ cm/sec}$ |
| 5              | $75^\circ$                | $94 \text{ cm/sec}$ |

---

## 📈 Task c. Uncertainty Propagation Simulation

Simulated the propagation of uncertainty over a sequence of motions using the sampling function implemented in Task **b**.

### Simulation Setup

1.  **Initialization:** We initialized $N = 240$ samples (particles) at the origin: $\mathbf{x}_0 = [0, 0, 0]^T$.
2.  **Procedure:** For each particle, we applied the following sequence of control commands, using our sampling function at each step to update the particle's pose. The total simulation duration is $t=8\text{s}$.

| Time Interval | Duration ($\Delta t$) | Steps | Motion Command $\mathbf{u}_t = [v, \alpha]^T$ |
| :-----------: | :--------------------: | :---: | :-------------------------------------------: |
| $t=1\text{s}$ to $3\text{s}$ | $2\text{s}$ | 3     | Drive straight with $\mathbf{u}_t = [55 \text{ cm/sec}, 0^\circ]^T$ |
| $t=4\text{s}$ to $5\text{s}$ | $2\text{s}$ | 2     | Turn right with $\mathbf{u}_t = [31 \text{ cm/sec}, -20^\circ]^T$ |
| $t=6\text{s}$ to $8\text{s}$ | $3\text{s}$ | 3     | Drive straight again with $\mathbf{u}_t = [65 \text{ cm/sec}, 0^\circ]^T$ |

### Visualization

Created a **single plot** that visualizes the entire simulation motion.

On this plot, the following elements have been shown:

1.  The **particle clouds** (the set of 240 samples) at the end of each motion segment (i.e., at $t=3\text{s}$, $t=5\text{s}$, and the final time $t=8\text{s}$).
2.  The **ideal** (noise-free) **trajectory** for comparison.

The plot should clearly illustrate how the uncertainty cloud **grows and deforms** as the bicycle moves.
