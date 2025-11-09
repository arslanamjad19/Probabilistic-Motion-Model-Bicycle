# Probabilistic Motion Model of Kinematic Bicycle

Since the request asks for the text of a GitHub README file based on the image, I will format the problem statement as a project description, including the title, problem overview, and the specific tasks.

---

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

Derive the deterministic state update equations, $\mathbf{x}_{t+1} = g(\mathbf{x}_t, \mathbf{u}_t)$, which predict the new pose given the current pose $\mathbf{x}_t$ and controls $\mathbf{u}_t$.

* **General Case:** Provide a solution for the general case of turning ($\alpha \neq 0$).
* **Special Case:** Provide a solution for the special case of moving straight ($\alpha = 0$).

### b. Implementation of Sampling Function

Implement a sampling function based on your derived model to simulate the bicycle's motion under **noisy controls**. The true controls, $\hat{v}$ and $\hat{\alpha}$, are sampled from a Gaussian distribution centered on the commanded values:

$$
\hat{v} = v + \epsilon_v, \quad \text{where } \epsilon_v \sim \mathcal{N}(0, \sigma_v^2)
$$

$$
\hat{\alpha} = \alpha + \epsilon_\alpha, \quad \text{where } \epsilon_\alpha \sim \mathcal{N}(0, \sigma_\alpha^2)
$$

## ⚙️ Simulation Parameters

Use the following parameters for implementation and simulation:

* **Wheelbase:** $l = 100 \text{ cm}$
* **Time step:** $\Delta t = 1 \text{ sec}$
* **Variance of steering angle:** $\sigma_\alpha^2 = 25 \text{ (degrees)}^2$ (i.e., standard deviation $\sigma_\alpha = 5^\circ$)
* **Variance of velocity:** $\sigma_v^2 = 50 \cdot v^2 \text{ in units of }(\text{cm}/\text{sec})^2$

For a bicycle starting at the origin, x0 = [0, 0, 0]T, use your function to generate at least
N = 200 posterior pose samples for a single time step. Create a separate plot of the (x, y) sample cloud for each of the control parameter sets listed in the table below.

| Problem Number | Steering Angle ($\alpha$) | Velocity ($v$) |
| :------------: | :-----------------------: | :--------------: |
| 1              | $25^\circ$                | $20 \text{ cm/sec}$ |
| 2              | $-25^\circ$               | $20 \text{ cm/sec}$ |
| 3              | $25^\circ$                | $90 \text{ cm/sec}$ |
| 4              | $80^\circ$                | $10 \text{ cm/sec}$ |
| 5              | $85^\circ$                | $90 \text{ cm/sec}$ |

---

## 📈 Task c. Uncertainty Propagation Simulation

Simulate the propagation of uncertainty over a sequence of motions using the sampling function implemented in Task **b**.

### Simulation Setup

1.  **Initialization:** Initialize $N = 200$ samples (particles) at the origin: $\mathbf{x}_0 = [0, 0, 0]^T$.
2.  **Procedure:** For each particle, apply the following sequence of control commands, using your sampling function at each step to update the particle's pose. The total simulation duration is $t=8\text{s}$.

| Time Interval | Duration ($\Delta t$) | Steps | Motion Command $\mathbf{u}_t = [v, \alpha]^T$ |
| :-----------: | :--------------------: | :---: | :-------------------------------------------: |
| $t=1\text{s}$ to $3\text{s}$ | $2\text{s}$ | 3     | Drive straight with $\mathbf{u}_t = [50 \text{ cm/sec}, 0^\circ]^T$ |
| $t=4\text{s}$ to $5\text{s}$ | $2\text{s}$ | 2     | Turn right with $\mathbf{u}_t = [30 \text{ cm/sec}, -20^\circ]^T$ |
| $t=6\text{s}$ to $8\text{s}$ | $3\text{s}$ | 3     | Drive straight again with $\mathbf{u}_t = [50 \text{ cm/sec}, 0^\circ]^T$ |

*(Note: The total steps for the time intervals sum to $3+2+3=8$ steps. Assuming the simulation starts at $t=0$ and steps occur at $t=1, 2, \dots, 8$. The prompt suggests 3 steps from $t=1\text{s}$ to $3\text{s}$ and 3 steps from $t=6\text{s}$ to $8\text{s}$, which is slightly ambiguous but will be interpreted as a total of 8 timesteps with $\Delta t = 1\text{s}$.)*

### Visualization

Create a **single plot** that visualizes the entire simulation motion.

On this plot, the following elements must be shown:

1.  The **particle clouds** (the set of 200 samples) at the end of each motion segment (i.e., at $t=3\text{s}$, $t=5\text{s}$, and the final time $t=8\text{s}$).
2.  The **ideal** (noise-free) **trajectory** for comparison.

The plot should clearly illustrate how the uncertainty cloud **grows and deforms** as the bicycle moves.
