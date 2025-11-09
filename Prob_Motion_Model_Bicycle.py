import numpy as np
import matplotlib.pyplot as plt

def bicycle_model(x, y, theta, v, alpha, l, dt):
    """
    Deterministic kinematic bicycle model.
    Input Parameters:
    - x, y: current position (cm)
    - theta: current orientation (radians)
    - v: forward velocity (cm/sec)
    - alpha: steering angle (radians)
    - l: wheelbase (cm)
    - dt: time step (sec)
    Returns:
    - x_new, y_new, theta_new: updated state
    """
    alpha_threshold = 1e-6                    # FOr α  = 0
    if abs(alpha) < alpha_threshold:
        # Special case: moving straight (α = 0)
        x_new = x + v * dt * np.cos(theta)
        y_new = y + v * dt * np.sin(theta)
        theta_new = theta
    else:
        # General case: turning (α != 0)
        R = l / np.tan(alpha)                  # Turning radius
        beta = v * np.tan(alpha) * dt / l      # Angular displacement
        # State updates
        x_new = x + R * (np.sin(theta + beta) - np.sin(theta))
        y_new = y + R * (np.cos(theta) - np.cos(theta + beta))
        theta_new = theta + beta
    
    return x_new, y_new, theta_new


def bicycle_motion(x0, y0, theta0, v_cmd, alpha_cmd, l, dt, sigma_v_factor, sigma_alpha_deg, N):
    """
    Sample N posterior poses after one time step with noisy controls
    Parameters:
    - x0, y0, theta0: initial state
    - v_cmd: commanded velocity (cm/sec)
    - alpha_cmd: commanded steering angle (degrees)
    - l: wheelbase (cm)
    - dt: time step (sec)
    - sigma_v_factor: velocity noise factor (σ_v² = sigma_v_factor * v²)
    - sigma_alpha_deg: steering angle standard deviation (degrees)
    - N: number of samples
    Returns:
    - samples: Nx3 array of [x, y, theta] samples
    """
    alpha_cmd_rad = np.deg2rad(alpha_cmd)
    sigma_v = np.sqrt(sigma_v_factor) * abs(v_cmd)           # Calculate standard deviations
    sigma_alpha_rad = np.deg2rad(sigma_alpha_deg)
    v_samples = np.random.normal(v_cmd, sigma_v, N)          # Sample noisy controls
    alpha_samples = np.random.normal(alpha_cmd_rad, sigma_alpha_rad, N)
    samples = np.zeros((N, 3))                               # Initialize sample array
    # Generate samples
    for i in range(N):
        x_new, y_new, theta_new = bicycle_model(
            x0, y0, theta0, 
            v_samples[i], alpha_samples[i], 
            l, dt)
        samples[i] = [x_new, y_new, theta_new]
    return samples

def propagate_particles(particles, v_cmd, alpha_cmd, l, dt, sigma_v_factor, sigma_alpha_deg, num_steps):
    """
    Inputs:
    - particles: Nx3 array of current particle states
    - v_cmd, alpha_cmd: commanded controls
    - num_steps: number of time steps to propagate
    Returns:
    - particles: updated Nx3 array
    """
    N = particles.shape[0]
    
    for step in range(num_steps):
        for i in range(N):
            x, y, theta = particles[i]
            x_new, y_new, theta_new = bicycle_motion(x, y, theta, v_cmd, alpha_cmd, l, dt, sigma_v_factor, sigma_alpha_deg, 1)[0]
            particles[i] = [x_new, y_new, theta_new]
    
    return particles


def compute_ideal_trajectory(x0, y0, theta0, control_sequence, l, dt):
    """
    Input parameters:
    - x0, y0, theta0: initial state
    - control_sequence: list of (v, alpha_deg, num_steps) tuples
    Returns:
    - trajectory: list of (x, y, theta) states
    """
    trajectory = [(x0, y0, theta0)]
    x, y, theta = x0, y0, theta0
    
    for v_cmd, alpha_cmd_deg, num_steps in control_sequence:
        alpha_cmd_rad = np.deg2rad(alpha_cmd_deg)
        
        for step in range(num_steps):
            x, y, theta = bicycle_model(x, y, theta, v_cmd, alpha_cmd_rad, l, dt)
            trajectory.append((x, y, theta))
    
    return trajectory

# Simulation parameters
l = 100                           # wheelbase (cm)
dt = 1                            # time step (sec)
sigma_v_factor = 42               # variance factor for velocity
sigma_alpha_deg = 4               # standard deviation of steering angle (degrees)
N = 240                           # number of samples
x0, y0, theta0 = 0, 0, 0          # Initial state

# Control parameter sets from the table
problems = [
    {"num": 1, "alpha": 35, "v": 22},
    {"num": 2, "alpha": -35, "v": 33},
    {"num": 3, "alpha": 25, "v": 85},
    {"num": 4, "alpha": 62, "v": 10},
    {"num": 5, "alpha": 75, "v": 94},
]

# Create subplots
fig, axes = plt.subplots(2, 3, figsize=(15, 10))
axes = axes.flatten()

for idx, problem in enumerate(problems):
    alpha_cmd = problem["alpha"]
    v_cmd = problem["v"]
    
    # Generate samples
    samples = bicycle_motion(
        x0, y0, theta0, 
        v_cmd, alpha_cmd, 
        l, dt, 
        sigma_v_factor, sigma_alpha_deg, 
        N
    )
    # Extract x and y coordinates
    x_samples = samples[:, 0]
    y_samples = samples[:, 1]
    ax = axes[idx]
    ax.scatter(x_samples, y_samples, alpha=0.5, s=20, color='blue')
    ax.scatter(x0, y0, color='red', s=100, marker='o', label='Start', zorder=5)
    ax.set_xlabel('x (cm)', fontsize=11)
    ax.set_ylabel('y (cm)', fontsize=11)
    ax.set_title(f'Problem {problem["num"]}: α = {alpha_cmd}°, v = {v_cmd} cm/s', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend()

# Print statistics for each problem
print("=" * 70)
print("Bicycle Motion Simulation Results")
print("=" * 70)
print(f"Parameters: l = {l} cm, Δt = {dt} sec, N = {N} samples")
print(f"Noise: σ_α = {sigma_alpha_deg} deg, σ_v^2 = {sigma_v_factor}·v^2")
print("=" * 70)

for idx, problem in enumerate(problems):
    alpha_cmd = problem["alpha"]
    v_cmd = problem["v"]
    
    samples = bicycle_motion(x0, y0, theta0, v_cmd, alpha_cmd, l, dt, sigma_v_factor, sigma_alpha_deg, N)
    x_mean = np.mean(samples[:, 0])
    y_mean = np.mean(samples[:, 1])
    x_std = np.std(samples[:, 0])
    y_std = np.std(samples[:, 1])
    
    print(f"\nProblem {problem['num']}: α = {alpha_cmd} deg, v = {v_cmd} cm/s")
    print(f"  Mean position: ({x_mean:.2f}, {y_mean:.2f}) cm")
    print(f"  Std deviation: (±{x_std:.2f}, ±{y_std:.2f}) cm")

# Part C 
# Control sequence: (velocity, steering_angle, num_steps)
control_sequence = [
    (55, 0, 3),      # t=1s to 3s: drive straight
    (31, -20, 2),    # t=4s to 5s: turn right
    (65, 0, 3),      # t=6s to 8s: drive straight
]
# Initialize particles at origin
particles = np.zeros((N, 3))
particles[:, 0] = x0
particles[:, 1] = y0
particles[:, 2] = theta0

# Storage for particle clouds at key times
clouds = {}
clouds[0] = particles.copy()

# Compute ideal trajectory
ideal_trajectory = compute_ideal_trajectory(x0, y0, theta0, control_sequence, l, dt)

# Propagate particles through the sequence
time = 0
for v_cmd, alpha_cmd, num_steps in control_sequence:
    particles = propagate_particles(
        particles, v_cmd, alpha_cmd, l, dt,
        sigma_v_factor, sigma_alpha_deg, num_steps
    )
    time += num_steps
    clouds[time] = particles.copy()

# Create visualization
fig, ax = plt.subplots(figsize=(12, 9))

# Plot ideal trajectory
ideal_x = [state[0] for state in ideal_trajectory]
ideal_y = [state[1] for state in ideal_trajectory]
ax.plot(ideal_x, ideal_y, 'r-', linewidth=2.5, label='Ideal Trajectory', zorder=5)

# Mark waypoints on ideal trajectory
waypoint_times = [0, 3, 5, 8]
waypoint_indices = [0, 3, 5, 8]
for i, t in enumerate(waypoint_times):
    idx = waypoint_indices[i]
    ax.plot(ideal_x[idx], ideal_y[idx], 'ro', markersize=10, 
            markeredgewidth=2, markeredgecolor='darkred', zorder=6)

# Plot particle clouds at key times
colors = ['blue', 'green', 'purple', 'orange']
labels = ['t = 0 s (Start)', 't = 3 s (After straight)', 
        't = 5 s (After turn)', 't = 8 s (Final)']
sizes = [100, 50, 50, 50]
alphas = [0.8, 0.5, 0.4, 0.4]

for i, (t, label, color, size, alpha) in enumerate(zip(waypoint_times, labels, colors, sizes, alphas)):
    cloud = clouds[t]
    ax.scatter(cloud[:, 0], cloud[:, 1], 
            s=size, alpha=alpha, c=color, 
            label=label, edgecolors='black', linewidths=0.5, zorder=4-i)

# Add arrows to show trajectory direction at key points
arrow_indices = [1, 3, 6]
for idx in arrow_indices:
    if idx < len(ideal_trajectory) - 1:
        x1, y1, theta = ideal_trajectory[idx]
        dx = 15 * np.cos(theta)
        dy = 15 * np.sin(theta)
        ax.arrow(x1, y1, dx, dy, head_width=8, head_length=6, 
                fc='red', ec='darkred', linewidth=1.5, zorder=7)

# Formatting
ax.set_xlabel('x (cm)', fontsize=12, fontweight='bold')
ax.set_ylabel('y (cm)', fontsize=12, fontweight='bold')
ax.set_title('Bicycle Motion: Uncertainty Propagation Over Time\n' + f'N={N} particles, l={l} cm, σ_α={sigma_alpha_deg}°, σ_v^2={sigma_v_factor}·v^2',fontsize=14, fontweight='bold', pad=15)
ax.grid(True, alpha=0.3, linestyle='--')
ax.axis('equal')
ax.legend(loc='upper left', fontsize=11, framealpha=0.95)
plt.tight_layout()
plt.savefig('bicycle_uncertainty_propagation.png', dpi=150, bbox_inches='tight')
plt.show()

# Print statistics
print("=" * 80)
print("Bicycle Uncertainty Propagation Results")
print("=" * 80)
print(f"Parameters: N = {N} particles, l = {l} cm, Δt = {dt} sec")
print(f"Noise: σ_α = {sigma_alpha_deg} deg, σ_v^2 = {sigma_v_factor}·v^2")
print("=" * 80)

for i, t in enumerate(waypoint_times):
    cloud = clouds[t]
    x_mean = np.mean(cloud[:, 0])
    y_mean = np.mean(cloud[:, 1])
    x_std = np.std(cloud[:, 0])
    y_std = np.std(cloud[:, 1])
    ideal_x, ideal_y, ideal_theta = ideal_trajectory[waypoint_indices[i]]
    print(f"\nt = {t} s ({labels[i]}):")
    print(f"  Ideal position:    ({ideal_x:.2f}, {ideal_y:.2f}) cm")
    print(f"  Mean position:     ({x_mean:.2f}, {y_mean:.2f}) cm")
    print(f"  Std deviation:     (±{x_std:.2f}, ±{y_std:.2f}) cm")
    # Compute spread (max distance from mean)
    distances = np.sqrt((cloud[:, 0] - x_mean)**2 + (cloud[:, 1] - y_mean)**2)
    max_spread = np.max(distances)
    print(f"  Max spread:        {max_spread:.2f} cm")
