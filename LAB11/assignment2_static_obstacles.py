#!/usr/bin/env python3
"""
CS359: Artificial Intelligence for Robotics - Lab Assignment 11
Assignment 2: Robot Navigation in a Warehouse with Static Obstacles
Real-world application: Collision-free navigation between storage racks

Problem Statement:
Simulate Robot A using APF in an environment containing two static rectangular obstacles.
"""

import os
os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib_cache'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

class RectangularObstacle:
    def __init__(self, x_min, x_max, y_min, y_max, name="Rack"):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.name = name

    def closest_point(self, pos):
        cx = np.clip(pos[0], self.x_min, self.x_max)
        cy = np.clip(pos[1], self.y_min, self.y_max)
        return np.array([cx, cy])

    def distance_and_vector(self, pos):
        cp = self.closest_point(pos)
        diff = pos - cp
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            # If inside the obstacle, push toward nearest edge
            dx_min = abs(pos[0] - self.x_min)
            dx_max = abs(pos[0] - self.x_max)
            dy_min = abs(pos[1] - self.y_min)
            dy_max = abs(pos[1] - self.y_max)
            min_d = min(dx_min, dx_max, dy_min, dy_max)
            if min_d == dx_min:
                diff = np.array([-1.0, 0.0])
            elif min_d == dx_max:
                diff = np.array([1.0, 0.0])
            elif min_d == dy_min:
                diff = np.array([0.0, -1.0])
            else:
                diff = np.array([0.0, 1.0])
            dist = 0.1
        return dist, diff / np.linalg.norm(diff), cp

class APFStaticObstaclesNav:
    def __init__(self, start=(10.0, 10.0), goal=(90.0, 90.0), radius=5.0,
                 k_att=1.0, k_rep=250.0, rho_0=16.0, v_max=1.8, dt=0.4):
        self.start = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.radius = radius
        self.k_att = k_att
        self.k_rep = k_rep
        self.rho_0 = rho_0
        self.v_max = v_max
        self.dt = dt
        self.position = np.copy(self.start)

        # Two static rectangular storage racks placed along the path
        self.obstacles = [
            RectangularObstacle(24.0, 44.0, 26.0, 40.0, name="Storage Rack 1"),
            RectangularObstacle(56.0, 76.0, 58.0, 72.0, name="Storage Rack 2")
        ]

        self.trajectory = [np.copy(self.position)]
        self.min_distances_to_obs = []
        self.time_steps = [0.0]

    def compute_attractive_force(self, pos):
        diff = self.goal - pos
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            return np.zeros(2)
        d_switch = 12.0
        if dist > d_switch:
            return self.k_att * d_switch * (diff / dist)
        else:
            return self.k_att * diff

    def compute_repulsive_force(self, pos):
        f_rep_total = np.zeros(2)
        min_dist = float('inf')

        for obs in self.obstacles:
            dist, unit_vec, _ = obs.distance_and_vector(pos)
            effective_dist = max(0.5, dist - self.radius)
            if effective_dist < min_dist:
                min_dist = effective_dist

            if effective_dist < self.rho_0:
                # Standard Khatib repulsive potential gradient
                # F_rep = k_rep * (1/d - 1/rho_0) * (1 / d^2) * unit_vector
                rep_mag = self.k_rep * (1.0 / effective_dist - 1.0 / self.rho_0) * (1.0 / (effective_dist ** 2))
                f_rep = rep_mag * unit_vec
                
                # Add slight tangential circulation to avoid collinear local minimum saddle
                tangential_vec = np.array([-unit_vec[1], unit_vec[0]])
                f_rep += 0.25 * rep_mag * tangential_vec
                
                f_rep_total += f_rep

        return f_rep_total, min_dist

    def step(self):
        f_att = self.compute_attractive_force(self.position)
        f_rep, min_obs_dist = self.compute_repulsive_force(self.position)
        f_total = f_att + f_rep

        f_norm = np.linalg.norm(f_total)
        if f_norm > 1e-6:
            speed = min(self.v_max, f_norm)
            velocity = speed * (f_total / f_norm)
        else:
            velocity = np.zeros(2)

        self.position = self.position + velocity * self.dt
        self.trajectory.append(np.copy(self.position))
        self.min_distances_to_obs.append(min_obs_dist)
        self.time_steps.append(self.time_steps[-1] + self.dt)

        finished = np.linalg.norm(self.position - self.goal) <= 1.2
        return finished

    def simulate(self, max_steps=1500):
        for step_idx in range(max_steps):
            if self.step():
                print(f"[Assignment 2] Goal reached safely in {step_idx + 1} steps (t = {self.time_steps[-1]:.1f}s)!")
                break
        return np.array(self.trajectory)

def plot_assignment_2(robot, save_path=None):
    traj = np.array(robot.trajectory)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # Grid for vector field visualization
    xg = np.linspace(0, 100, 35)
    yg = np.linspace(0, 100, 35)
    X, Y = np.meshgrid(xg, yg)
    Fx = np.zeros_like(X)
    Fy = np.zeros_like(Y)

    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            pt = np.array([X[i, j], Y[i, j]])
            fa = robot.compute_attractive_force(pt)
            fr, _ = robot.compute_repulsive_force(pt)
            ft = fa + fr
            m = np.hypot(ft[0], ft[1]) + 1e-9
            Fx[i, j] = ft[0] / m
            Fy[i, j] = ft[1] / m

    ax1.quiver(X, Y, Fx, Fy, color='lightgray', alpha=0.6, width=0.0025, scale=40)

    # Draw Obstacles (Warehouse Racks)
    colors = ['#d9534f', '#f0ad4e']
    for idx, obs in enumerate(robot.obstacles):
        rect = Rectangle((obs.x_min, obs.y_min), obs.x_max - obs.x_min, obs.y_max - obs.y_min,
                         facecolor=colors[idx], edgecolor='black', linewidth=1.5, hatch='//', alpha=0.7)
        ax1.add_patch(rect)
        ax1.text((obs.x_min + obs.x_max)/2, (obs.y_min + obs.y_max)/2, obs.name,
                 color='black', weight='bold', ha='center', va='center', fontsize=9,
                 bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.85))

    # Plot Robot path
    ax1.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=2.5, label='APF Robot Path')
    
    # Draw Start and Goal
    ax1.scatter(robot.start[0], robot.start[1], c='green', s=160, marker='o', edgecolors='black', zorder=6, label='Start [10, 10]')
    ax1.scatter(robot.goal[0], robot.goal[1], c='gold', s=200, marker='*', edgecolors='black', zorder=6, label='Goal [90, 90]')

    # Robot footprint circles along trajectory
    sample_indices = np.linspace(0, len(traj)-1, 8, dtype=int)
    for idx in sample_indices:
        pos = traj[idx]
        circ = Circle((pos[0], pos[1]), robot.radius, color='darkblue', fill=False, linestyle=':', linewidth=1.2, alpha=0.7)
        ax1.add_patch(circ)
        ax1.scatter(pos[0], pos[1], c='darkblue', s=25)

    ax1.set_xlim(0, 100)
    ax1.set_ylim(0, 100)
    ax1.set_aspect('equal')
    ax1.set_xlabel('Warehouse X (m)', fontsize=11)
    ax1.set_ylabel('Warehouse Y (m)', fontsize=11)
    ax1.set_title('Assignment 2: Navigation with 2 Static Storage Racks', fontsize=12, fontweight='bold')
    ax1.legend(loc='lower right', fontsize=9)
    ax1.grid(True, linestyle=':', alpha=0.6)

    # Subplot 2: Minimum Distance to Storage Racks vs Time
    times = robot.time_steps[1:]
    min_dists = robot.min_distances_to_obs
    ax2.plot(times, min_dists, 'm-', linewidth=2, label='Clearance to Nearest Rack')
    ax2.axhline(y=0.0, color='red', linestyle='--', linewidth=1.5, label='Collision Boundary (0 m)')
    ax2.axhline(y=robot.rho_0, color='orange', linestyle=':', linewidth=1.5, label=f'Influence Zone (\\rho_0 = {robot.rho_0} m)')
    ax2.set_xlabel('Simulation Time (s)', fontsize=11)
    ax2.set_ylabel('Clearance Distance (m)', fontsize=11)
    ax2.set_title('Robot Clearance to Static Obstacles vs Time', fontsize=12, fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='lower right', fontsize=10)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"[Assignment 2] Figure saved to {save_path}")
    plt.close()

if __name__ == "__main__":
    robot = APFStaticObstaclesNav()
    robot.simulate()
    os.makedirs('/home/sravanthi/GAZEBO/LAB11/plots', exist_ok=True)
    plot_assignment_2(robot, save_path='/home/sravanthi/GAZEBO/LAB11/plots/assignment2_result.png')

