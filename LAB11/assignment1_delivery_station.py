#!/usr/bin/env python3
"""
CS359: Artificial Intelligence for Robotics - Lab Assignment 11
Assignment 1: Autonomous Robot Navigation to a Delivery Station
Real-world application: Warehouse goods delivery

Problem Statement:
Develop an APF-based simulation in which an autonomous robot moves from its starting
position to a specified goal position using an attractive potential field.
"""

import os
os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib_cache'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

class APFDeliveryRobot:
    def __init__(self, start=(10.0, 10.0), goal=(90.0, 90.0), radius=5.0,
                 k_att=1.2, v_max=2.0, dt=0.5, goal_threshold=1.0):
        self.start = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.radius = radius
        self.k_att = k_att
        self.v_max = v_max
        self.dt = dt
        self.goal_threshold = goal_threshold
        self.position = np.copy(self.start)
        self.trajectory = [np.copy(self.position)]
        self.distances_to_goal = [np.linalg.norm(self.position - self.goal)]
        self.time_steps = [0.0]

    def compute_attractive_force(self, pos):
        """
        Attractive Potential: U_att(q) = 0.5 * k_att * ||q - q_goal||^2
        Attractive Force:     F_att(q) = -grad(U_att) = k_att * (q_goal - pos)
        """
        diff = self.goal - pos
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            return np.zeros(2)
        # Conical attractive force when dist > d_switch for smooth constant velocity,
        # quadratic when close to prevent overshoot
        d_switch = 10.0
        if dist > d_switch:
            force = self.k_att * d_switch * (diff / dist)
        else:
            force = self.k_att * diff
        return force

    def step(self):
        f_att = self.compute_attractive_force(self.position)
        force_norm = np.linalg.norm(f_att)
        
        if force_norm > 1e-6:
            direction = f_att / force_norm
            speed = min(self.v_max, force_norm)
            velocity = speed * direction
        else:
            velocity = np.zeros(2)

        self.position = self.position + velocity * self.dt
        dist = np.linalg.norm(self.position - self.goal)
        self.trajectory.append(np.copy(self.position))
        self.distances_to_goal.append(dist)
        self.time_steps.append(self.time_steps[-1] + self.dt)

        finished = dist <= self.goal_threshold
        return finished

    def simulate(self, max_steps=1000):
        for step_idx in range(max_steps):
            if self.step():
                print(f"[Assignment 1] Goal reached successfully in {step_idx + 1} steps (t = {self.time_steps[-1]:.1f}s)!")
                break
        return np.array(self.trajectory), np.array(self.distances_to_goal), np.array(self.time_steps)

def plot_assignment_1(robot, save_path=None):
    traj = np.array(robot.trajectory)
    
    # 2D Field Grid for Potential Contours and Vector Quiver
    x = np.linspace(0, 100, 40)
    y = np.linspace(0, 100, 40)
    X, Y = np.meshgrid(x, y)
    
    # Calculate Potential and Gradient Forces
    U_att = 0.5 * robot.k_att * ((X - robot.goal[0])**2 + (Y - robot.goal[1])**2)
    Fx = robot.k_att * (robot.goal[0] - X)
    Fy = robot.k_att * (robot.goal[1] - Y)
    F_mag = np.hypot(Fx, Fy) + 1e-9
    Fx_norm = Fx / F_mag
    Fy_norm = Fy / F_mag

    fig = plt.figure(figsize=(14, 6))

    # Subplot 1: Trajectory and Potential Quiver Map
    ax1 = fig.add_subplot(1, 2, 1)
    contour = ax1.contourf(X, Y, U_att, levels=30, cmap='Blues_r', alpha=0.35)
    plt.colorbar(contour, ax=ax1, label='Attractive Potential $U_{att}(q)$')
    ax1.quiver(X[::2, ::2], Y[::2, ::2], Fx_norm[::2, ::2], Fy_norm[::2, ::2],
               color='gray', alpha=0.5, scale=30, width=0.003)

    # Plot Robot path
    ax1.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=2.5, label='Robot Trajectory')
    
    # Draw Start and Goal
    ax1.scatter(robot.start[0], robot.start[1], c='green', s=160, marker='o', edgecolors='black', zorder=5, label=f'Start Storage ({robot.start[0]:.0f}, {robot.start[1]:.0f})')
    ax1.scatter(robot.goal[0], robot.goal[1], c='gold', s=200, marker='*', edgecolors='black', zorder=5, label=f'Goal Delivery Station ({robot.goal[0]:.0f}, {robot.goal[1]:.0f})')

    # Draw Robot at intervals
    sample_indices = np.linspace(0, len(traj)-1, 6, dtype=int)
    for idx in sample_indices:
        pos = traj[idx]
        circle = Circle((pos[0], pos[1]), robot.radius, color='darkblue', fill=False, linestyle='--', linewidth=1.2, alpha=0.6)
        ax1.add_patch(circle)
        ax1.scatter(pos[0], pos[1], c='darkblue', s=30)

    # Final robot footprint
    final_circle = Circle((traj[-1, 0], traj[-1, 1]), robot.radius, color='darkblue', alpha=0.3)
    ax1.add_patch(final_circle)

    ax1.set_xlim(0, 100)
    ax1.set_ylim(0, 100)
    ax1.set_aspect('equal')
    ax1.set_xlabel('Warehouse X (m)', fontsize=11)
    ax1.set_ylabel('Warehouse Y (m)', fontsize=11)
    ax1.set_title('Assignment 1: Navigation to Delivery Station (APF)', fontsize=12, fontweight='bold')
    ax1.legend(loc='lower right', fontsize=9)
    ax1.grid(True, linestyle=':', alpha=0.6)

    # Subplot 2: Distance to Delivery Station vs Time
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(robot.time_steps, robot.distances_to_goal, 'r-', linewidth=2, label='Euclidean Distance to Goal')
    ax2.axhline(y=robot.goal_threshold, color='g', linestyle='--', label=f'Threshold ({robot.goal_threshold} m)')
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Distance to Goal (m)', fontsize=11)
    ax2.set_title('Distance to Delivery Station vs Simulation Time', fontsize=12, fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=10)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"[Assignment 1] Figure saved to {save_path}")
    plt.close()

if __name__ == "__main__":
    robot = APFDeliveryRobot(start=(10, 10), goal=(90, 90), radius=5.0)
    robot.simulate()
    os.makedirs('/home/sravanthi/GAZEBO/LAB11/plots', exist_ok=True)
    plot_assignment_1(robot, save_path='/home/sravanthi/GAZEBO/LAB11/plots/assignment1_result.png')

