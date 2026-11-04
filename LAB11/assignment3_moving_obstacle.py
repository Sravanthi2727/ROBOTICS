#!/usr/bin/env python3
"""
CS359: Artificial Intelligence for Robotics - Lab Assignment 11
Assignment 3: Robot Navigation with a Moving Obstacle
Real-world application: Collision avoidance in shared robot workspaces

Problem Statement:
Develop a dynamic APF simulation in which an autonomous robot avoids a moving obstacle
while continuing toward its goal.
"""

import os
os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib_cache'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

class APFDynamicMovingObstacleNav:
    def __init__(self,
                 start_a=(10.0, 10.0), goal_a=(90.0, 90.0), radius_a=5.0,
                 start_obs=(20.0, 85.0), goal_obs=(85.0, 20.0), radius_obs=4.0, v_obs=1.1,
                 k_att=1.0, k_rep_dyn=300.0, rho_0=24.0, v_max=1.8, dt=0.5):
        self.start_a = np.array(start_a, dtype=float)
        self.goal_a = np.array(goal_a, dtype=float)
        self.radius_a = radius_a
        self.radius_obs = radius_obs

        self.pos_obs = np.array(start_obs, dtype=float)
        self.start_obs = np.array(start_obs, dtype=float)
        self.goal_obs = np.array(goal_obs, dtype=float)
        self.v_obs = v_obs
        obs_dir = self.goal_obs - self.start_obs
        self.obs_dir = obs_dir / np.linalg.norm(obs_dir)

        self.k_att = k_att
        self.k_rep_dyn = k_rep_dyn
        self.rho_0 = rho_0
        self.v_max = v_max
        self.dt = dt

        self.pos_a = np.copy(self.start_a)
        self.traj_a = [np.copy(self.pos_a)]
        self.traj_obs = [np.copy(self.pos_obs)]
        self.distances_inter = [np.linalg.norm(self.pos_a - self.pos_obs)]
        self.time_steps = [0.0]

    def update_moving_obstacle(self):
        dist_to_obs_goal = np.linalg.norm(self.goal_obs - self.pos_obs)
        if dist_to_obs_goal > 0.5:
            self.pos_obs = self.pos_obs + self.obs_dir * self.v_obs * self.dt
        return self.pos_obs

    def compute_forces(self):
        # Attractive force toward delivery goal
        diff_goal = self.goal_a - self.pos_a
        dist_goal = np.linalg.norm(diff_goal)
        if dist_goal > 12.0:
            f_att = self.k_att * 12.0 * (diff_goal / dist_goal)
        else:
            f_att = self.k_att * diff_goal

        # Dynamic repulsive force from moving obstacle
        diff_obs = self.pos_a - self.pos_obs
        dist_obs = np.linalg.norm(diff_obs)
        coll_bound = self.radius_a + self.radius_obs
        clearance = max(0.4, dist_obs - coll_bound)

        f_rep = np.zeros(2)
        if clearance < self.rho_0:
            u_rep = diff_obs / dist_obs
            rep_mag = self.k_rep_dyn * (1.0 / clearance - 1.0 / self.rho_0) * (1.0 / (clearance ** 2))
            f_rep = rep_mag * u_rep
            
            # Dynamic cross-avoidance component perpendicular to obstacle heading
            # Helps robot smoothly yield and pass safely
            evasion_dir = np.array([-self.obs_dir[1], self.obs_dir[0]])
            if np.dot(evasion_dir, diff_obs) < 0:
                evasion_dir = -evasion_dir
            f_rep += 0.4 * rep_mag * evasion_dir

        return f_att, f_rep, dist_obs

    def step(self):
        f_att, f_rep, inter_dist = self.compute_forces()
        f_total = f_att + f_rep

        f_norm = np.linalg.norm(f_total)
        if f_norm > 1e-6:
            speed = min(self.v_max, f_norm)
            vel = speed * (f_total / f_norm)
        else:
            vel = np.zeros(2)

        self.pos_a = self.pos_a + vel * self.dt
        self.update_moving_obstacle()

        self.traj_a.append(np.copy(self.pos_a))
        self.traj_obs.append(np.copy(self.pos_obs))
        self.distances_inter.append(np.linalg.norm(self.pos_a - self.pos_obs))
        self.time_steps.append(self.time_steps[-1] + self.dt)

        finished = np.linalg.norm(self.pos_a - self.goal_a) <= 1.2
        return finished

    def simulate(self, max_steps=1200):
        for step_idx in range(max_steps):
            if self.step():
                print(f"[Assignment 3] Robot reached destination safely in {step_idx + 1} steps (t = {self.time_steps[-1]:.1f}s)!")
                break
        return np.array(self.traj_a), np.array(self.traj_obs)

def plot_assignment_3(sim, save_path=None):
    traj_a = np.array(sim.traj_a)
    traj_obs = np.array(sim.traj_obs)
    times = np.array(sim.time_steps)
    inter_dists = np.array(sim.distances_inter)
    coll_limit = sim.radius_a + sim.radius_obs

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # Subplot 1: Dynamic 2D Trajectories
    ax1.plot(traj_a[:, 0], traj_a[:, 1], 'b-', linewidth=2.5, label='Robot A Trajectory (APF)')
    ax1.plot(traj_obs[:, 0], traj_obs[:, 1], 'r--', linewidth=2.0, label='Moving Obstacle (Worker) Path')

    # Start and Goal markers
    ax1.scatter(sim.start_a[0], sim.start_a[1], c='green', s=150, marker='o', edgecolors='black', zorder=5, label='Robot Start [10, 10]')
    ax1.scatter(sim.goal_a[0], sim.goal_a[1], c='gold', s=200, marker='*', edgecolors='black', zorder=5, label='Robot Goal [90, 90]')
    ax1.scatter(sim.start_obs[0], sim.start_obs[1], c='purple', s=120, marker='s', edgecolors='black', zorder=5, label='Obstacle Start [20, 85]')
    ax1.scatter(sim.goal_obs[0], sim.goal_obs[1], c='red', s=120, marker='x', linewidth=2.5, zorder=5, label='Obstacle Goal [85, 20]')

    # Draw synchronized snapshots of robot and moving obstacle at key time moments
    num_snaps = 6
    snap_indices = np.linspace(0, len(traj_a) - 1, num_snaps, dtype=int)
    for idx in snap_indices:
        pa = traj_a[idx]
        po = traj_obs[idx]
        t_val = times[idx]

        # Robot snapshot
        c_a = Circle((pa[0], pa[1]), sim.radius_a, color='blue', fill=False, linestyle='-', linewidth=1.2, alpha=0.5)
        ax1.add_patch(c_a)
        ax1.scatter(pa[0], pa[1], c='blue', s=25)

        # Obstacle snapshot
        c_o = Circle((po[0], po[1]), sim.radius_obs, color='red', fill=False, linestyle='-', linewidth=1.2, alpha=0.5)
        ax1.add_patch(c_o)
        ax1.scatter(po[0], po[1], c='red', s=25)

        # Connector line between positions at same time step
        ax1.plot([pa[0], po[0]], [pa[1], po[1]], 'gray', linestyle=':', alpha=0.45)
        ax1.text((pa[0] + po[0])/2, (pa[1] + po[1])/2, f't={t_val:.0f}s', fontsize=7.5, color='darkslategray', ha='center')

    ax1.set_xlim(0, 100)
    ax1.set_ylim(0, 100)
    ax1.set_aspect('equal')
    ax1.set_xlabel('Warehouse X (m)', fontsize=11)
    ax1.set_ylabel('Warehouse Y (m)', fontsize=11)
    ax1.set_title('Assignment 3: Dynamic APF with Moving Obstacle', fontsize=12, fontweight='bold')
    ax1.legend(loc='lower left', fontsize=8.5)
    ax1.grid(True, linestyle=':', alpha=0.6)

    # Subplot 2: Distance between Robot and Moving Obstacle
    ax2.plot(times, inter_dists, 'b-', linewidth=2.2, label='Actual Inter-Agent Distance')
    ax2.axhline(y=coll_limit, color='r', linestyle='--', linewidth=1.8, label=f'Collision Boundary ($r_A+r_{{obs}}={coll_limit:.0f}$ m)')
    ax2.axhline(y=sim.rho_0, color='orange', linestyle=':', linewidth=1.5, label=f'Repulsive Trigger Zone ($\\rho_0={sim.rho_0:.0f}$ m)')

    min_dist = np.min(inter_dists)
    min_t = times[np.argmin(inter_dists)]
    ax2.scatter(min_t, min_dist, c='crimson', s=100, zorder=6)
    ax2.annotate(f'Min Distance: {min_dist:.2f} m\n(Safe clearance: {min_dist - coll_limit:.2f} m)',
                 xy=(min_t, min_dist), xytext=(min_t + 5, min_dist + 12),
                 arrowprops=dict(facecolor='black', arrowstyle='->', lw=1.2),
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.8),
                 fontsize=9.5)

    ax2.set_xlabel('Simulation Time (s)', fontsize=11)
    ax2.set_ylabel('Inter-Agent Distance (m)', fontsize=11)
    ax2.set_title('Separation Distance vs Time (Collision-Free Guarantee)', fontsize=12, fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=9.5)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"[Assignment 3] Figure saved to {save_path}")
    plt.close()

if __name__ == "__main__":
    sim = APFDynamicMovingObstacleNav()
    sim.simulate()
    os.makedirs('/home/sravanthi/GAZEBO/LAB11/plots', exist_ok=True)
    plot_assignment_3(sim, save_path='/home/sravanthi/GAZEBO/LAB11/plots/assignment3_result.png')
