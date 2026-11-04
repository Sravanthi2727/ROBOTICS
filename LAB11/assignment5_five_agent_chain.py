#!/usr/bin/env python3
"""
CS359: Artificial Intelligence for Robotics - Lab Assignment 11
Assignment 5: Five-Agent Chain Formation and Following
Real-world application: Convoy-based goods transportation

Problem Statement:
Design an APF-based multi-agent simulation in which five agents form and maintain
a leader–follower chain while moving toward a common destination.
"""

import os
os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib_cache'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

class FiveAgentChainSim:
    def __init__(self, num_agents=5, radius=3.5, d_chain=12.0, d_safe=8.0,
                 k_leader_att=1.0, k_follow_att=1.2, k_follow_rep=80.0, k_avoid=120.0,
                 v_max_leader=1.4, v_max_follower=1.7, dt=0.4):
        self.num_agents = num_agents
        self.radius = radius
        self.d_chain = d_chain
        self.d_safe = d_safe
        self.k_leader_att = k_leader_att
        self.k_follow_att = k_follow_att
        self.k_follow_rep = k_follow_rep
        self.k_avoid = k_avoid
        self.v_max_leader = v_max_leader
        self.v_max_follower = v_max_follower
        self.dt = dt

        # Common destination for convoy
        self.destination = np.array([88.0, 85.0])

        # Initial staggered staging positions in warehouse
        self.initial_positions = [
            np.array([28.0, 18.0]),  # Agent 1 (Leader)
            np.array([20.0, 14.0]),  # Agent 2
            np.array([14.0, 22.0]),  # Agent 3
            np.array([12.0, 36.0]),  # Agent 4
            np.array([16.0, 52.0]),  # Agent 5
        ]

        self.positions = [np.copy(p) for p in self.initial_positions]
        self.trajectories = [[np.copy(p)] for p in self.initial_positions]
        self.chain_distances = [[] for _ in range(num_agents - 1)]
        self.time_steps = [0.0]

    def compute_leader_force(self):
        diff_dest = self.destination - self.positions[0]
        dist_dest = np.linalg.norm(diff_dest)
        if dist_dest > 12.0:
            f_dest = self.k_leader_att * 12.0 * (diff_dest / dist_dest)
        else:
            f_dest = self.k_leader_att * diff_dest
        return f_dest

    def compute_follower_force(self, k):
        pos_k = self.positions[k]
        pos_pred = self.positions[k - 1]  # Agent immediately ahead

        # 1. Chain Follower Potential with Predecessor
        diff_pred = pos_pred - pos_k
        dist_pred = np.linalg.norm(diff_pred)
        if dist_pred < 1e-4:
            diff_pred = np.array([0.1, 0.0])
            dist_pred = 0.1

        u_pred = diff_pred / dist_pred

        # Harmonic tracking around target spacing d_chain
        f_chain = self.k_follow_att * (dist_pred - self.d_chain) * u_pred

        # Additional collision safety barrier if clearance is critically low
        clearance = dist_pred - 2 * self.radius
        if clearance < (self.d_safe - 2 * self.radius):
            eff_clear = max(0.3, clearance)
            rep_mag = self.k_follow_rep * (1.0 / eff_clear - 1.0 / (self.d_safe - 2 * self.radius)) * (1.0 / (eff_clear ** 2))
            f_chain -= rep_mag * u_pred

        # 2. Collision avoidance with other non-adjacent convoy members
        f_avoid = np.zeros(2)
        for j in range(self.num_agents):
            if j == k or j == k - 1:
                continue
            diff_j = pos_k - self.positions[j]
            d_j = np.linalg.norm(diff_j)
            clear_j = d_j - 2 * self.radius
            if clear_j < (self.d_safe - 2 * self.radius) and d_j > 1e-4:
                eff_c = max(0.3, clear_j)
                u_j = diff_j / d_j
                rep = self.k_avoid * (1.0 / eff_c - 1.0 / (self.d_safe - 2 * self.radius)) * (1.0 / (eff_c ** 2))
                f_avoid += rep * u_j

        return f_chain + f_avoid

    def step(self):
        new_positions = [np.copy(p) for p in self.positions]

        # Leader Step
        f_leader = self.compute_leader_force()
        fn_leader = np.linalg.norm(f_leader)
        dist_dest_leader = np.linalg.norm(self.destination - self.positions[0])

        if dist_dest_leader > 1.0:
            speed = min(self.v_max_leader, fn_leader)
            vel_leader = speed * (f_leader / fn_leader) if fn_leader > 1e-6 else np.zeros(2)
            new_positions[0] = self.positions[0] + vel_leader * self.dt
        else:
            new_positions[0] = self.destination

        # Followers Step
        for k in range(1, self.num_agents):
            f_k = self.compute_follower_force(k)
            fn_k = np.linalg.norm(f_k)
            speed_k = min(self.v_max_follower, fn_k)
            vel_k = speed_k * (f_k / fn_k) if fn_k > 1e-6 else np.zeros(2)
            new_positions[k] = self.positions[k] + vel_k * self.dt

        self.positions = new_positions
        for k in range(self.num_agents):
            self.trajectories[k].append(np.copy(self.positions[k]))

        # Record consecutive pair distances
        for k in range(self.num_agents - 1):
            gap = np.linalg.norm(self.positions[k] - self.positions[k + 1])
            self.chain_distances[k].append(gap)

        self.time_steps.append(self.time_steps[-1] + self.dt)

        # Finished when leader reached destination and all consecutive links within 1.5m of d_chain
        leader_done = dist_dest_leader <= 1.0
        all_chain_stabilized = all(abs(gap - self.d_chain) < 1.5 for gap in self.chain_distances[-1]) if self.chain_distances[0] else False
        return leader_done and all_chain_stabilized

    def simulate(self, max_steps=1400):
        for step_idx in range(max_steps):
            if self.step():
                print(f"[Assignment 5] Convoy chain formed and leader arrived in {step_idx + 1} steps (t = {self.time_steps[-1]:.1f}s)!")
                break
        return self.trajectories

def plot_assignment_5(sim, save_path=None):
    num_agents = sim.num_agents
    trajs = [np.array(sim.trajectories[i]) for i in range(num_agents)]
    times = np.array(sim.time_steps[1:])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    labels = ['Leader (A1)', 'Follower 2 (A2)', 'Follower 3 (A3)', 'Follower 4 (A4)', 'Follower 5 (A5)']

    # Subplot 1: 2D Warehouse Convoy Trajectories
    for i in range(num_agents):
        ax1.plot(trajs[i][:, 0], trajs[i][:, 1], color=colors[i], linewidth=2.2, label=labels[i])
        ax1.scatter(sim.initial_positions[i][0], sim.initial_positions[i][1], color=colors[i], marker='o', s=90, edgecolors='black', zorder=5)
        ax1.text(sim.initial_positions[i][0]-3, sim.initial_positions[i][1]-3, f'Start A{i+1}', fontsize=8, color=colors[i], fontweight='bold')

    # Draw common destination
    ax1.scatter(sim.destination[0], sim.destination[1], color='gold', marker='*', s=220, edgecolors='black', zorder=6, label='Convoy Destination [88, 85]')

    # Draw chain links at multiple time snapshots
    snap_indices = np.linspace(0, len(trajs[0]) - 1, 6, dtype=int)
    for idx in snap_indices:
        pts = [trajs[i][idx] for i in range(num_agents)]
        for i in range(num_agents):
            circ = Circle((pts[i][0], pts[i][1]), sim.radius, color=colors[i], fill=False, linestyle=':', alpha=0.5)
            ax1.add_patch(circ)
            ax1.scatter(pts[i][0], pts[i][1], color=colors[i], s=20)
        # Connect consecutive chain members
        for i in range(num_agents - 1):
            ax1.plot([pts[i][0], pts[i+1][0]], [pts[i][1], pts[i+1][1]], color='darkgray', linestyle='-', linewidth=1.5, alpha=0.45)

    ax1.set_xlim(0, 100)
    ax1.set_ylim(0, 100)
    ax1.set_aspect('equal')
    ax1.set_xlabel('Warehouse X (m)', fontsize=11)
    ax1.set_ylabel('Warehouse Y (m)', fontsize=11)
    ax1.set_title('Assignment 5: Five-Agent Chain Formation & Convoy', fontsize=12, fontweight='bold')
    ax1.legend(loc='lower right', fontsize=8.5)
    ax1.grid(True, linestyle=':', alpha=0.6)

    # Subplot 2: Consecutive Chain Link Distances vs Time
    link_labels = ['Gap A1-A2', 'Gap A2-A3', 'Gap A3-A4', 'Gap A4-A5']
    for k in range(num_agents - 1):
        ax2.plot(times, sim.chain_distances[k], color=colors[k+1], linewidth=2.0, label=link_labels[k])

    ax2.axhline(y=sim.d_chain, color='black', linestyle='--', linewidth=1.8, label=f'Target Spacing ($d_{{chain}} = {sim.d_chain:.0f}$ m)')
    ax2.axhline(y=2 * sim.radius, color='red', linestyle=':', linewidth=1.8, label=f'Collision Limit ($2r = {2*sim.radius:.0f}$ m)')

    ax2.set_xlabel('Simulation Time (s)', fontsize=11)
    ax2.set_ylabel('Consecutive Spacing (m)', fontsize=11)
    ax2.set_title('Convoy Chain Spacing Convergence vs Time', fontsize=12, fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=9.5)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"[Assignment 5] Figure saved to {save_path}")
    plt.close()

if __name__ == "__main__":
    sim = FiveAgentChainSim()
    sim.simulate()
    os.makedirs('/home/sravanthi/GAZEBO/LAB11/plots', exist_ok=True)
    plot_assignment_5(sim, save_path='/home/sravanthi/GAZEBO/LAB11/plots/assignment5_result.png')
