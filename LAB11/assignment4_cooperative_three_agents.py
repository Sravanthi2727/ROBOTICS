#!/usr/bin/env python3
"""
CS359: Artificial Intelligence for Robotics - Lab Assignment 11
Assignment 4: Cooperative Navigation of Three Agents
Real-world application: Cooperative warehouse robots and team navigation

Problem Statement:
Simulate three agents using an extended APF model that combines goal attraction,
inter-agent attraction, and collision-avoidance repulsion.
"""

import os
os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib_cache'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

class CooperativeThreeAgentsSim:
    def __init__(self, radius=5.0, d_pref=22.0, d_safe=12.0,
                 k_att=1.2, k_col_rep=180.0, k_team_att=0.08, k_team_rep=0.15,
                 v_max=1.6, dt=0.4):
        self.radius = radius
        self.d_pref = d_pref
        self.d_safe = d_safe
        self.k_att = k_att
        self.k_col_rep = k_col_rep
        self.k_team_att = k_team_att
        self.k_team_rep = k_team_rep
        self.v_max = v_max
        self.dt = dt

        # Initial positions and individual goals for 3 cooperative agents
        # Agents cross and coordinate across the warehouse floor
        self.starts = {
            'A': np.array([12.0, 20.0]),
            'B': np.array([12.0, 50.0]),
            'C': np.array([12.0, 80.0])
        }
        self.goals = {
            'A': np.array([88.0, 25.0]),
            'B': np.array([88.0, 50.0]),
            'C': np.array([88.0, 75.0])
        }

        self.positions = {k: np.copy(v) for k, v in self.starts.items()}
        self.trajectories = {k: [np.copy(v)] for k, v in self.starts.items()}
        self.distances_inter = {'AB': [], 'BC': [], 'CA': []}
        self.time_steps = [0.0]

    def compute_agent_forces(self, agent_name):
        pos_i = self.positions[agent_name]
        goal_i = self.goals[agent_name]

        # 1. Individual Goal Attraction Force
        diff_goal = goal_i - pos_i
        dist_goal = np.linalg.norm(diff_goal)
        if dist_goal > 10.0:
            f_goal = self.k_att * 10.0 * (diff_goal / dist_goal)
        else:
            f_goal = self.k_att * diff_goal

        # 2. Inter-Agent Interaction Forces (Collision Repulsion + Preferred Spacing Attraction/Repulsion)
        f_inter_total = np.zeros(2)

        for other_name, pos_j in self.positions.items():
            if other_name == agent_name:
                continue

            diff_ij = pos_i - pos_j
            d_ij = np.linalg.norm(diff_ij)
            if d_ij < 1e-4:
                diff_ij = np.random.uniform(-0.1, 0.1, size=2)
                d_ij = np.linalg.norm(diff_ij)

            u_ij = diff_ij / d_ij

            # Collision-Avoidance Repulsion (Activated when too close)
            clearance = d_ij - 2 * self.radius
            if clearance < (self.d_safe - 2 * self.radius):
                eff_clear = max(0.4, clearance)
                rep_mag = self.k_col_rep * (1.0 / eff_clear - 1.0 / (self.d_safe - 2 * self.radius)) * (1.0 / (eff_clear ** 2))
                f_inter_total += rep_mag * u_ij

            # Coordination Field (Spring/Equilibrium around d_pref)
            # Pulls agents together when d_ij > d_pref (inter-agent attraction)
            # Pushes agents gently apart when d_ij < d_pref (inter-agent repulsion)
            if d_ij > self.d_pref:
                f_coord = self.k_team_att * (d_ij - self.d_pref) * (-u_ij)
            else:
                f_coord = self.k_team_rep * (self.d_pref - d_ij) * u_ij
            
            f_inter_total += f_coord

        # Smoothly attenuate coordination force as agent nears its final parking goal
        w_coord = min(1.0, dist_goal / 15.0)
        return f_goal + w_coord * f_inter_total

    def step(self):
        new_positions = {}
        all_finished = True

        for name in ['A', 'B', 'C']:
            dist_to_goal = np.linalg.norm(self.goals[name] - self.positions[name])
            if dist_to_goal <= 1.2:
                new_positions[name] = self.goals[name]
                continue

            all_finished = False
            f_total = self.compute_agent_forces(name)
            f_norm = np.linalg.norm(f_total)

            if f_norm > 1e-6:
                speed = min(self.v_max, f_norm)
                vel = speed * (f_total / f_norm)
            else:
                vel = np.zeros(2)

            new_positions[name] = self.positions[name] + vel * self.dt

        self.positions = new_positions
        for k in ['A', 'B', 'C']:
            self.trajectories[k].append(np.copy(self.positions[k]))

        # Record pairwise inter-agent distances
        self.distances_inter['AB'].append(np.linalg.norm(self.positions['A'] - self.positions['B']))
        self.distances_inter['BC'].append(np.linalg.norm(self.positions['B'] - self.positions['C']))
        self.distances_inter['CA'].append(np.linalg.norm(self.positions['C'] - self.positions['A']))
        self.time_steps.append(self.time_steps[-1] + self.dt)

        return all_finished

    def simulate(self, max_steps=1200):
        for step_idx in range(max_steps):
            if self.step():
                print(f"[Assignment 4] All three agents reached their goals in {step_idx + 1} steps (t = {self.time_steps[-1]:.1f}s)!")
                break
        return self.trajectories

def plot_assignment_4(sim, save_path=None):
    traj_a = np.array(sim.trajectories['A'])
    traj_b = np.array(sim.trajectories['B'])
    traj_c = np.array(sim.trajectories['C'])
    times = np.array(sim.time_steps[1:])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # Subplot 1: 2D Trajectories of 3 Agents
    colors = {'A': 'black', 'B': 'red', 'C': 'green'}
    labels = {'A': 'Agent A (Black)', 'B': 'Agent B (Red)', 'C': 'Agent C (Green)'}

    for name, traj in [('A', traj_a), ('B', traj_b), ('C', traj_c)]:
        ax1.plot(traj[:, 0], traj[:, 1], color=colors[name], linewidth=2.4, label=labels[name])
        ax1.scatter(sim.starts[name][0], sim.starts[name][1], color=colors[name], marker='o', s=130, edgecolors='black', zorder=5)
        ax1.scatter(sim.goals[name][0], sim.goals[name][1], color=colors[name], marker='*', s=180, edgecolors='black', zorder=5)
        ax1.text(sim.starts[name][0]-3, sim.starts[name][1]-4, f'Start {name}', fontsize=9, fontweight='bold', color=colors[name])
        ax1.text(sim.goals[name][0]-4, sim.goals[name][1]+3, f'Goal {name}', fontsize=9, fontweight='bold', color=colors[name])

    # Draw footprints and cooperative triangle links at key timestamps
    sample_indices = np.linspace(0, len(traj_a)-1, 5, dtype=int)
    for idx in sample_indices:
        pa, pb, pc = traj_a[idx], traj_b[idx], traj_c[idx]
        for p, c in [(pa, 'black'), (pb, 'red'), (pc, 'green')]:
            circ = Circle((p[0], p[1]), sim.radius, color=c, fill=False, linestyle=':', alpha=0.5)
            ax1.add_patch(circ)
            ax1.scatter(p[0], p[1], color=c, s=25)
        # Formation links between the 3 agents
        ax1.plot([pa[0], pb[0], pc[0], pa[0]], [pa[1], pb[1], pc[1], pa[1]], 'gray', linestyle='--', alpha=0.35)

    ax1.set_xlim(0, 100)
    ax1.set_ylim(0, 100)
    ax1.set_aspect('equal')
    ax1.set_xlabel('Warehouse X (m)', fontsize=11)
    ax1.set_ylabel('Warehouse Y (m)', fontsize=11)
    ax1.set_title('Assignment 4: Cooperative 3-Agent Navigation (APF)', fontsize=12, fontweight='bold')
    ax1.legend(loc='lower right', fontsize=9)
    ax1.grid(True, linestyle=':', alpha=0.6)

    # Subplot 2: Pairwise Inter-Agent Distances vs Time
    ax2.plot(times, sim.distances_inter['AB'], 'b-', linewidth=2.0, label='Distance Agent A - Agent B ($d_{AB}$)')
    ax2.plot(times, sim.distances_inter['BC'], 'g-', linewidth=2.0, label='Distance Agent B - Agent C ($d_{BC}$)')
    ax2.plot(times, sim.distances_inter['CA'], 'm-', linewidth=2.0, label='Distance Agent C - Agent A ($d_{CA}$)')

    ax2.axhline(y=sim.d_pref, color='darkorange', linestyle='--', linewidth=1.8, label=f'Preferred Distance ($d_{{pref}}={sim.d_pref:.0f}$ m)')
    ax2.axhline(y=2 * sim.radius, color='crimson', linestyle=':', linewidth=1.8, label=f'Physical Collision Boundary ($2r={2*sim.radius:.0f}$ m)')

    ax2.set_xlabel('Simulation Time (s)', fontsize=11)
    ax2.set_ylabel('Inter-Agent Separation (m)', fontsize=11)
    ax2.set_title('Inter-Agent Spacing Coordination vs Time', fontsize=12, fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=9.5)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"[Assignment 4] Figure saved to {save_path}")
    plt.close()

if __name__ == "__main__":
    sim = CooperativeThreeAgentsSim()
    sim.simulate()
    os.makedirs('/home/sravanthi/GAZEBO/LAB11/plots', exist_ok=True)
    plot_assignment_4(sim, save_path='/home/sravanthi/GAZEBO/LAB11/plots/assignment4_result.png')
