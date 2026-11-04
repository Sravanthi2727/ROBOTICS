#!/usr/bin/env python3
"""
CS359: Artificial Intelligence for Robotics - Lab Assignment 11
Master Execution Script for All Five APF Assignments

Runs simulations for Assignments 1 through 5, compiles quantitative metrics,
and generates individual high-res figures plus an integrated master comparison overview.
"""

import os
os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib_cache'
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from assignment1_delivery_station import APFDeliveryRobot, plot_assignment_1
from assignment2_static_obstacles import APFStaticObstaclesNav, plot_assignment_2
from assignment3_moving_obstacle import APFDynamicMovingObstacleNav, plot_assignment_3
from assignment4_cooperative_three_agents import CooperativeThreeAgentsSim, plot_assignment_4
from assignment5_five_agent_chain import FiveAgentChainSim, plot_assignment_5

def compute_path_length(trajectory):
    diffs = np.diff(trajectory, axis=0)
    return np.sum(np.linalg.norm(diffs, axis=1))

def run_all_assignments():
    plots_dir = os.path.join(os.path.dirname(__file__), "plots")
    os.makedirs(plots_dir, exist_ok=True)

    print("=" * 80)
    print(" SARDAR VALLABHBHAI NATIONAL INSTITUTE OF TECHNOLOGY, SURAT")
    print(" COMPUTER SCIENCE & ENGINEERING DEPARTMENT")
    print(" CS359: Artificial Intelligence for Robotics — Lab Assignment 11")
    print(" Artificial Potential Field (APF) Approach")
    print("=" * 80)

    results = []

    # -------------------------------------------------------------
    # Assignment 1
    # -------------------------------------------------------------
    print("\n>>> Running Assignment 1: Navigation to Delivery Station (APF)...")
    t0 = time.time()
    sim1 = APFDeliveryRobot()
    sim1.simulate()
    t1 = time.time()
    plot_assignment_1(sim1, os.path.join(plots_dir, "assignment1_result.png"))
    traj1 = np.array(sim1.trajectory)
    plen1 = compute_path_length(traj1)
    results.append({
        "Assignment": "1: Goal Navigation",
        "Steps": len(traj1) - 1,
        "Sim Time (s)": sim1.time_steps[-1],
        "Path Length (m)": plen1,
        "Min Clearance": "N/A (Free)",
        "Goal Error (m)": np.linalg.norm(traj1[-1] - sim1.goal)
    })

    # -------------------------------------------------------------
    # Assignment 2
    # -------------------------------------------------------------
    print("\n>>> Running Assignment 2: Static Obstacles Avoidance (APF)...")
    t0 = time.time()
    sim2 = APFStaticObstaclesNav()
    sim2.simulate()
    t1 = time.time()
    plot_assignment_2(sim2, os.path.join(plots_dir, "assignment2_result.png"))
    traj2 = np.array(sim2.trajectory)
    plen2 = compute_path_length(traj2)
    min_clear2 = min(sim2.min_distances_to_obs)
    results.append({
        "Assignment": "2: Static Racks",
        "Steps": len(traj2) - 1,
        "Sim Time (s)": sim2.time_steps[-1],
        "Path Length (m)": plen2,
        "Min Clearance": f"{min_clear2:.2f} m",
        "Goal Error (m)": np.linalg.norm(traj2[-1] - sim2.goal)
    })

    # -------------------------------------------------------------
    # Assignment 3
    # -------------------------------------------------------------
    print("\n>>> Running Assignment 3: Moving Obstacle Avoidance (Dynamic APF)...")
    t0 = time.time()
    sim3 = APFDynamicMovingObstacleNav()
    sim3.simulate()
    t1 = time.time()
    plot_assignment_3(sim3, os.path.join(plots_dir, "assignment3_result.png"))
    traj3 = np.array(sim3.traj_a)
    plen3 = compute_path_length(traj3)
    coll_limit3 = sim3.radius_a + sim3.radius_obs
    min_dist3 = min(sim3.distances_inter)
    results.append({
        "Assignment": "3: Moving Obstacle",
        "Steps": len(traj3) - 1,
        "Sim Time (s)": sim3.time_steps[-1],
        "Path Length (m)": plen3,
        "Min Clearance": f"{min_dist3 - coll_limit3:.2f} m",
        "Goal Error (m)": np.linalg.norm(traj3[-1] - sim3.goal_a)
    })

    # -------------------------------------------------------------
    # Assignment 4
    # -------------------------------------------------------------
    print("\n>>> Running Assignment 4: Cooperative Three Agents...")
    t0 = time.time()
    sim4 = CooperativeThreeAgentsSim()
    sim4.simulate()
    t1 = time.time()
    plot_assignment_4(sim4, os.path.join(plots_dir, "assignment4_result.png"))
    traj4_a = np.array(sim4.trajectories['A'])
    plen4_a = compute_path_length(traj4_a)
    min_inter_dist4 = min(min(sim4.distances_inter['AB']), min(sim4.distances_inter['BC']), min(sim4.distances_inter['CA']))
    results.append({
        "Assignment": "4: Cooperative 3 Agents",
        "Steps": len(traj4_a) - 1,
        "Sim Time (s)": sim4.time_steps[-1],
        "Path Length (m)": plen4_a,
        "Min Clearance": f"{min_inter_dist4 - 2 * sim4.radius:.2f} m",
        "Goal Error (m)": np.mean([np.linalg.norm(sim4.positions[k] - sim4.goals[k]) for k in ['A', 'B', 'C']])
    })

    # -------------------------------------------------------------
    # Assignment 5
    # -------------------------------------------------------------
    print("\n>>> Running Assignment 5: Five-Agent Chain Formation & Following...")
    t0 = time.time()
    sim5 = FiveAgentChainSim()
    sim5.simulate()
    t1 = time.time()
    plot_assignment_5(sim5, os.path.join(plots_dir, "assignment5_result.png"))
    traj5_lead = np.array(sim5.trajectories[0])
    plen5_lead = compute_path_length(traj5_lead)
    final_gaps = [np.linalg.norm(sim5.positions[k] - sim5.positions[k+1]) for k in range(4)]
    max_gap_error = max(abs(g - sim5.d_chain) for g in final_gaps)
    results.append({
        "Assignment": "5: Convoy Chain (5)",
        "Steps": len(traj5_lead) - 1,
        "Sim Time (s)": sim5.time_steps[-1],
        "Path Length (m)": plen5_lead,
        "Min Clearance": f"Gap err: {max_gap_error:.2f} m",
        "Goal Error (m)": np.linalg.norm(sim5.positions[0] - sim5.destination)
    })

    # -------------------------------------------------------------
    # Print Executive Summary Table
    # -------------------------------------------------------------
    print("\n" + "=" * 80)
    print("                     LAB 11 SIMULATION RESULTS SUMMARY")
    print("=" * 80)
    header = f"{'Assignment':<25} | {'Steps':<7} | {'Sim Time':<10} | {'Path Len':<10} | {'Min Clearance':<15} | {'Goal Err':<9}"
    print(header)
    print("-" * 80)
    for r in results:
        print(f"{r['Assignment']:<25} | {r['Steps']:<7} | {r['Sim Time (s)']:<8.1f} s | {r['Path Length (m)']:<8.2f} m | {r['Min Clearance']:<15} | {r['Goal Error (m)']:<7.3f} m")
    print("=" * 80)

    # -------------------------------------------------------------
    # Generate Master Overview Figure
    # -------------------------------------------------------------
    generate_master_figure(sim1, sim2, sim3, sim4, sim5, os.path.join(plots_dir, "lab11_master_overview.png"))
    print(f"\n[Master Overview] Figure saved to {os.path.join(plots_dir, 'lab11_master_overview.png')}")

def generate_master_figure(s1, s2, s3, s4, s5, save_path):
    fig, axes = plt.subplots(1, 5, figsize=(25, 5))

    # P1
    t1 = np.array(s1.trajectory)
    axes[0].plot(t1[:, 0], t1[:, 1], 'b-', lw=2)
    axes[0].scatter(s1.start[0], s1.start[1], c='g', s=80, marker='o')
    axes[0].scatter(s1.goal[0], s1.goal[1], c='gold', s=120, marker='*')
    axes[0].set_title("1: Goal Attraction", fontsize=11, fontweight='bold')

    # P2
    t2 = np.array(s2.trajectory)
    from matplotlib.patches import Rectangle
    for obs in s2.obstacles:
        r = Rectangle((obs.x_min, obs.y_min), obs.x_max - obs.x_min, obs.y_max - obs.y_min,
                      fc='coral', ec='k', hatch='//', alpha=0.7)
        axes[1].add_patch(r)
    axes[1].plot(t2[:, 0], t2[:, 1], 'b-', lw=2)
    axes[1].scatter(s2.start[0], s2.start[1], c='g', s=80, marker='o')
    axes[1].scatter(s2.goal[0], s2.goal[1], c='gold', s=120, marker='*')
    axes[1].set_title("2: Static Racks", fontsize=11, fontweight='bold')

    # P3
    t3 = np.array(s3.traj_a)
    to3 = np.array(s3.traj_obs)
    axes[2].plot(t3[:, 0], t3[:, 1], 'b-', lw=2, label='Robot')
    axes[2].plot(to3[:, 0], to3[:, 1], 'r--', lw=1.8, label='Obstacle')
    axes[2].scatter(s3.start_a[0], s3.start_a[1], c='g', s=80, marker='o')
    axes[2].scatter(s3.goal_a[0], s3.goal_a[1], c='gold', s=120, marker='*')
    axes[2].set_title("3: Moving Obstacle", fontsize=11, fontweight='bold')
    axes[2].legend(fontsize=8, loc='lower left')

    # P4
    colors4 = {'A': 'k', 'B': 'r', 'C': 'g'}
    for name in ['A', 'B', 'C']:
        tp = np.array(s4.trajectories[name])
        axes[3].plot(tp[:, 0], tp[:, 1], color=colors4[name], lw=2, label=f'Agent {name}')
        axes[3].scatter(s4.starts[name][0], s4.starts[name][1], color=colors4[name], s=60)
        axes[3].scatter(s4.goals[name][0], s4.goals[name][1], color=colors4[name], marker='*', s=100)
    axes[3].set_title("4: 3-Agent Cooperation", fontsize=11, fontweight='bold')
    axes[3].legend(fontsize=8, loc='lower right')

    # P5
    colors5 = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    for i in range(5):
        tp = np.array(s5.trajectories[i])
        axes[4].plot(tp[:, 0], tp[:, 1], color=colors5[i], lw=2, label=f'A{i+1}')
        axes[4].scatter(s5.initial_positions[i][0], s5.initial_positions[i][1], color=colors5[i], s=50)
    axes[4].scatter(s5.destination[0], s5.destination[1], color='gold', marker='*', s=140, ec='k')
    axes[4].set_title("5: 5-Agent Convoy Chain", fontsize=11, fontweight='bold')
    axes[4].legend(fontsize=8, loc='lower right')

    for ax in axes:
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_aspect('equal')
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.set_xlabel("X (m)", fontsize=10)
        ax.set_ylabel("Y (m)", fontsize=10)

    plt.suptitle("CS359 Lab 11: Artificial Potential Field (APF) Simulations Overview", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

if __name__ == "__main__":
    run_all_assignments()

