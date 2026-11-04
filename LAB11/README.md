# CS359: Artificial Intelligence for Robotics — Lab Assignment 11
**Department of Computer Science & Engineering**  
**Sardar Vallabhbhai National Institute of Technology (SVNIT), Surat**  
**B.Tech. (CSE) IIIrd Year**

---

## Artificial Potential Field (APF) Approach for Multi-Scenario Robot Navigation

This repository contains the complete implementation, simulations, visualizations, and academic report for **Lab Assignment 11** on the **Artificial Potential Field (APF)** approach.

The project models real-world autonomous robotic tasks across five progressively complex scenarios: single-agent navigation, static obstacle avoidance, dynamic moving worker/obstacle evasion, cooperative multi-agent team navigation, and 5-agent convoy chain formation.

Both **Python** (executable with automated matplotlib graphics & metrics compilation) and **MATLAB/Octave** (following the exact department template structure) implementations are provided.

---

## Directory Structure

```
~/GAZEBO/LAB11/
├── plots/                                       # High-resolution generated simulation figures
│   ├── assignment1_result.png                   # Attractive field contours & distance vs time
│   ├── assignment2_result.png                   # Two rectangular racks avoidance & clearance
│   ├── assignment3_result.png                   # Dynamic worker evasion & minimum distance
│   ├── assignment4_result.png                   # 3-agent cooperative coordination & gaps
│   ├── assignment5_result.png                   # 5-agent convoy chain assembly & spacing
│   └── lab11_master_overview.png                # Master 5-panel comparative overview
│
├── assignment1_delivery_station.py              # Assignment 1 Python implementation
├── assignment2_static_obstacles.py              # Assignment 2 Python implementation
├── assignment3_moving_obstacle.py               # Assignment 3 Python implementation
├── assignment4_cooperative_three_agents.py      # Assignment 4 Python implementation
├── assignment5_five_agent_chain.py              # Assignment 5 Python implementation
├── run_all.py                                   # Master runner executing all 5 assignments
│
├── assignment1_delivery_station.m               # Assignment 1 MATLAB implementation
├── assignment2_static_obstacles.m               # Assignment 2 MATLAB implementation
├── assignment3_moving_obstacle.m                # Assignment 3 MATLAB implementation
├── assignment4_cooperative_three_agents.m       # Assignment 4 MATLAB implementation
├── assignment5_five_agent_chain.m               # Assignment 5 MATLAB implementation
├── sample_code_from_sheet.m                     # Baseline sample code from handout sheet
├── run_all_matlab.m                             # Master runner for MATLAB
│
├── HOW_TO_RUN.txt                               # Quick execution instructions
├── README.md                                    # This documentation
├── LAB_11_REPORT.md                             # Comprehensive academic lab report
└── LAB_11_DEEP_EXPLANATION_AND_CODE_WALKTHROUGH.md # Line-by-line block explanation of all code
```

---

## Assignments Summary & Mathematical Formulations

### Assignment 1: Autonomous Robot Navigation to a Delivery Station
* **Real-world Application**: Warehouse package transport from storage rack to dispatch station.
* **Environment**: Obstacle-free warehouse floor $[0, 100] \times [0, 100]$.
* **Start**: $(10, 10)$, **Goal**: $(90, 90)$, Robot Radius: $5.0\text{ m}$.
* **APF Formulation**:
  $$U_{att}(q) = \frac{1}{2} k_{att} \|q - q_{goal}\|^2$$
  $$F_{att}(q) = -\nabla U_{att}(q) = k_{att} (q_{goal} - q)$$
* **Performance**: 113 steps ($56.5\text{ s}$), smooth straight trajectory, $0.45\text{ m}$ final arrival error.

---

### Assignment 2: Robot Navigation with Static Obstacles
* **Real-world Application**: Collision-free navigation between fixed storage racks.
* **Environment**: Contains two rectangular storage racks placed along the path:
  - Rack 1: $[24, 44] \times [26, 40]$
  - Rack 2: $[56, 76] \times [58, 72]$
* **APF Formulation**:
  $$F_{total}(q) = F_{att}(q) + \sum_{i=1}^2 F_{rep, i}(q)$$
  For closest boundary point $c_i$ on rack $i$ and clearance $d_{i} = \|q - c_i\| - r$:
  $$F_{rep, i}(q) = k_{rep} \left( \frac{1}{d_i} - \frac{1}{\rho_0} \right) \frac{1}{d_i^2} \frac{q - c_i}{\|q - c_i\|} \quad (\text{for } d_i < \rho_0)$$
  Includes tangential vortex bias to prevent local minima on rectangular faces.
* **Performance**: 183 steps ($73.2\text{ s}$), min clearance $2.54\text{ m}$ (zero collisions).

---

### Assignment 3: Robot Navigation with a Moving Obstacle
* **Real-world Application**: Collision avoidance with crossing AGV or walking warehouse worker.
* **Entities**:
  - Delivery Robot: $(10, 10) \rightarrow (90, 90)$
  - Moving Worker: $(20, 85) \rightarrow (85, 20)$ crossing central corridor at $1.1\text{ m/s}$.
* **APF Formulation**: Time-varying dynamic repulsive field:
  $$F_{rep, dyn}(t) = k_{rep\_dyn} \left( \frac{1}{d(t) - (r_A + r_{obs})} - \frac{1}{\rho_0} \right) \frac{1}{(d(t) - (r_A + r_{obs}))^2} \hat{u}_{AO} + F_{evade}$$
* **Performance**: 128 steps ($64.0\text{ s}$), min distance $12.08\text{ m}$ (well above physical limit $9.0\text{ m}$, giving $3.08\text{ m}$ safe buffer).

---

### Assignment 4: Cooperative Navigation of Three Agents
* **Real-world Application**: Collaborative warehouse robot fleet moving to individual destinations while maintaining team cohesion.
* **APF Formulation**: Combines 3 distinct potential forces for each agent $i$:
  1. Goal attraction: $F_{goal, i}$
  2. Collision repulsion: $F_{col, ij}$ when $d_{ij} < d_{safe}$
  3. Coordination field: Spring-like potential around preferred spacing $d_{pref} = 22.0\text{ m}$:
     $$F_{coord, ij} = \begin{cases} -k_{team\_att} (d_{ij} - d_{pref}) \hat{u}_{ij} & \text{if } d_{ij} > d_{pref} \\ k_{team\_rep} (d_{pref} - d_{ij}) \hat{u}_{ij} & \text{if } d_{ij} \le d_{pref} \end{cases}$$
* **Performance**: 121 steps ($48.4\text{ s}$), all 3 agents converge to their respective docking goals with $0.0\text{ m}$ error and maintain minimum separation $> 11.63\text{ m}$.

---

### Assignment 5: Five-Agent Chain Formation & Following
* **Real-world Application**: Multi-agent goods transportation convoy.
* **Topology**: Agent 1 is Leader; Agents 2–5 are Followers tracking their immediate predecessor.
* **APF Formulation**:
  - Leader driven by global goal attraction $F_{dest}$.
  - Follower $k$ driven by harmonic inter-agent tracking potential with predecessor $k-1$:
    $$F_{chain, k} = k_{follow\_att} (d_{k, k-1} - d_{chain}) \hat{u}_{k, k-1}$$
    coupled with non-adjacent collision avoidance:
    $$F_{avoid, kj} = k_{avoid} \left( \frac{1}{d_{kj}} - \frac{1}{d_{safe}} \right) \frac{1}{d_{kj}^2} \hat{u}_{kj}$$
* **Performance**: Convoy self-assembles from staggered initial locations, snakes smoothly across the warehouse, and arrives at destination with all inter-agent gaps converging to exactly $12.00\text{ m}$ ($0.00\text{ m}$ gap error).

---

## Quantitative Results Benchmark

| Assignment | Description | Steps | Sim Time | Path Length | Min Obstacle Clearance | Goal Error | Status |
|:---|:---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Assignment 1** | Delivery Station Navigation | 113 | 56.5 s | 112.68 m | N/A (Obstacle-Free) | 0.455 m | **SUCCESS** |
| **Assignment 2** | 2 Static Storage Racks | 183 | 73.2 s | 131.61 m | +2.54 m (Safe Buffer) | 0.848 m | **SUCCESS** |
| **Assignment 3** | Moving Worker Obstacle | 128 | 64.0 s | 114.91 m | +3.08 m (Safe Buffer) | 0.607 m | **SUCCESS** |
| **Assignment 4** | Cooperative 3-Agent Team | 121 | 48.4 s | 77.39 m | +11.63 m ($> 2r$) | 0.000 m | **SUCCESS** |
| **Assignment 5** | 5-Agent Convoy Chain | 1400 | 560.0 s | 89.94 m | Gap Error: 0.00 m | 0.000 m | **SUCCESS** |

---

## How to Run

### Run All in Python:
```bash
cd ~/GAZEBO/LAB11
python3 run_all.py
```

### Run Individual Scripts in Python:
```bash
python3 assignment1_delivery_station.py
python3 assignment2_static_obstacles.py
python3 assignment3_moving_obstacle.py
python3 assignment4_cooperative_three_agents.py
python3 assignment5_five_agent_chain.py
```

### Run in MATLAB / GNU Octave:
```matlab
cd ~/GAZEBO/LAB11
run_all_matlab
```

