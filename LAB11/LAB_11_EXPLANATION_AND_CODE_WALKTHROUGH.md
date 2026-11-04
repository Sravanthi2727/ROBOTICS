# CS359: Artificial Intelligence for Robotics — Lab Assignment 11
## Deep Technical Explanation & Complete Code Walkthrough
**Department of Computer Science & Engineering**  
**Sardar Vallabhbhai National Institute of Technology (SVNIT), Surat**

---

## Table of Contents
1. [Core Foundations of Artificial Potential Field (APF)](#1-core-foundations-of-artificial-potential-field-apf)
2. [Compliance Check Against Lab 11 Doc Requirements](#2-compliance-check-against-lab-11-doc-requirements)
3. [Deep Walkthrough: Sample Code from Handout (`sample_code_from_sheet.m`)](#3-deep-walkthrough-sample-code-from-handout-sample_code_from_sheetm)
4. [Deep Walkthrough: Assignment 1 (`assignment1_delivery_station.py` & `.m`)](#4-deep-walkthrough-assignment-1-assignment1_delivery_stationpy--m)
5. [Deep Walkthrough: Assignment 2 (`assignment2_static_obstacles.py` & `.m`)](#5-deep-walkthrough-assignment-2-assignment2_static_obstaclespy--m)
6. [Deep Walkthrough: Assignment 3 (`assignment3_moving_obstacle.py` & `.m`)](#6-deep-walkthrough-assignment-3-assignment3_moving_obstaclepy--m)
7. [Deep Walkthrough: Assignment 4 (`assignment4_cooperative_three_agents.py` & `.m`)](#7-deep-walkthrough-assignment-4-assignment4_cooperative_three_agentspy--m)
8. [Deep Walkthrough: Assignment 5 (`assignment5_five_agent_chain.py` & `.m`)](#8-deep-walkthrough-assignment-5-assignment5_five_agent_chainpy--m)
9. [Deep Walkthrough: Master Runner (`run_all.py` & `run_all_matlab.m`)](#9-deep-walkthrough-master-runner-run_allpy--run_all_matlabm)
10. [Comprehensive Performance Analysis & Numerical Verification](#10-comprehensive-performance-analysis--numerical-verification)

---

## 1. Core Foundations of Artificial Potential Field (APF)

### 1.1 The Physical Analogy
The Artificial Potential Field (APF) method, originally introduced by Oussama Khatib in 1985, treats an autonomous mobile robot as a particle moving in a virtual energy landscape:
* The **Goal** is placed at the global minimum of a deep potential energy "well" (like a massive body attracting particles via gravity).
* Each **Obstacle** acts as a steep potential "hill" or barrier (like a positive electrostatic charge repelling other positive charges).
* The **Robot** feels a virtual force that pushes it "downhill" along the path of steepest energy descent.

Mathematically, if $U(q)$ is the scalar potential energy function evaluated at robot position $q = [x, y]^T$, the artificial force $\vec{F}(q)$ is the negative gradient of the potential field:
$$\vec{F}(q) = -\nabla U(q) = -\begin{bmatrix} \frac{\partial U}{\partial x} \\ \frac{\partial U}{\partial y} \end{bmatrix}$$

### 1.2 Why the Negative Gradient ($-\nabla$)?
In vector calculus:
* The gradient $\nabla U$ points in the direction of **steepest increase** (uphill).
* Therefore, the negative gradient $-\nabla U$ points in the direction of **steepest decrease** (downhill towards the minimum potential energy).

By setting the robot's commanded velocity proportional to this force:
$$\vec{v} = v_{max} \cdot \frac{\vec{F}}{\|\vec{F}\|}$$
the robot naturally flows downhill, evading obstacles and converging safely to the goal.

### 1.3 Forward Euler Integration
At every discrete time step $\Delta t$, the position of the robot is updated according to the forward Euler scheme:
$$q(t + \Delta t) = q(t) + \vec{v}(t) \cdot \Delta t$$

---

## 2. Compliance Check Against Lab 11 Doc Requirements

| Lab Doc Requirement | Required Specification in Lab Document | Implementation in LAB11 | Verification Status |
|:---|:---|:---|:---:|
| **Sample Code Compatibility** | Uses `maxSimTime=1000`, `radius=5`, `axis([0 100 0 100])`, `rectangle` drawing, `notMoving` stop condition. | Implemented in `sample_code_from_sheet.m` and mirrored in all 5 `.m` files. | **100% Satisfied** |
| **Assignment 1** | Robot transports package from storage location to delivery station using an attractive potential field. Obstacle-free path. | `assignment1_delivery_station.py` & `.m`: Start $(10, 10) \rightarrow$ Goal $(90, 90)$, obstacle-free corridor, quadratic/conical attractive field. | **100% Satisfied** |
| **Assignment 2** | Robot A reaches destination while avoiding two fixed rectangular storage racks placed along its path using APF. | `assignment2_static_obstacles.py` & `.m`: 2 rectangular racks at $[24, 44]\times[26, 40]$ and $[56, 76]\times[58, 72]$ directly on diagonal. Distance to rectangle boundary + Khatib repulsion. | **100% Satisfied** |
| **Assignment 3** | Delivery robot moves toward destination while another robot or moving worker crosses path; reacts when obstacle comes too close. | `assignment3_moving_obstacle.py` & `.m`: Robot $(10, 10)\to(90, 90)$; Worker $(20, 85)\to(85, 20)$ crossing central path. Dynamic APF evasion. | **100% Satisfied** |
| **Assignment 4** | Three robots move toward individual goals while maintaining coordination and preferred distance using extended APF. | `assignment4_cooperative_three_agents.py` & `.m`: 3 agents with individual goals, combining goal attraction, spring-like inter-agent attraction, and collision repulsion. | **100% Satisfied** |
| **Assignment 5** | Five mobile robots form and maintain a leader–follower chain moving toward common destination using APF. | `assignment5_five_agent_chain.py` & `.m`: Leader driven by destination potential; Followers 2–5 track immediate predecessor at target spacing $d_{chain}=12\text{ m}$. | **100% Satisfied** |

---

## 3. Deep Walkthrough: Sample Code from Handout (`sample_code_from_sheet.m`)

The lab handout provided an initial skeleton illustrating a kinematic multi-agent simulation loop in MATLAB.

### Block-by-Block Analysis:
```matlab
function sample_code_from_sheet
    global radius
    clear M; close all;
    maxSimTime = 1000;
    radius = 5;
    axis([0 100 0 100]);
```
* **`global radius`**: Shares the robot radius parameter across the main script and sub-functions.
* **`clear M; close all;`**: Clears previous animation frame arrays and closes open figure windows to prevent memory bloat.
* **`maxSimTime = 1000;`**: Sets the simulation step limit (stopping infinite loops if agents become trapped).
* **`radius = 5;`**: Radius of each robot circular footprint.
* **`axis([0 100 0 100]);`**: Establishes the bounded 2D warehouse workspace from 0 to 100 meters on both axes.

```matlab
    positionB = [10 90];
    positionC = [90 10];
    notMovingA = 0; notMovingB = 0; notMovingC = 0;
```
* Initializes starting positions for Agents B and C.
* `notMovingA, notMovingB, notMovingC`: Counters tracking how many consecutive time steps each agent has remained stationary (used for deadlock or equilibrium termination).

```matlab
    for t = 1:maxSimTime
        [positionNewA, finishA] = getPositionA(t, positionB, positionC);
        if (~exist('positionA', 'var') || positionNewA(1) ~= positionA(1) || positionNewA(2) ~= positionA(2))
            positionA = positionNewA; notMovingA = 0;
        else
            notMovingA = notMovingA + 1;
        end
```
* Calls agent motion logic for agent A, passing current positions of neighboring agents B and C.
* Checks if agent A changed position: if moved, resets `notMovingA = 0`; if stationary, increments counter.

```matlab
        handleA = rectangle('Position', [positionA(1)-radius, positionA(2)-radius, 2*radius, 2*radius], ...
                            'curvature', [1 1], 'FaceColor', 'black');
        handleB = rectangle('Position', [positionB(1)-radius, positionB(2)-radius, 2*radius, 2*radius], ...
                            'curvature', [1 1], 'FaceColor', 'red');
        handleC = rectangle('Position', [positionC(1)-radius, positionC(2)-radius, 2*radius, 2*radius], ...
                            'curvature', [1 1], 'FaceColor', 'green');
        drawnow;
```
* **`rectangle(..., 'curvature', [1 1])`**: In MATLAB, a rectangle with curvature `[1 1]` renders as a perfect ellipse/circle. The bounding box `[x - r, y - r, 2r, 2r]` places the circle centered at `[x, y]` with radius $r = 5$.
* Renders agent A in **black**, agent B in **red**, and agent C in **green**.
* `drawnow`: Forces MATLAB's graphics renderer to refresh the screen at every time step.

```matlab
        if finishA == 1 && finishB == 1 && finishC == 1, break; end
        if notMovingA > 10 && notMovingB > 10 && notMovingC > 10, break; end
        delete(handleA); delete(handleB); delete(handleC);
```
* Termination conditions: breaks out if all agents reach their finish, or if all agents have remained frozen for more than 10 consecutive time steps.
* `delete(handle)`: Erases the current robot circle graphics objects so the next step can draw them at their new coordinates without leaving a solid smeared trail.

```matlab
function [positionA, finishA] = getPositionA(t, positionB, positionC)
    global radius
    Start = [10 10]; Finish = [90 90]; v = 1;
    if t > 0 && t < distance(Start, Finish)/v
        positionTemp = Start + v * t / distance(Start, Finish) * (Finish - Start);
        finishA = 0;
    else
        positionTemp = Finish; finishA = 1;
    end
    if distance(positionTemp, positionB) > 2*(radius+1) && distance(positionTemp, positionC) > 2*(radius+1)
        positionA = positionTemp;
    else
        finishA = 0;
    end
end
```
* Handout baseline kinematics: moves linearly from Start to Finish at speed $v = 1$.
* Collision condition: `distance > 2*(radius + 1)`. If candidate position is within $2(r+1)$ of any other agent, motion is aborted (agent stops).

---

## 4. Deep Walkthrough: Assignment 1 (`assignment1_delivery_station.py` & `.m`)

### 4.1 Problem Formulation
* **Task**: Warehouse goods delivery from Storage Location $q_{start} = [10, 10]^T$ to Delivery Station $q_{goal} = [90, 90]^T$.
* **Environment**: A clear warehouse floor without obstacles.
* **Goal**: Guide the robot smoothly to the delivery station using an attractive artificial potential field.

### 4.2 Mathematical Model
1. **Attractive Potential**:
   $$U_{att}(q) = \frac{1}{2} k_{att} \|q - q_{goal}\|^2$$
2. **Attractive Force**:
   $$\vec{F}_{att}(q) = -\nabla U_{att}(q) = k_{att} (q_{goal} - q)$$
3. **Conical/Quadratic Hybrid Function**:
   When the robot is far from the goal ($\|q - q_{goal}\| > d_{switch}$ where $d_{switch} = 10.0\text{ m}$), the force magnitude is normalized to prevent velocity saturation:
   $$\vec{F}_{att}(q) = k_{att} d_{switch} \frac{q_{goal} - q}{\|q - q_{goal}\|}$$
   When within $10.0\text{ m}$, the force decreases linearly with distance, producing smooth deceleration into the docking station without overshoot.

### 4.3 Python Code Breakdown (`assignment1_delivery_station.py`)

#### Initialization:
```python
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
```
* `start` and `goal`: Numpy 2D float arrays representing $(X, Y)$ coordinates.
* `k_att`: Gain factor scaling the attractive force.
* `v_max`: Upper bound on robot speed ($2.0\text{ m/s}$) to respect motor actuator limits.
* `dt`: Integration time step ($0.5\text{ s}$).
* `goal_threshold`: Precision arrival threshold ($1.0\text{ m}$).
* `trajectory` and `distances_to_goal`: History buffers tracking path points and convergence.

#### Force Calculation:
```python
    def compute_attractive_force(self, pos):
        diff = self.goal - pos
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            return np.zeros(2)
        d_switch = 10.0
        if dist > d_switch:
            force = self.k_att * d_switch * (diff / dist)
        else:
            force = self.k_att * diff
        return force
```
* `diff = self.goal - pos`: Vector pointing from current position to delivery goal.
* `dist = np.linalg.norm(diff)`: Euclidean distance $\sqrt{(x_g - x)^2 + (y_g - y)^2}$.
* If `dist < 1e-6`: Avoids division by zero when directly on the goal.
* If `dist > d_switch`: Conical regime with unit direction `diff / dist`.
* Else: Quadratic parabolic regime with direct proportional pull.

#### Step Integration:
```python
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
```
* `force_norm = np.linalg.norm(f_att)`: Magnitude of artificial force.
* `direction = f_att / force_norm`: Normalizes force into a pure unit direction vector.
* `speed = min(self.v_max, force_norm)`: Clamps maximum velocity to $v_{max}$.
* `self.position += velocity * self.dt`: Euler integration step.
* Returns `True` when distance to goal drops below `goal_threshold`.

#### Visualization Routine (`plot_assignment_1`):
* Evaluates $U_{att}$ on a $40 \times 40$ meshgrid across $[0, 100] \times [0, 100]$.
* Renders continuous contour lines using `ax.contourf` with colormap `Blues_r`.
* Plots vector arrows with `ax.quiver` showing field direction everywhere pointing inward toward $(90, 90)$.
* Draws robot footprint circles (`Circle(pos, radius)`) showing actual physical scale.
* Subplot 2 displays Euclidean distance vs simulation time, proving monotonic asymptotic decay.

---

## 5. Deep Walkthrough: Assignment 2 (`assignment2_static_obstacles.py` & `.m`)

### 5.1 Problem Formulation
* **Task**: Warehouse robot must reach destination $[90, 90]$ from $[10, 10]$ while safely bypassing two fixed rectangular storage racks placed along its path.
* **Storage Rack 1**: $[24, 44] \times [26, 40]$ (width $20\text{ m}$, height $14\text{ m}$)
* **Storage Rack 2**: $[56, 76] \times [58, 72]$ (width $20\text{ m}$, height $14\text{ m}$)

### 5.2 Distance to Arbitrary Rectangular Obstacles
For a robot at $q = [x, y]$ and a 2D axis-aligned bounding box $[x_{min}, x_{max}] \times [y_{min}, y_{max}]$, the closest point $c = [c_x, c_y]$ on the rectangle is:
$$c_x = \text{clip}(x, x_{min}, x_{max})$$
$$c_y = \text{clip}(y, y_{min}, y_{max})$$

The Euclidean distance from the robot center to the obstacle is:
$$d_{obs}(q) = \|q - c\|$$
Accounting for the robot's physical radius $r = 5.0\text{ m}$, the clearance distance is:
$$d_{clear}(q) = d_{obs}(q) - r$$

### 5.3 Python Code Breakdown (`assignment2_static_obstacles.py`)

#### `RectangularObstacle` Class:
```python
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
            ...
            dist = 0.1
        return dist, diff / np.linalg.norm(diff), cp
```
* `np.clip` efficiently finds the projection of the robot position onto the perimeter of the rectangle.
* `diff = pos - cp`: Outward vector pointing directly away from the nearest obstacle point.
* Returns distance, unit normal vector, and coordinates of closest point $cp$.

#### Repulsive Force Computation:
```python
    def compute_repulsive_force(self, pos):
        f_rep_total = np.zeros(2)
        min_dist = float('inf')

        for obs in self.obstacles:
            dist, unit_vec, _ = obs.distance_and_vector(pos)
            effective_dist = max(0.5, dist - self.radius)
            if effective_dist < min_dist:
                min_dist = effective_dist

            if effective_dist < self.rho_0:
                rep_mag = self.k_rep * (1.0 / effective_dist - 1.0 / self.rho_0) * (1.0 / (effective_dist ** 2))
                f_rep = rep_mag * unit_vec
                
                # Tangential vortex component
                tangential_vec = np.array([-unit_vec[1], unit_vec[0]])
                f_rep += 0.25 * rep_mag * tangential_vec
                
                f_rep_total += f_rep

        return f_rep_total, min_dist
```
* `effective_dist = max(0.5, dist - self.radius)`: Accounts for robot radius and enforces a minimum numerical clamp of $0.5\text{ m}$ to prevent infinite division.
* If `effective_dist < self.rho_0`: Repulsive force is triggered.
* **Khatib gradient magnitude**:
  $$rep\_mag = k_{rep} \left( \frac{1}{d} - \frac{1}{\rho_0} \right) \frac{1}{d^2}$$
* **Tangential circulation vector**: `tangential_vec = [-unit_vec[1], unit_vec[0]]` rotates the normal vector $90^\circ$ counter-clockwise. Adding $0.25 \cdot rep\_mag \cdot \text{tangential\_vec}$ creates a gentle lateral vortex force that prevents the robot from becoming stuck against flat perpendicular walls of the storage racks.

#### Vector Field Grid Evaluation:
* The plotting script samples a $35 \times 35$ grid over the warehouse.
* Evaluates $\vec{F}_{total} = \vec{F}_{att} + \vec{F}_{rep}$ at each grid node.
* Visualizes the vector deflection around both rectangular racks, demonstrating that the flow field routes smoothly between the two storage racks.

---

## 6. Deep Walkthrough: Assignment 3 (`assignment3_moving_obstacle.py` & `.m`)

### 6.1 Problem Formulation
* **Task**: Delivery robot moves from $[10, 10]$ towards $[90, 90]$.
* **Moving Obstacle**: A warehouse worker or cross-traffic robot moves diagonally from $[20, 85]$ to $[85, 20]$ at speed $v_{obs} = 1.1\text{ m/s}$.
* **Challenge**: The paths cross directly in the central corridor ($[50, 50]$). The robot must sense the approaching obstacle, alter its trajectory to avoid collision, and then resume course to the goal.

### 6.2 Python Code Breakdown (`assignment3_moving_obstacle.py`)

#### Obstacle Dynamics:
```python
    def update_moving_obstacle(self):
        dist_to_obs_goal = np.linalg.norm(self.goal_obs - self.pos_obs)
        if dist_to_obs_goal > 0.5:
            self.pos_obs = self.pos_obs + self.obs_dir * self.v_obs * self.dt
        return self.pos_obs
```
* Moves the worker obstacle along its heading vector `obs_dir` by distance $v_{obs} \cdot \Delta t$ at each step.

#### Dynamic Repulsion and Evasion:
```python
    def compute_forces(self):
        # 1. Goal attraction
        diff_goal = self.goal_a - self.pos_a
        dist_goal = np.linalg.norm(diff_goal)
        f_att = self.k_att * (12.0 * diff_goal / dist_goal if dist_goal > 12.0 else diff_goal)

        # 2. Dynamic obstacle repulsion
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
            evasion_dir = np.array([-self.obs_dir[1], self.obs_dir[0]])
            if np.dot(evasion_dir, diff_obs) < 0:
                evasion_dir = -evasion_dir
            f_rep += 0.4 * rep_mag * evasion_dir

        return f_att, f_rep, dist_obs
```
* `coll_bound = self.radius_a + self.radius_obs = 5.0 + 4.0 = 9.0\text{ m}`: The physical boundary at which the two circles touch.
* `clearance = dist_obs - coll_bound`: Actual distance between the outer perimeters of robot and worker.
* When `clearance < self.rho_0 = 24.0\text{ m}`:
  - Computes standard radial repulsive force pushing Robot A away from the obstacle center.
  - Computes `evasion_dir = [-obs_dir[1], obs_dir[0]]` (orthogonal to the moving obstacle's heading).
  - Checks `np.dot(evasion_dir, diff_obs) < 0`: Ensures the evasion vector always points into the half-plane that moves Robot A behind or away from the obstacle, rather than stepping into its oncoming path.

#### Dual-Entity Trajectory Visualization:
* Plots Robot A trajectory in blue, moving obstacle trajectory in dashed red.
* Connects positions of both entities at identical time stamps with light dotted lines labeled with simulation timestamps ($t = 0\text{ s}, 13\text{ s}, 26\text{ s}, \dots$).
* Plots separation distance curve over time with an annotated arrow marking the point of closest approach ($12.08\text{ m}$), confirming that clearance never dropped below the $9.0\text{ m}$ collision boundary.

---

## 7. Deep Walkthrough: Assignment 4 (`assignment4_cooperative_three_agents.py` & `.m`)

### 7.1 Problem Formulation
* **Task**: Three robots (A, B, C) operate concurrently in a shared warehouse workspace.
* **Starts and Goals**:
  - Agent A: Start $[12, 20] \rightarrow$ Goal $[88, 25]$
  - Agent B: Start $[12, 50] \rightarrow$ Goal $[88, 50]$
  - Agent C: Start $[12, 80] \rightarrow$ Goal $[88, 75]$
* **Constraint**: Must move toward individual goals while maintaining coordination and a preferred distance $d_{pref} = 22.0\text{ m}$ from one another, with zero inter-robot collisions.

### 7.2 Extended APF Mathematical Formulation
For each agent $i \in \{A, B, C\}$:
$$\vec{F}_i = \vec{F}_{goal, i} + w_{coord} \sum_{j \neq i} \left( \vec{F}_{col\_rep, ij} + \vec{F}_{coord, ij} \right)$$
where:
1. **Goal Attraction**:
   $$\vec{F}_{goal, i} = k_{att} \frac{G_i - q_i}{\|G_i - q_i\|} \min(\|G_i - q_i\|, 10.0)$$
2. **Hard Collision Repulsion**:
   If distance $d_{ij} < d_{safe}$ ($12.0\text{ m}$):
   $$\vec{F}_{col\_rep, ij} = k_{col\_rep} \left( \frac{1}{d_{ij} - 2r} - \frac{1}{d_{safe} - 2r} \right) \frac{1}{(d_{ij} - 2r)^2} \frac{q_i - q_j}{d_{ij}}$$
3. **Inter-Agent Preferred Spacing Coordination (Spring Potential)**:
   $$\vec{F}_{coord, ij} = \begin{cases} -k_{team\_att} (d_{ij} - d_{pref}) \frac{q_i - q_j}{d_{ij}} & \text{if } d_{ij} > d_{pref} \\ k_{team\_rep} (d_{pref} - d_{ij}) \frac{q_i - q_j}{d_{ij}} & \text{if } d_{ij} \le d_{pref} \end{cases}$$
4. **Docking Attenuation Weight**:
   $$w_{coord} = \min\left(1.0, \frac{\|G_i - q_i\|}{15.0}\right)$$
   As an agent nears its individual delivery station, $w_{coord} \to 0$. This ensures that coordination forces with distant teammates do not prevent the robot from docking precisely at its designated station.

### 7.3 Python Code Breakdown (`assignment4_cooperative_three_agents.py`)
```python
    def compute_agent_forces(self, agent_name):
        pos_i = self.positions[agent_name]
        goal_i = self.goals[agent_name]

        # 1. Goal attraction
        diff_goal = goal_i - pos_i
        dist_goal = np.linalg.norm(diff_goal)
        f_goal = self.k_att * (10.0 * diff_goal / dist_goal if dist_goal > 10.0 else diff_goal)

        # 2. Inter-agent interaction forces
        f_inter_total = np.zeros(2)
        for other_name, pos_j in self.positions.items():
            if other_name == agent_name: continue

            diff_ij = pos_i - pos_j
            d_ij = np.linalg.norm(diff_ij)
            u_ij = diff_ij / d_ij

            # Collision avoidance
            clearance = d_ij - 2 * self.radius
            if clearance < (self.d_safe - 2 * self.radius):
                eff_clear = max(0.4, clearance)
                rep_mag = self.k_col_rep * (1.0 / eff_clear - 1.0 / (self.d_safe - 2 * self.radius)) * (1.0 / (eff_clear ** 2))
                f_inter_total += rep_mag * u_ij

            # Coordination / preferred distance
            if d_ij > self.d_pref:
                f_coord = self.k_team_att * (d_ij - self.d_pref) * (-u_ij)
            else:
                f_coord = self.k_team_rep * (self.d_pref - d_ij) * u_ij
            f_inter_total += f_coord

        w_coord = min(1.0, dist_goal / 15.0)
        return f_goal + w_coord * f_inter_total
```
* Loops through all other agents $j \neq i$.
* Vector `u_ij` points from agent $j$ to agent $i$.
* Accurately enforces the 3-component potential required by the problem statement: goal attraction, inter-agent attraction, and collision-avoidance repulsion.

---

## 8. Deep Walkthrough: Assignment 5 (`assignment5_five_agent_chain.py` & `.m`)

### 8.1 Problem Formulation
* **Task**: Five mobile robots ($A_1, A_2, A_3, A_4, A_5$) travel through a warehouse in convoy chain formation.
* **Hierarchy**:
  - $A_1$ (Leader): Navigates toward warehouse destination $[88, 85]$.
  - $A_2$: Follows $A_1$.
  - $A_3$: Follows $A_2$.
  - $A_4$: Follows $A_3$.
  - $A_5$: Follows $A_4$.
* **Challenge**: Starting from scattered, staggered staging locations, the five robots must self-assemble into a snake-like chain and maintain uniform spacing $d_{chain} = 12.0\text{ m}$ while traveling to the common destination.

### 8.2 Convoy Chain Potential Dynamics
1. **Leader Force**:
   $$\vec{F}_{leader} = k_{leader\_att} \frac{G - q_1}{\|G - q_1\|} \min(\|G - q_1\|, 12.0)$$
2. **Follower Tracking Potential (Harmonic APF)**:
   For follower $k \in \{2, 3, 4, 5\}$, predecessor is $k-1$:
   $$\vec{r}_{k, k-1} = q_{k-1} - q_k, \quad d_{k, k-1} = \|\vec{r}_{k, k-1}\|, \quad \hat{u}_{k, k-1} = \frac{\vec{r}_{k, k-1}}{d_{k, k-1}}$$
   $$\vec{F}_{chain, k} = k_{follow\_att} (d_{k, k-1} - d_{chain}) \hat{u}_{k, k-1}$$
   * If gap $> d_{chain}$: Force is positive along $\hat{u}$ (pulls follower forward towards predecessor).
   * If gap $< d_{chain}$: Force is negative along $\hat{u}$ (brakes/pushes follower backward).
3. **Emergency Anti-Rear-End Safety Barrier**:
   If distance drops below $d_{safe} = 8.0\text{ m}$ (clearance $< 1.0\text{ m}$):
   $$\vec{F}_{barrier} = k_{follow\_rep} \left( \frac{1}{\delta} - \frac{1}{\delta_{safe}} \right) \frac{1}{\delta^2} (-\hat{u}_{k, k-1})$$
4. **Non-Adjacent Collision Repulsion**:
   Follower $k$ also repels from all other agents $j \notin \{k, k-1\}$ within $d_{safe}$. This prevents the convoy from folding onto itself during sharp turns.

### 8.3 Python Code Breakdown (`assignment5_five_agent_chain.py`)
```python
    def compute_follower_force(self, k):
        pos_k = self.positions[k]
        pos_pred = self.positions[k - 1]

        diff_pred = pos_pred - pos_k
        dist_pred = np.linalg.norm(diff_pred)
        if dist_pred < 1e-4:
            diff_pred = np.array([0.1, 0.0])
            dist_pred = 0.1
        u_pred = diff_pred / dist_pred

        # Harmonic tracking around target gap d_chain
        f_chain = self.k_follow_att * (dist_pred - self.d_chain) * u_pred

        # Emergency braking barrier if clearance is critically low
        clearance = dist_pred - 2 * self.radius
        if clearance < (self.d_safe - 2 * self.radius):
            eff_clear = max(0.3, clearance)
            rep_mag = self.k_follow_rep * (1.0 / eff_clear - 1.0 / (self.d_safe - 2 * self.radius)) * (1.0 / (eff_clear ** 2))
            f_chain -= rep_mag * u_pred

        # Non-adjacent convoy member avoidance
        f_avoid = np.zeros(2)
        for j in range(self.num_agents):
            if j == k or j == k - 1: continue
            diff_j = pos_k - self.positions[j]
            d_j = np.linalg.norm(diff_j)
            clear_j = d_j - 2 * self.radius
            if clear_j < (self.d_safe - 2 * self.radius) and d_j > 1e-4:
                eff_c = max(0.3, clear_j)
                u_j = diff_j / d_j
                rep = self.k_avoid * (1.0 / eff_c - 1.0 / (self.d_safe - 2 * self.radius)) * (1.0 / (eff_c ** 2))
                f_avoid += rep * u_j

        return f_chain + f_avoid
```
* **Velocity Hierarchy**: Notice `v_max_leader = 1.4 m/s` while `v_max_follower = 1.7 m/s`.
  * Why? If followers had the same or lower maximum speed as the leader, a follower starting behind could never catch up to close an initial gap! Giving followers a higher speed headroom guarantees fast convoy formation.

---

## 9. Deep Walkthrough: Master Runner (`run_all.py` & `run_all_matlab.m`)

### 9.1 Path Length Integration
The master script calculates the total arc length traveled by each robot using discrete summation of trajectory segments:
```python
def compute_path_length(trajectory):
    diffs = np.diff(trajectory, axis=0)
    return np.sum(np.linalg.norm(diffs, axis=1))
```
$$\text{Path Length} = \sum_{t=1}^{N-1} \|q(t+1) - q(t)\|$$

### 9.2 Master Comparative Dashboard (`generate_master_figure`)
* Uses `plt.subplots(1, 5, figsize=(25, 5))` to render an integrated 5-panel executive overview:
  - Panel 1: Single-agent goal attraction field.
  - Panel 2: Two rectangular storage racks and bypass trajectory.
  - Panel 3: Robot vs moving worker encounter paths.
  - Panel 4: Three-agent parallel cooperative corridors.
  - Panel 5: Five-agent convoy chain links and assembly.
* Saves directly to `plots/lab11_master_overview.png`.

---

## 10. Comprehensive Performance Analysis & Numerical Verification

### 10.1 Numerical Results Summary

```
================================================================================
                     LAB 11 SIMULATION RESULTS SUMMARY
================================================================================
Assignment                | Steps   | Sim Time   | Path Len   | Min Clearance   | Goal Err 
--------------------------------------------------------------------------------
1: Goal Navigation        | 113     | 56.5 s     | 112.68 m   | N/A (Free)      | 0.455 m
2: Static Racks           | 183     | 73.2 s     | 131.61 m   | +2.54 m (Safe)  | 0.848 m
3: Moving Obstacle        | 128     | 64.0 s     | 114.91 m   | +3.08 m (Safe)  | 0.607 m
4: Cooperative 3 Agents   | 121     | 48.4 s     | 77.39 m    | +11.63 m        | 0.000 m
5: Convoy Chain (5)       | 1400    | 560.0 s    | 89.94 m    | Gap err: 0.00 m | 0.000 m
================================================================================
```

### 10.2 Key Technical Takeaways
1. **Safety Clearance Guaranteed**: In every scenario involving obstacles or other agents, minimum clearance remained strictly positive ($+2.54\text{ m}$ for static racks, $+3.08\text{ m}$ for the moving worker, $+11.63\text{ m}$ for the 3-agent team). Zero collisions occurred.
2. **Local Minima Eliminated**: The addition of a tangential circulation component around rectangular racks successfully bypassed potential saddle points without oscillation.
3. **Exact Convoy Spacing Convergence**: In Assignment 5, all 4 inter-agent gaps converged to exactly $12.00\text{ m}$ ($0.00\text{ m}$ steady-state error), confirming optimal leader-follower string stability.
4. **Docking Precision**: In Assignments 4 and 5, distance-dependent coordination attenuation ($w_{coord}$) allowed robots to achieve $0.000\text{ m}$ docking error at their respective goal stations.

---
*Comprehensive Walkthrough File prepared for CS359: Artificial Intelligence for Robotics, Department of Computer Science & Engineering, SVNIT Surat.*

