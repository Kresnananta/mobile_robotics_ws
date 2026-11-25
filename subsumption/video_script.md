# English Video Narration Script — ROS 2 Subsumption Architecture (TurtleBot3)

**Estimated Duration:** ~4 minutes  
**Presenter:** Anak Agung Ngurah Agung Kresna Ananta (NRP 5024241085) — Computer Engineering, ITS

---

## Scene 1 — Introduction & System Overview `[0:00 – 0:35]`
*(Show Gazebo `turtlebot3_world`, RViz2, and terminal ready)*

> Hello everyone. My name is Anak Agung Ngurah Agung Kresna Ananta, NRP 5024241085, from Computer Engineering, ITS.
>
> In this video, I will demonstrate my implementation of a three-layer **Subsumption Architecture** on a **TurtleBot3 Burger** simulated in **ROS 2 Humble**, **Gazebo**, and **RViz2**.
>
> This project adapts the paper *"Subsumption Model Implemented on ROS for Mobile Robots"* by Minglong Li and colleagues, which reproduces Rodney Brooks' autonomous wander robot using loosely coupled ROS nodes, Finite State Machines, and reusable **Inhibitor** and **Suppressor** templates.
>
> Rather than combining all logic into one monolithic controller, every behavior layer and every coordination unit is built as an independent ROS 2 node communicating asynchronously over publish-subscribe topics, using the 360-degree 2D LiDAR on `/scan` and velocity commands on `/cmd_vel`. On the right side, RViz2 visualizes the live LiDAR points in red and the robot's odometry trajectory trail in blue.

---

## Scene 2 — Level 0: Avoid Contact (Survival Reflex) `[0:35 – 1:25]`
*(Run `level0_avoid.launch.py`. Pause 4–5 seconds in open space, then place a box / drag robot near a pillar in Gazebo.)*

> Let's test the architecture incrementally, starting with **Level 0: Avoid Contact**, which is the lowest survival reflex layer.
>
> Right now, only `avoid_node` is running, publishing directly to `/cmd_vel`. Because the robot starts in open space with no obstacles inside its 0.32-meter danger threshold, its Finite State Machine remains in `IDLE_SAFE`, and the robot stays completely stationary.
>
> Now, watch what happens when an obstacle gets too close.
>
> *(Place a box close to the TurtleBot3 or move the robot near a pillar.)*
>
> Immediately, `avoid_node` detects the obstacle via `/scan`, computes a virtual repulsive force inspired by Brooks' `feelforce` module, and transitions into `EMERGENCY_COLLIDE` and `RUNAWAY`. The robot backs away and turns toward open space. Once its minimum clearance exceeds the 0.37-meter hysteresis threshold, it sends a single zero-velocity stop command and returns to `IDLE_SAFE`, halting cleanly with zero collisions.

---

## Scene 3 — Level 0 + 1: Adding Wander & Safety Suppressor `[1:25 – 2:15]`
*(Stop Level 0, press `Ctrl+R` in Gazebo, click `Reset` in RViz2, then run `level1_wander.launch.py`.)*

> Next, we stack **Layer 1: Wander** on top of Layer 0 by launching `level1_wander.launch.py`.
>
> This adds two nodes: `wander_node`, which periodically generates random turns and forward cruising commands on `/wander/cmd_vel`, and `suppressor_safety`—an instance of our reusable Suppressor template.
>
> Notice that `wander_node` does not subscribe to `/scan` at all; it has zero awareness of obstacles. Normally, its velocity commands pass straight through `suppressor_safety` to `/cmd_vel`, so the robot wanders aimlessly around the arena.
>
> However, whenever the robot approaches a wall or pillar, `avoid_node` fires an escape velocity on `/avoid/cmd_vel`. In the terminal log, we can see `suppressor_safety` immediately suppressing `/wander/cmd_vel` and replacing it with `/avoid/cmd_vel` for its 0.5-second timespan. Once the robot is safely clear, the suppressor releases control and random wandering resumes automatically.

---

## Scene 4 — Level 0 + 1 + 2: Full Subsumption (Explore + Inhibitor + Suppressor) `[2:15 – 3:15]`
*(Stop Level 1, press `Ctrl+R` in Gazebo, click `Reset` in RViz2, then run `level2_explore.launch.py`.)*

> Now, let's launch the full three-layer architecture with `level2_explore.launch.py`.
>
> This adds **Layer 2: Explore** (`explore_node`) along with two more coordination nodes: `inhibitor_wander` and `suppressor_l1_l2`.
>
> `explore_node` analyzes 360-degree LiDAR scans across overlapping 32-degree angular sectors—mapping `inf` readings to maximum sensor range so open corridors score highest, while using a lower-percentile term to penalize thin pillars.
>
> When it detects a clear open corridor, it enters `LOOK_SCAN` and sends a trigger to `/explore/inhibit`. Our **Inhibitor node** (`inhibitor_wander`) compares incoming timestamps and blocks `/wander/cmd_vel` for 1.5 seconds so random turns cannot interfere. Then, `explore_node` publishes smooth steering commands on `/explore/cmd_vel`, which **Suppressor 1** (`suppressor_l1_l2`) forwards to replace Wander.
>
> You can clearly see the contrast in both Gazebo and the blue odometry trail in RViz2: instead of meandering randomly in one corner, the robot decisively locks onto the long open lane between the pillars, drives straight across the arena, and transitions smoothly into the next corridor—while `suppressor_safety` still lets Layer 0 intervene locally whenever needed.

---

## Scene 5 — `rqt_graph` Architecture Walkthrough & Quantitative Comparison `[3:15 – 4:30]`
*(Part 1: Switch to the live `rqt_graph` window while Level 2 is still running, and point your mouse cursor along the graph from left to right as you speak.)*

> To verify that our implementation truly follows a modular multi-node Subsumption Architecture rather than a monolithic script, let's inspect the live `rqt_graph` for Level 2.
>
> Starting from the left, the 2D LiDAR topic `/scan` from Gazebo feeds into two sensor-driven modules: **`avoid_node`** for Layer 0 and **`explore_node`** for Layer 2. Meanwhile, **`wander_node`** for Layer 1 operates purely on its internal timer without subscribing to `/scan`.
>
> Now, let's trace the coordination pipeline from Layer 1 up to the robot's actuator:
> - First, `wander_node` publishes random velocities on `/wander/cmd_vel` into **`inhibitor_wander`**. Whenever `explore_node` scans or plans a corridor, it sends a Boolean trigger on `/explore/inhibit` to this Inhibitor node. Inside the node, two callbacks compare timestamps; if the time difference is under its 1.5-second timespan, the wander message is dropped.
> - Second, the inhibited wander output `/wander/inhibited_cmd_vel` enters **`suppressor_l1_l2`** alongside `/explore/cmd_vel` from `explore_node`. This Suppressor node forwards Explore's commands directly and blocks the lower input for 1.0 second, outputting the chosen navigation command on `/subsumption/nav_cmd_vel`.
> - Finally, `/subsumption/nav_cmd_vel` enters **`suppressor_safety`** together with `/avoid/cmd_vel` from Layer 0. Placing `suppressor_safety` right before `/cmd_vel` mirrors Minglong Li's ROS architecture and guarantees that survival obstacle avoidance can always override both Wander and Explore within a 0.5-second safety window, preventing priority inversion and collisions.

*(Part 2: Open `results/trajectory_comparison.png` and briefly walk through the three panels from left to right.)*

> Finally, let's examine the quantitative 2D trajectory comparison recorded from identical 55-second runs in `turtlebot3_world`. Each path is color-coded by which layer controlled `/cmd_vel`: **red** for Layer 0 Avoid, **orange** for Layer 1 Wander, **blue** for Layer 2 Explore, and **grey** for Idle, with the green circle marking the start and the purple X marking the end.
>
> - On the **left panel (Level 0: Avoid Only)**, when spawned close to a pillar at 0.20 meters, `avoid_node` activates in red, escapes 0.31 meters to safe clearance, and immediately transitions to grey Idle—while in open space it remains 100 percent stationary with 0.00 meters of drift.
> - In the **middle panel (Level 0 + 1: Avoid + Wander)**, the robot wanders stochastically in orange for 74.5 percent of the run, traveling 5.42 meters and covering 2.11 square meters, with crisp red Layer 0 deflections (23.8 percent) keeping minimum obstacle clearance at 0.279 meters.
> - On the **right panel (Level 0 + 1 + 2: Full Subsumption)**, the improvement in autonomy is unmistakable: `explore_node` dominates 78.4 percent of the run in blue, guiding the robot straight down the long horizontal lane across the pillar grid and smoothly into the perpendicular vertical corridor. In the exact same timeframe, total path length increases by **35 percent** to **7.34 meters**, and explored bounding-box area more than **doubles**—rising **122 percent** to **4.69 square meters**—while `suppressor_safety` keeps minimum clearance at **0.260 meters**, well above the TurtleBot3 Burger's 0.105-meter radius, achieving zero collisions across all tests.
>
> That concludes my demonstration of the ROS 2 Subsumption Architecture on TurtleBot3. Thank you for watching!
