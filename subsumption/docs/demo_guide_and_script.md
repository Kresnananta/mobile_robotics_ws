# Live Demo Guide & English Video Narration Script

Dokumen ini berisi panduan langkah demi langkah menjalankan simulasi Gazebo + `rqt_graph` sendiri di Ubuntu 24.04 (`ros2_humble` container) sekaligus naskah narasi lengkap berbahasa Inggris untuk perekaman video demo (±3.5–4.5 menit).

---

## Part A — Step-by-Step Setup Sebelum Mulai Merekam

Susun layar Anda dengan layout yang nyaman dilihat di video (misalnya: **Gazebo GUI** di kiri/atas ~65%, **Terminal 2 (launch logs)** di kanan bawah, dan **rqt_graph** dibuka saat sesi arsitektur / Level 2).

### 1. Terminal 1 (Host → Container): Menyalakan Gazebo GUI

Jalankan di terminal host Ubuntu 24.04:

```bash
xhost +local:docker
docker start ros2_humble
docker exec -it -e DISPLAY=$DISPLAY ros2_humble bash
```

Lalu di dalam container `ros2_humble`:

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}"

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

> **Tips Kamera Gazebo:** Arahkan sudut kamera Gazebo agak miring dari atas (*top-down isometric*) sehingga seluruh arena heksagonal dan 9 pilar terlihat jelas.

### 2. Terminal 2 (Host → Container): Siapkan Eksekusi Inkremental

Buka terminal baru di host:

```bash
docker exec -it ros2_humble bash
source /opt/ros/humble/setup.bash
source /mobile_robotics_ws/install/setup.bash
```

Urutan perintah yang akan dijalankan selama rekaman:

1. **Level 0 (Avoid Only):**
   ```bash
   ros2 launch turtlebot3_subsumption level0_avoid.launch.py
   ```
   *Aksi di Gazebo:* Biarkan 4–5 detik agar terlihat robot diam di ruang terbuka (`IDLE_SAFE`). Lalu klik ikon **Box/Cylinder** di toolbar atas Gazebo dan letakkan tepat di depan/belakang TurtleBot3 (`< 0.32 m`), atau klik robot lalu tekan `T` (*Translate*) dan geser dekat pilar. Robot akan langsung mundur/berputar menjauh hingga aman lalu berhenti sendiri! Setelah selesai, hapus box sementara (klik kanan → Delete) dan tekan `Ctrl+C` di Terminal 2.

2. **Reset posisi robot** di Gazebo: klik jendela Gazebo lalu tekan `Ctrl+R` (*Reset World*), atau jalankan stop singkat jika perlu:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
   ```

3. **Level 0 + 1 (Avoid + Wander):**
   ```bash
   ros2 launch turtlebot3_subsumption level1_wander.launch.py
   ```
   *Aksi di Gazebo:* Biarkan robot berkeliling acak selama ±25–30 detik hingga terlihat 1–2 kali momen `avoid_node` mengambil alih lewat `suppressor_safety` saat mendekati dinding/pilar. Lalu `Ctrl+C` di Terminal 2 dan tekan `Ctrl+R` di Gazebo.

4. **Level 0 + 1 + 2 (Full Subsumption: Avoid + Wander + Explore):**
   ```bash
   ros2 launch turtlebot3_subsumption level2_explore.launch.py
   ```
   *Aksi di Gazebo:* Biarkan robot berjalan ±30–40 detik. Terlihat jelas robot langsung mengunci lorong terbuka yang panjang di antara pilar, melaju lurus menyeberangi arena, dan berbelok mulus ke koridor berikutnya sambil tetap menghindari pilar lokal.

### 3. Terminal 3 (Host → Container): Menyalakan RViz2 (Live LiDAR + Jejak Lintasan)

Jalankan perintah ini di terminal baru sebelum mulai merekam (RViz2 sudah otomatis terkonfigurasi menampilkan `RobotModel`, sinar `/scan`, dan jejak lintasan `/odom`):

```bash
docker exec -it -e DISPLAY=$DISPLAY -e ROS_DOMAIN_ID=85 ros2_humble bash -c \
  "source /opt/ros/humble/setup.bash && source /mobile_robotics_ws/install/setup.bash && ros2 launch turtlebot3_subsumption rviz.launch.py"
```

*(Tips: Saat berganti dari Level 0 → Level 1 → Level 2, klik **Reset** di pojok kiri bawah RViz2 jika ingin membersihkan jejak panah `/odom` lama agar lintasan tiap level terlihat bersih.)*

### 4. Terminal 4 (Host → Container): Menampilkan `rqt_graph`

Saat **Level 2** masih berjalan, buka Terminal 4:

```bash
docker exec -it -e DISPLAY=$DISPLAY -e ROS_DOMAIN_ID=85 ros2_humble bash -c "source /opt/ros/humble/setup.bash && rqt_graph"
```

*Pada jendela `rqt_graph`:*
- Pilih dropdown kiri atas: **`Nodes/Topics (all)`**
- Hilangkan centang **`Debug`** (dan centang **`Leaf topics`** bila ingin lebih ringkas)
- Klik tombol **Refresh (↻)** agar ke-6 node (`/avoid_node`, `/wander_node`, `/explore_node`, `/inhibitor_wander`, `/suppressor_l1_l2`, `/suppressor_safety`) tampil rapi.

---

## Part B — English Video Narration Script (~4 Minutes)

### [0:00 – 0:35] Scene 1: Introduction & System Overview
*(Visual: Show Gazebo `turtlebot3_world` + project folder or architecture diagram)*

> "Hello everyone. My name is Anak Agung Ngurah Agung Kresna Ananta, NRP 5024241085, from Computer Engineering, ITS.
>
> In this video, I will demonstrate my implementation of a three-layer **Subsumption Architecture** on a **TurtleBot3 Burger** simulated in **ROS 2 Humble** and **Gazebo**.
>
> This implementation adapts the paper *'Subsumption Model Implemented on ROS for Mobile Robots'* by Minglong Li and colleagues, which originally reproduces Rodney Brooks' autonomous wander robot using loosely coupled ROS nodes, Finite State Machines, and reusable **Inhibitor** and **Suppressor** templates.
>
> Instead of combining everything into one monolithic controller, every behavior layer and every coordination unit is built as an independent ROS 2 node communicating asynchronously via publish-subscribe topics, using the TurtleBot3's 360-degree 2D LiDAR on `/scan` and velocity commands on `/cmd_vel`."

---

### [0:35 – 1:25] Scene 2: Level 0 — Avoid Contact (Survival Reflex)
*(Visual: Run `ros2 launch turtlebot3_subsumption level0_avoid.launch.py` in Terminal 2. Show robot staying completely still first, then place a box / move robot close to a pillar in Gazebo.)*

> "Let's test the architecture incrementally, starting with **Level 0: Avoid Contact**, which is the lowest survival reflex layer.
>
> Right now, only `avoid_node` is running, publishing directly to `/cmd_vel`. As you can see, because the robot spawns in open space with no obstacles inside its 0.32-meter danger threshold, its Finite State Machine stays in `IDLE_SAFE`, so the robot remains completely stationary.
>
> Now, watch what happens when an obstacle gets too close."
>
> *(Place a box close to the TurtleBot3 or drag the robot near a pillar in Gazebo.)*
>
> "Immediately, `avoid_node` detects the obstacle, computes a virtual repulsive force inspired by Brooks' `feelforce` module, and transitions into `EMERGENCY_COLLIDE` and `RUNAWAY`. The robot backs away and turns toward open space. Once its minimum clearance exceeds the 0.37-meter hysteresis threshold, it sends a single zero-velocity stop command and returns to `IDLE_SAFE`, waiting peacefully again—with zero collisions."

---

### [1:25 – 2:15] Scene 3: Level 0 + 1 — Adding Wander & Safety Suppressor
*(Visual: Press `Ctrl+C`, press `Ctrl+R` in Gazebo, then run `ros2 launch turtlebot3_subsumption level1_wander.launch.py`. Watch robot wander randomly and deflect off obstacles; point briefly to Terminal 2 logs showing `Suppressing "/wander/cmd_vel" with "/avoid/cmd_vel"`.)*

> "Next, we stack **Layer 1: Wander** on top of Layer 0 by launching `level1_wander.launch.py`.
>
> This adds two new nodes: `wander_node`, which generates periodic random turns and forward cruising commands on `/wander/cmd_vel`, and `suppressor_safety`—an instance of our reusable Suppressor template.
>
> Normally, `wander_node`'s commands pass through `suppressor_safety` straight to `/cmd_vel`, so the robot roams aimlessly around the arena. Notice that `wander_node` itself does not even subscribe to `/scan`—it has zero awareness of obstacles.
>
> However, look at what happens whenever the robot approaches a pillar or a wall. `avoid_node` publishes an escape command to `/avoid/cmd_vel`. In the terminal log, you can see `suppressor_safety` immediately suppressing `/wander/cmd_vel` and replacing it with `/avoid/cmd_vel` for its 0.5-second timespan. Once the robot is safely clear, the suppressor releases control and random wandering resumes automatically."

---

### [2:15 – 3:15] Scene 4: Level 0 + 1 + 2 — Full Subsumption (Adding Explore, Inhibitor & Suppressor)
*(Visual: Press `Ctrl+C`, press `Ctrl+R` in Gazebo, then run `ros2 launch turtlebot3_subsumption level2_explore.launch.py`. Watch robot lock onto the long open corridor and glide across the arena!)*

> "Now, let's launch the full three-layer architecture with `level2_explore.launch.py`.
>
> This adds **Layer 2: Explore** (`explore_node`) along with two more coordination nodes: `inhibitor_wander` and `suppressor_l1_l2`.
>
> `explore_node` continuously analyzes 360-degree LiDAR scans across overlapping 32-degree angular sectors—mapping `inf` readings to maximum sensor range so open corridors score highest, while using a lower-percentile term to avoid thin pillars.
>
> When `explore_node` detects a clear open corridor, it enters `LOOK_SCAN` and sends a trigger to `/explore/inhibit`. Our **Inhibitor node** (`inhibitor_wander`) compares the timestamps of incoming messages and blocks `/wander/cmd_vel` for 1.5 seconds so random turns cannot disturb the robot. Then, `explore_node` publishes smooth steering commands on `/explore/cmd_vel`, which **Suppressor 1** (`suppressor_l1_l2`) forwards to replace Wander.
>
> Compare this motion to Level 1: instead of meandering randomly on one side of the map, the robot decisively locks onto the long open lane between the pillars, drives straight across the arena, and transitions cleanly into the perpendicular corridor. And whenever it passes close to a wall or pillar, Layer 0 still overrides via `suppressor_safety` to guarantee collision-free navigation."

---

### [3:15 – 3:55] Scene 5: `rqt_graph` & Quantitative Comparison
*(Visual: Switch to `rqt_graph` window showing all 6 modular nodes, then briefly show `results/trajectory_comparison.png`.)*

> "To verify true modularity, here is the live `rqt_graph` for Level 2. You can clearly see all six independent nodes: `avoid_node`, `wander_node`, `explore_node`, `inhibitor_wander`, `suppressor_l1_l2`, and `suppressor_safety`, connected asynchronously through their dedicated topics right into Gazebo's `/cmd_vel`.
>
> Finally, looking at our recorded 2D trajectory comparison from identical 55-second runs in `turtlebot3_world`:
> - Under **Level 0**, the robot stays stationary in open space and immediately escapes when placed near a pillar.
> - Under **Level 0 + 1**, it wanders 5.42 meters covering 2.11 square meters with zero collisions.
> - Under **Level 0 + 1 + 2**, the full subsumption robot travels **7.34 meters** and covers **4.69 square meters**—more than **double the explored area** of Level 1—while maintaining a safe minimum clearance of 0.26 meters throughout.
>
> That concludes my demonstration of the ROS 2 Subsumption Architecture on TurtleBot3. Thank you for watching!"
