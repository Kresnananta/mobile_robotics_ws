# a2_worlds — Assignment 2 (Bug 2)

Three Gazebo worlds, one launch file, and **one file you edit**:
`a2_worlds/bug2.py`. Everything else is given and must stay untouched.

TurtleBot3 **Burger** only.

## Build and run

```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

cd ~/mobile_robotics_ws
colcon build --packages-select a2_worlds
source install/setup.bash

# terminal 1 -- y_pose MUST equal the Y0 you set in bug2.py
ros2 launch a2_worlds bug_world.launch.py world:=bug_1_circles y_pose:=0.10

# terminal 2
ros2 run a2_worlds bug2
```

Worlds: `bug_1_circles`, `bug_2_boxes`, `bug_3_concave`. Add `gui:=false` to run
headless. Start is `(0, y0)`, goal is `(10, 0)`, marked by a green disc that is
**visual only** — the laser cannot see it.

**If the world appears but the robot does not**, and the log says
`Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?`,
then Gazebo was simply slower to start than the spawner was willing to wait.
The plugin is fine. Give it more time:

```bash
ros2 launch a2_worlds bug_world.launch.py world:=bug_1_circles y_pose:=0.10 spawn_delay:=25
```

The fallback always works too: leave the world running and spawn the robot
yourself from another terminal.

```bash
ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=0.0 y_pose:=0.10
```

## The one file you edit

Open `a2_worlds/bug2.py`. Five constants at the top are the only settings you
ever change:

```python
Y0       = 0.10     # 0.10 * (NRP % 5) - 0.20   -- must match y_pose
MODE     = 'test'   # 'test' = go_and_stop (given) | 'bug2' = yours
RUN_NAME = 'w1'     # names the CSV for this run
HAND     = +1       # contour following: +1 wall on your left, -1 on your right
DELTA    = 0.25     # progress margin; -1e6 for the naive run
```

The main loop calls **exactly one** of two driving functions, chosen by `MODE`:

| Function | Status | What it does |
|---|---|---|
| `go_and_stop()` | **given, complete** | drives at the goal, stops at the first obstacle, prints the surface it found and the hit point `H` that implies, next to the reference numbers |
| `bug2()` | **yours** | four TODO blocks in one function |

Both return `(v, w)` — you never touch `Twist`.

**Run `MODE = 'test'` first.** It proves the world, the robot, the laser and
the odometry all work, and it shows you the `H` arithmetic you need in TODO 2.

Of the ~400 lines in that file you need to read exactly two things: the five
constants, and the function `bug2()`. The subscriptions, the 20 Hz loop, the
scan filtering, the m-line arithmetic, the CSV writer and the RViz markers are
written for you.

Any change to a constant needs `colcon build --packages-select a2_worlds` again.

## Output

Each run writes `~/a2_results/<RUN_NAME>.csv` (one row per control tick) and
appends one row to `~/a2_results/summary.csv`. That summary file is the table
your report needs — do not retype the numbers by hand.

For the RViz screenshots: set **Fixed Frame** to `odom`, then
*Add → By topic → /a2_markers → MarkerArray*, and add `/scan` as a LaserScan.

## Known noise you should ignore

Opening a world with the Gazebo GUI prints

```
[Err] [InsertModelWidget.cc:403] Missing model.config for model
      ".../turtlebot3_gazebo/models/turtlebot3_autorace_2020"
```

That comes from the Gazebo GUI scanning `GAZEBO_MODEL_PATH` to fill its
*Insert* tab, not from these worlds. It appears with any world, including the
stock TurtleBot3 ones. Ignore it.
