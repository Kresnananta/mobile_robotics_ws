# Name: Anak Agung Ngurah Agung Kresna Ananta
# NRP: 5024241085
# Mobile Programing (Lab 1)

### Question 1
* **Subscribers (2 total):**
  * `/parameter_events` (`rcl_interfaces/msg/ParameterEvent`)
  * `/turtle1/cmd_vel` (`geometry_msgs/msg/Twist`)
* **Publishers (4 total):**
  * `/parameter_events` (`rcl_interfaces/msg/ParameterEvent`)
  * `/rosout` (`rcl_interfaces/msg/Log`)
  * `/turtle1/color_sensor` (`turtlesim/msg/Color`)
  * `/turtle1/pose` (`turtlesim/msg/Pose`)
* **Service Servers (13 total):**
  * 7 simulator services: `/clear`, `/kill`, `/reset`, `/spawn`, `/turtle1/set_pen`, `/turtle1/teleport_absolute`, `/turtle1/teleport_relative`
  * 6 parameter management services: `/turtlesim/describe_parameters`, `/turtlesim/get_parameter_types`, `/turtlesim/get_parameters`, `/turtlesim/list_parameters`, `/turtlesim/set_parameters`, `/turtlesim/set_parameters_atomically`

---

### Question 2
* `/turtle1/pose` publishes continuously at a fixed rate of approximately 62.5 Hz (around 60 Hz) as the simulation physics loop updates.
* `/turtle1/cmd_vel` has no steady rate because it is an asynchronous, event-driven topic that only publishes messages when user input or an explicit publish event occurs.

---

### Question 3
* The two controllable values are forward linear velocity (`linear.x`) and yaw angular velocity (`angular.z`).
* `linear.y` is ruled out by the non-holonomic kinematic constraint of a differential-drive base. The standard fixed wheels can only roll forwards or backwards along their heading and cannot slip or slide laterally without losing traction.

---

### Question 4
* `/turtlesim` only knew that an anonymous subscriber matching the topic name, message type, and Quality of Service (QoS) profile was present on the DDS middleware layer.
* When the subscriber exited, `/turtlesim` did not have to alter its execution state or logic at all. The underlying DDS discovery protocol deregistered the endpoint, and `/turtlesim` continued publishing periodic updates into the transport layer as before.

---

### Question 5
* An action provides continuous, intermediate feedback during task execution and the ability to preemptively cancel an active goal, neither of which is supported by a standard service.
* For the command "navigate to the kitchen", you should use an **Action**. Navigation is a long-duration, non-instantaneous behavior that requires ongoing progress tracking (feedback) and the ability to abort safely if an obstacle or emergency arises.

---

### Question 6
* The changes took effect immediately because the package was built using `--symlink-install`, which links the Python scripts directly from the source directory to the install space. Because Python is an interpreted language, the runtime interpreter executes the modified source code directly on the next run without recompilation.
* If `lab1_turtle` were an `ament_cmake` C++ package, editing the `.cpp` file would have had no effect because C++ must be compiled into binary machine code. The older compiled binary in `install/` would continue executing until rebuilt using `colcon build`.

---

### Question 7
* **Cause 1:** The workspace overlay environment has not been sourced in the new terminal session.
  * *Fix:* Run `source install/setup.bash` from the workspace root (`~/mobile_robotics_ws`).
* **Cause 2:** The executable entry point was missing or mistyped in `setup.py`, or the workspace was not built after declaring it.
  * *Fix:* Ensure `'circle_driver = lab1_turtle.circle_driver:main'` is correctly declared under `console_scripts` in `setup.py`, then run `colcon build --symlink-install --packages-select lab1_turtle` and re-source the terminal.
