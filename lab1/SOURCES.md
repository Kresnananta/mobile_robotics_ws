# Sources & Acknowledgments

## 1. Course Handouts & Materials
* **Lab Handout:** *Lab 1: Introduction to the Robot Operating System (ROS 2)* by Muhtadin, S.T., M.T., Departemen Teknik Komputer ITS.
  * *Contribution:* Provided instructions, structural boilerplate for `circle_driver.py`, build commands, and experiment workflows.

## 2. AI Assistance
* **Google Gemini:**
  * *Contribution:*
    * Diagnosed indentation errors in `circle_driver.py` (placing `main()` at the module root rather than inside `CircleDriver`).
    * Troubleshot the `"Package not found"` error by clarifying that `source install/setup.bash` is scoped per terminal session.
    * Resolved Git push failures related to local `master` branch mismatch and GitHub Personal Access Token (PAT) authentication.

## 3. Official Documentation
* **ROS 2 Humble Documentation** (https://docs.ros.org/en/humble/Tutorials.html)[cite: 1]:
  * *Contribution:* Reference for CLI tools (`ros2 run`, `ros2 topic`, `ros2 node`, `ros2 param`), node anatomy in `rclpy`, and colcon workspace setup.
