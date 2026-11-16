# Mobile Robotics Workspace (`mobile_robotics_ws`)

ROS 2 Humble workspace containing mobile robotics laboratory exercises and reactive path planning assignments using TurtleSim and TurtleBot3 in Gazebo Classic.

- **Author:** Anak Agung Ngurah Agung Kresna Ananta
- **NRP:** 5024241085
- **Major** Computer Engineering
- **Environment:** Ubuntu 22.04 / 24.04 LTS (Docker), ROS 2 Humble, Gazebo Classic 11, RViz2

## Quick Navigation

| Module | Folder | ROS 2 Package | Documentation & Outputs |
| :--- | :--- | :--- | :--- |
| **Lab 1 — TurtleSim Publisher** | [`lab1/`](lab1/) | [`lab1/src/lab1_turtle/`](lab1/src/lab1_turtle/) | [Answers](lab1/lab1_answers.md) · [Graph](lab1/lab1_rqt_graph.png) · [Sources](lab1/SOURCES.md) |
| **Assignment 2 — Bug 2 Navigation** | [`assignment2/`](assignment2/) | [`assignment2/src/a2_worlds/`](assignment2/src/a2_worlds/) | [README & Videos](assignment2/README.md) · [CSV Results](assignment2/results/) · [Screenshots](assignment2/screenshot/) · [Report](assignment2/a2_report.pdf) · [Sources](assignment2/SOURCES.md) |

## Repository Structure

```text
mobile_robotics_ws/
├── .gitignore
├── README.md
├── lab1/
│   ├── src/lab1_turtle/         # ROS 2 package for TurtleSim circular motion controller
│   ├── lab1_answers.md          # Lab 1 theoretical and node graph analysis
│   ├── lab1_rqt_graph.png       # ROS 2 computation graph snapshot
│   └── SOURCES.md               # References and acknowledgments for Lab 1
└── assignment2/
    ├── src/a2_worlds/           # ROS 2 package implementing Bug 2 reactive navigation
    ├── results/                 # Raw CSV trajectory logs and summary.csv across 5 runs
    ├── screenshot/              # RViz2 trajectory & marker visualizations (w1, w2, w3)
    ├── a2_report.pdf            # 2-page technical analysis report (Q1 & Q2)
    ├── README.md                # Run configurations, metrics, screenshots, and demo links
    └── SOURCES.md               # References and acknowledgments for Assignment 2