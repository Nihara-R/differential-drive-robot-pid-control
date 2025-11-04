Differential Drive Robot Simulation

📘 Overview

Python simulation of a differential drive robot with P, PD, and PID control. Interactive Pygame visualization with real-time parameter tuning and trajectory tracking. Compare control strategies for autonomous navigation. Built for IE4060 Robotics @ SLIIT.

The simulation demonstrates:

Closed-loop control for distance and heading.

The effects of proportional, derivative, and integral gains on motion behavior.

Interactive tuning of control parameters.

Visual comparison between P, PD, and PID trajectories.

⚙️ Features

✅ Interactive GUI using Pygame
✅ Real-time robot motion and trajectory visualization
✅ Switch between P, PD, and PID controllers instantly
✅ Adjustable controller parameters during simulation
✅ Anti-windup mechanism for integral control
✅ Color-coded trajectory trails for each controller:

🔴 Red → P controller

🟢 Green → PD controller

🔵 Blue → PID controller

🧩 Requirements

Install the required dependencies using:

pip install pygame


(Python’s built-in libraries like math, sys, and collections are already included.)

🚀 How to Run

Download or clone this repository.

Open a terminal in the project folder.

Run the script:

python assignment_ddr.py


A Pygame simulation window will open.

