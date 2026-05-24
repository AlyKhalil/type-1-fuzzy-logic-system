# Fuzzy Logic System

A Python fuzzy logic framework with a complete LiDAR-based autonomous robot navigation example using ROS 2.

## Overview

This project provides reusable fuzzy logic components — fuzzification, inference, and defuzzification — alongside a practical robot controller that demonstrates multiple intelligent control strategies:

- Obstacle avoidance
- Wall/edge following
- Subsumption control
- Context blending
- PID control

## Repository Structure

```
fuzzy-logic-system/
├── fuzzy.py                    # Core fuzzy logic engine
├── common.py                   # Membership function library
└── LiDAR_robot_example_code.py # ROS 2 robot controller application
```

## Dependencies

```bash
pip install numpy matplotlib
```

ROS 2 is required only for the robot controller example:

```bash
# Follow the official ROS 2 installation guide for your platform
# https://docs.ros.org/en/rolling/Installation.html
```

## Core Modules

### `common.py` — Membership Functions

Provides standard fuzzy membership function shapes:

| Function | Signature | Description |
|---|---|---|
| `triangular` | `(a, b, c)` | Triangular shape |
| `trapezoid` | `(a, b, c, d)` | Trapezoidal shape |
| `gaussian` | `(a, theta)` | Gaussian/bell curve |
| `plot` | `(fn, range, label)` | Plot any membership function |

### `fuzzy.py` — Fuzzy Logic Engine

Three classes that implement the full fuzzy inference pipeline:

**`Fuzzifier`** — converts crisp sensor readings into fuzzy membership degrees
- `calc_membership_values()` — compute membership degrees for each linguistic label
- `plot_membership_functions()` — visualise membership functions; optionally mark a crisp input

**`Inference`** — Mamdani-style rule evaluation
- `compute_firing_strengths()` — apply min-operator to evaluate all rules
- `print_rule_evaluation()` — print a formatted rule-firing summary

**`Defuzzifier`** — converts fuzzy outputs back to a crisp value
- `defuzzify()` — weighted average of output peaks based on firing strengths

### `LiDAR_robot_example_code.py` — Robot Controller

A ROS 2 node that reads a `/scan` LaserScan topic and publishes velocity commands to `/cmd_vel`.

**LiDAR input regions:**

| Region | Purpose |
|---|---|
| Front | Obstacle detection |
| Front-Left / Front-Right | Directional obstacle avoidance |
| Right-Front / Right-Back | Wall following |
| Left / Right | General spatial awareness |

**Fuzzy rule sets:**
- Obstacle avoidance — 27 rules over `{close, medium, far}` inputs
- Right edge following — 9 rules over right-front and right-back distances

**Output variables:**
- Linear velocity — `slow / moderate / fast` (0.05–0.5 m/s)
- Angular velocity — `left / straight / right` (−0.65 to +0.65 rad/s)

**Control modes** (toggled via flags in the script):

| Mode | Description |
|---|---|
| Obstacle avoidance | Pure fuzzy obstacle avoidance |
| Right edge following | Fuzzy wall-following |
| Subsumption | Obstacle avoidance takes priority over wall following |
| Context blending | Weighted combination of both behaviours |
| PID | Maintains a desired distance from the wall (Kp=0.7, Ki=0.002, Kd=20) |

**Safety:** an emergency stop triggers if any obstacle is detected within 0.2 m.

## Usage

### Using the fuzzy logic library

```python
import numpy as np
from common import triangular, trapezoid
from fuzzy import Fuzzifier, Inference, Defuzzifier

# Define membership functions for an input variable
mf = {
    "close":  triangular(0.0, 0.0, 0.5),
    "medium": triangular(0.25, 0.5, 0.75),
    "far":    triangular(0.5, 1.0, 1.0),
}

# Fuzzify a crisp sensor reading
fuzzifier = Fuzzifier(mf)
memberships = fuzzifier.calc_membership_values(crisp_value=0.3)

# Evaluate rules and defuzzify
# (see LiDAR_robot_example_code.py for a complete worked example)
```

### Running the robot controller

```bash
# Source your ROS 2 workspace, then:
python3 LiDAR_robot_example_code.py
```

The node expects:
- `/scan` — `sensor_msgs/LaserScan`
- `/cmd_vel` — `geometry_msgs/Twist` (published)
