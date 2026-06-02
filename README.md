# Type-1 Fuzzy Logic System

A Python library implementing the full Type-1 Mamdani fuzzy inference pipeline — fuzzification, rule evaluation, and defuzzification — with a complete worked example: a ROS 2 node that reads a 2D LiDAR scan and drives a robot using fuzzy logic for obstacle avoidance and right-wall following.

## Architecture

```
membership_functions.py       # Membership function factories (triangular, trapezoid, gaussian)
fuzzy.py                      # Fuzzifier → Inference → Defuzzifier pipeline
LiDAR_robot_example_code.py  # ROS 2 robot controller built on the above
```

The library is framework-agnostic. `fuzzy.py` takes any callables as membership functions and any dict of `Fuzzifier` objects as inputs — no ROS 2 dependency.

## Dependencies

```bash
pip install numpy matplotlib
```

ROS 2 is required only for the robot controller example. See the [official installation guide](https://docs.ros.org/en/rolling/Installation.html).

## Core modules

### `membership_functions.py`

Factories that return a callable `membership(x)` accepting scalars or NumPy arrays:

| Function | Signature | Description |
|---|---|---|
| `triangular` | `(a, b, c)` | Triangular shape with peak at `b` |
| `trapezoid` | `(a, b, c, d)` | Trapezoidal shape; handles degenerate edges (`a==b`, `c==d`) |
| `gaussian` | `(a, theta)` | Gaussian bell curve centred at `a` |
| `plot` | `(fn, x_range, num_points, title)` | Plot any membership function |

### `fuzzy.py`

Three classes that implement the Mamdani inference pipeline:

**`Fuzzifier`** — converts a crisp sensor reading into a dict of membership degrees, one per linguistic label.
- `calc_membership_values(crisp_input)` — evaluate all membership functions at `crisp_input`; result stored in `self.membership_values`
- `plot_membership_functions(x_range, num_points)` — visualise all functions; marks `crisp_input` if set

**`Inference`** — evaluates a rule base against a set of fuzzified inputs.
- Rules are tuples of `(variable_name, label)` pairs; antecedents come first, consequents last
- `compute_firing_strengths()` — applies the AND (min) operator across each rule's antecedents; returns `[(rule, strength), ...]`
- `print_rule_evaluation()` — formatted rule-by-rule summary

**`Defuzzifier`** — produces crisp output values from firing strengths using the centre-of-singletons method (weighted average of output peaks).
- `defuzzify()` — returns a dict of crisp output values; returns `0.0` for any output whose total firing strength is zero

## Usage

```python
import membership_functions as mf
from fuzzy import Fuzzifier, Inference, Defuzzifier

# Define membership functions for an input variable
distance_fuzzy_set = [
    ('close',  mf.trapezoid(0.0, 0.0, 0.2, 0.4)),
    ('medium', mf.triangular(0.2, 0.4, 0.6)),
    ('far',    mf.trapezoid(0.4, 1.2, 15.0, 15.0)),
]

# Fuzzify a crisp sensor reading
fuzzifier = Fuzzifier(distance_fuzzy_set)
memberships = fuzzifier.calc_membership_values(crisp_input=0.3)
# {'close': 0.5, 'medium': 0.5, 'far': 0.0}

# Define rules: (antecedent pairs..., consequent pairs...)
rules = [
    (('distance', 'close'),  ('speed', 'slow')),
    (('distance', 'medium'), ('speed', 'moderate')),
    (('distance', 'far'),    ('speed', 'fast')),
]

inference = Inference({'distance': fuzzifier}, rules)
firing_strengths = inference.compute_firing_strengths()

# Output peak values (centre of each output singleton)
output_peaks = {
    'speed': {'slow': 0.1, 'moderate': 0.3, 'fast': 0.5}
}

crisp_output = Defuzzifier(firing_strengths, output_peaks).defuzzify()
# {'speed': 0.2}
```

## Robot controller example

`LiDAR_robot_example_code.py` is a ROS 2 node that subscribes to `/scan` (`sensor_msgs/LaserScan`) and publishes to `/cmd_vel` (`geometry_msgs/Twist`).

**LiDAR regions used:**

| Key | Sector |
|---|---|
| `front` | Forward obstacle detection |
| `front_left` / `front_right` | Directional obstacle avoidance |
| `right_front` / `right_back` | Right-wall following |
| `left` / `right` | General spatial awareness |

**Fuzzy rule sets:**
- Obstacle avoidance — 27 rules over `{close, medium, far}` × 3 inputs
- Right-wall following — 9 rules over right-front and right-back distances

**Output variables:**
- `speed` — `slow / moderate / fast` mapped to 0.05–0.5 m/s
- `direction` — `left / straight / right` mapped to ±0.65 rad/s

**Control modes** (set via flags at the top of the script):

| Flag | Mode |
|---|---|
| `OA_ONLY` | Pure fuzzy obstacle avoidance |
| `RF_ONLY` | Pure fuzzy right-wall following |
| `PID_FLAG` | PID right-wall following (Kp=0.7, Ki=0.002, Kd=20) |
| `CONTEXT_BLENDING_FLAG` | Weighted blend of obstacle avoidance and wall following |
| all `False` | Subsumption: obstacle avoidance takes priority, wall following otherwise |

Emergency stop activates at < 0.2 m on any front-facing sensor.

```bash
# Source your ROS 2 workspace, then:
python3 LiDAR_robot_example_code.py
```
