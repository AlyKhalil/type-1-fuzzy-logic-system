#!/usr/bin/env python3
import rclpy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

import common as cmn
from fuzzy import Fuzzifier, Inference, Defuzzifier

# --- Constants ---
EMERGENCY_STOP_DISTANCE = 0.2
OBSTACLE_AVOIDANCE_DISTANCE = 0.5 # used in subsumption  

# --- PID Constants ---
KP = 0.7
KI = 0.002 # Bigger number quicker turns
KD = 20 # Bigger number, stronger smoothing

#---PID State---
ei = 0.0
e_previous = 0

# --- global Variables ---
mynode_ = None
pub_ = None
regions_ = {
    'left': 0,
    'right': 0,
    'fLeft': 0,
    'fRight': 0,
    'front1': 0,
    'front2': 0,
}
twstmsg_ = None
count = 0

#---Right Edge Following---

# Inputs fuzzy sets
right_front_fuzzy_set = [
    ('close', cmn.trapezoid(0, 0, 0.2, 0.4)),
    ('medium', cmn.triangular(0.2, 0.4, 0.6)),
    ('far', cmn.trapezoid(0.4, 1.2, 15.0, 15.0))
]

right_back_fuzzy_set = [
    ('close', cmn.trapezoid(0, 0, 0.3, 0.4)),
    ('medium', cmn.triangular(0.2, 0.4, 0.6)),
    ('far', cmn.trapezoid(0.4, 1.2, 15.0, 15.0))
]


right_front_fuzzifier = Fuzzifier(right_front_fuzzy_set)
right_back_fuzzifier = Fuzzifier(right_back_fuzzy_set)

# Rule base
right_following_rule_base = [ 
    ( 
        ('right front', 'close'), ('right back', 'close'), ('speed', 'slow'), ('direction', 'left')
    ),

    ( 
        ('right front', 'close'), ('right back', 'medium'), ('speed', 'slow'), ('direction', 'left')
    ),

    ( 
        ('right front', 'close'), ('right back', 'far'), ('speed', 'slow'), ('direction', 'left') 
    ), 
    
    ( 
        ('right front', 'medium'), ('right back', 'close'), ('speed', 'moderate'), ('direction', 'right') 
    ), 
    
    ( 
        ('right front', 'medium'), ('right back', 'medium'), ('speed', 'moderate'), ('direction', 'straight') 
    ), 
    
    ( 
        ('right front', 'medium'), ('right back', 'far'), ('speed', 'moderate'), ('direction', 'left') 
    ),
    
    ( 
        ('right front', 'far'), ('right back', 'close'), ('speed', 'moderate'), ('direction', 'right') 
    ),
        
    ( 
        ('right front', 'far'), ('right back', 'medium'), ('speed', 'moderate'), ('direction', 'right')
    ),

    # Encountering an edge case, therefore speed=slow and direction=right
    (
        ('right front', 'far'), ('right back', 'far'), ('speed', 'slow'), ('direction', 'right')
    )
]

right_edge_inference = Inference(
    {'right front': right_front_fuzzifier, 'right back': right_back_fuzzifier},
    right_following_rule_base
)

# --- Obstacle Avoidance ---

front_fuzzy_set = [
    ('close', cmn.trapezoid(0, 0, 0.2, 0.4)),
    ('medium', cmn.triangular(0.2, 0.4, 0.6)),
    ('far', cmn.trapezoid(0.4, 1.2, 15, 15))
]

front_left_fuzzy_set = [
    ('close', cmn.trapezoid(0, 0, 0.2, 0.4)),
    ('medium', cmn.triangular(0.2, 0.4, 0.6)),
    ('far', cmn.trapezoid(0.4, 1.2, 15, 15))
]

front_right_fuzzy_set = [
    ('close', cmn.trapezoid(0, 0, 0.2, 0.4)),
    ('medium', cmn.triangular(0.2, 0.4, 0.6)),
    ('far', cmn.trapezoid(0.4, 1.2, 15, 15))
]

front_fuzzifier = Fuzzifier(front_fuzzy_set)
front_left_fuzzifier = Fuzzifier(front_left_fuzzy_set)
front_right_fuzzifier = Fuzzifier(front_right_fuzzy_set)

obstacle_avoidance_rule_base = [
    (
        ('front', 'close'), ('front left', 'close'), ('front right', 'close'), ('speed', 'slow'), ('direction', 'left')
    ),
    (
        ('front', 'close'), ('front left', 'close'), ('front right', 'medium'), ('speed', 'slow'), ('direction', 'right')
    ),
    (
        ('front', 'close'), ('front left', 'medium'), ('front right', 'close'), ('speed', 'slow'), ('direction', 'left')
    ),
    (
        ('front', 'close'), ('front left', 'medium'), ('front right', 'medium'), ('speed', 'slow'), ('direction', 'left')
    ),
    (
        ('front', 'close'), ('front left', 'close'), ('front right', 'far'), ('speed', 'slow'), ('direction', 'right')
    ),
    (
        ('front', 'close'), ('front left', 'far'), ('front right', 'close'), ('speed', 'slow'), ('direction', 'left')
    ),
    (
        ('front', 'close'), ('front left', 'medium'), ('front right', 'far'), ('speed', 'slow'), ('direction', 'right')
    ),
    (
        ('front', 'close'), ('front left', 'far'), ('front right', 'medium'), ('speed', 'slow'), ('direction', 'left')
    ),
    (
        ('front', 'close'), ('front left', 'far'), ('front right', 'far'), ('speed', 'slow'), ('direction', 'left')
    ),
    
    
    (
        ('front', 'medium'), ('front left', 'close'), ('front right', 'close'), ('speed', 'slow'), ('direction', 'straight')
    ),
    (
        ('front', 'medium'), ('front left', 'close'), ('front right', 'medium'), ('speed', 'slow'), ('direction', 'right')
    ),
    (
        ('front', 'medium'), ('front left', 'medium'), ('front right', 'close'), ('speed', 'slow'), ('direction', 'left')
    ),
    (
        ('front', 'medium'), ('front left', 'medium'), ('front right', 'medium'), ('speed', 'moderate'), ('direction', 'straight')
    ),
    (
        ('front', 'medium'), ('front left', 'close'), ('front right', 'far'), ('speed', 'slow'), ('direction', 'right')
    ),
    (
        ('front', 'medium'), ('front left', 'far'), ('front right', 'close'), ('speed', 'slow'), ('direction', 'left')
    ),
    (
        ('front', 'medium'), ('front left', 'medium'), ('front right', 'far'), ('speed', 'moderate'), ('direction', 'right')
    ),
    (
        ('front', 'medium'), ('front left', 'far'), ('front right', 'medium'), ('speed', 'moderate'), ('direction', 'left')
    ),
    (
        ('front', 'medium'), ('front left', 'far'), ('front right', 'far'), ('speed', 'moderate'), ('direction', 'left')
    ),


    (
        ('front', 'far'), ('front left', 'close'), ('front right', 'close'), ('speed', 'slow'), ('direction', 'straight')
    ),
    (
        ('front', 'far'), ('front left', 'close'), ('front right', 'medium'), ('speed', 'slow'), ('direction', 'right')
    ),
    (
        ('front', 'far'), ('front left', 'medium'), ('front right', 'close'), ('speed', 'slow'), ('direction', 'left')
    ),
    (
        ('front', 'far'), ('front left', 'medium'), ('front right', 'medium'), ('speed', 'moderate'), ('direction', 'straight')
    ),
    (
        ('front', 'far'), ('front left', 'close'), ('front right', 'far'), ('speed', 'slow'), ('direction', 'right')
    ),
    (
        ('front', 'far'), ('front left', 'far'), ('front right', 'close'), ('speed', 'slow'), ('direction', 'left')
    ),
    (
        ('front', 'far'), ('front left', 'medium'), ('front right', 'far'), ('speed', 'moderate'), ('direction', 'right')
    ),
    (
        ('front', 'far'), ('front left', 'far'), ('front right', 'medium'), ('speed', 'moderate'), ('direction', 'left')
    ),
    (
        ('front', 'far'), ('front left', 'far'), ('front right', 'far'), ('speed', 'moderate'), ('direction', 'straight')
    )
]

obstacle_avoidance_inference = Inference(
    {'front': front_fuzzifier, 'front left': front_left_fuzzifier, 'front right': front_right_fuzzifier},
    obstacle_avoidance_rule_base
)

# --- Outputs peaks ---
output_peaks_RF = {
    'speed': {
        'slow': 0.1,
        'moderate': 0.3,
        'fast': 0.5
    },
    'direction': {
        'left': 0.25,
        'straight': 0.0,
        'right': -0.25
    }
}

output_peaks_OA = {
    'speed': {
        'slow': 0.05,
        'moderate': 0.3,
        'fast': 0.5
    },
    'direction': {
        'left': 0.65,
        'straight': 0.0,
        'right': -0.65
    }
}

# --- Context Blending ---

PRIORITY_FACTOR = 2.0

# Obstacle Avoidence
context_blending_fuzzy_set_d1 = [
    ('Near', cmn.trapezoid(0, 0, 0.4, 0.8))
]

# Right Edge Following
context_blending_fuzzy_set_d2 = [
    ('Near', cmn.trapezoid(0, 0, 0.8, 1.5))
]

context_blending_Fuzzifier_d1 = Fuzzifier(context_blending_fuzzy_set_d1)
context_blending_Fuzzifier_d2 = Fuzzifier(context_blending_fuzzy_set_d2)

# =========================================
PID_FLAG = False
CONTEXT_BLENDING_FLAG = False
OA_ONLY = False
RF_ONLY = False
# False - False - False - False triggers subsumption control
# =========================================

def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)

def clbk_laser(msg):
    global regions_, twstmsg_, count

    print("Data Points: ", len(msg.ranges))

    regions_ = {
        #LIDAR readings are anti-clockwise, starting at 0 on the right-most edge of the LiDaR FOV.
        'right':  find_nearest(msg.ranges[21:23]),
        'front': find_nearest(msg.ranges[110:130]),
        'left': find_nearest(msg.ranges[219:221]),
        'right_back': find_nearest(msg.ranges[0:2]),
        'right_front':  find_nearest(msg.ranges[38:41]),
        'front_right': find_nearest(msg.ranges[70:73]),
        'front_left': find_nearest(msg.ranges[169:172]),
    }

    # Log sensor readings for debugging
    print(f"F: {regions_['front']:.2f} | FL: {regions_['front_left']:.2f} | "
          f"FR: {regions_['front_right']:.2f} | RF: {regions_['right_front']:.2f} | "
          f"RB: {regions_['right_back']:.2f}")
    
    if PID_FLAG:
        print('PID')
        twstmsg_ = pid_controller()

    elif CONTEXT_BLENDING_FLAG:
        print('CONTEXT BLENDING')
        twstmsg_ = context_blending()
    
    elif OA_ONLY:
        print('OBSTACLE AVOIDANCE')
        msg = Twist()
        final_crisp_output = avoid_obstacle()
        msg.linear.x = float(final_crisp_output['speed']) if min(regions_['front'], regions_['front_left'], regions_['front_right']) > EMERGENCY_STOP_DISTANCE else 0.02
        msg.angular.z = float(final_crisp_output['direction'])
        print(f"  → Final Speed: {msg.linear.x:.2f}, Final Turn: {msg.angular.z:.2f}")
        twstmsg_ = msg

    elif RF_ONLY:
        print('RIGHT EDGE FOLLOWING')
        msg = Twist()
        final_crisp_output = right_edge_following()
        msg.linear.x = float(final_crisp_output['speed']) if min(regions_['front'], regions_['front_left'], regions_['front_right']) > EMERGENCY_STOP_DISTANCE else 0.02
        msg.angular.z = float(final_crisp_output['direction'])
        print(f"  → Final Speed: {msg.linear.x:.2f}, Final Turn: {msg.angular.z:.2f}")
        twstmsg_ = msg

    else:
        print('SUBSUMPTION CONTROL')
        twstmsg_ = subsumption_control()

def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)
    return min(min(f_list, default=10), 10)

def subsumption_control():
    global regions_

    msg = Twist()
    
    if min(regions_['front'], regions_['front_left'], regions_['front_right']) < OBSTACLE_AVOIDANCE_DISTANCE:
        print('Obstacle Avoidance')
        final_crisp_output = avoid_obstacle()
    else:
        print('Right Edge Following')
        final_crisp_output = right_edge_following()

    msg.linear.x = float(final_crisp_output['speed']) if min(regions_['front'], regions_['front_left'], regions_['front_right']) > EMERGENCY_STOP_DISTANCE else 0.02
    msg.angular.z = float(final_crisp_output['direction'])

    print()
    print(f"  → Speed: {msg.linear.x:.2f}, Turn: {msg.angular.z:.2f}")

    return msg

def context_blending():
    global regions_

    msg = Twist()

    final_crisp_output = {
        'speed': 0.0,
        'direction': 0.0
    }

    d1 = min(regions_['front'], regions_['front_left'], regions_['front_right'])
    d2 = min(regions_['right_front'], regions_['right_back'])

    weight_d1 = context_blending_Fuzzifier_d1.calc_membership_values(d1)['Near'] * PRIORITY_FACTOR
    weight_d2 = context_blending_Fuzzifier_d2.calc_membership_values(d2)['Near'] 

    obstacle_output = avoid_obstacle()

    right_edge_output = right_edge_following()

    # # ==================== DEBUG PRINTS ====================    
    print(f"\n{'='*60}")
    print(f"{'='*60}")
    print(f"\n[CONTEXT WEIGHTS]")
    print(f"  d1 (front dist): {d1:.2f}m → w_obstacle: {weight_d1:.3f}")
    print(f"  d2 (right dist): {d2:.2f}m → w_wall: {weight_d2:.3f}")
    
    print(f"\n[DEFUZZIFIED OUTPUTS]")
    print(f"  Obstacle: speed={obstacle_output['speed']:.3f}, dir={obstacle_output['direction']:.3f}")
    print(f"  Wall:     speed={right_edge_output['speed']:.3f}, dir={right_edge_output['direction']:.3f}")
    print(f"\n{'='*60}")
    print(f"{'='*60}")
    # ==================== END DEBUG =========================

    if weight_d1 + weight_d2 != 0.0:
        final_crisp_output = {
            'speed': (
                float(
                    (weight_d1 * obstacle_output['speed'] + weight_d2 * right_edge_output['speed'])
                    /
                    (weight_d1 + weight_d2)
                )
            ),

            'direction':(
                float(
                    (weight_d1 * obstacle_output['direction'] + weight_d2 * right_edge_output['direction'])
                    /
                    (weight_d1 + weight_d2)   
                )
            )
        }
    else:
        # Slow right movement when no context is detected
        print('NO CONTEXT DETECTED')
        final_crisp_output = {
            'speed': 0.2,
            'direction': -0.1
        }

    msg.linear.x = float(final_crisp_output['speed']) if d1 > EMERGENCY_STOP_DISTANCE else 0.02
    msg.angular.z = float(final_crisp_output['direction'])

    print(f"  → Final Speed: {msg.linear.x:.2f}, Final Turn: {msg.angular.z:.2f}")

    return msg

def right_edge_following():
    global regions_

    right_front_fuzzifier.calc_membership_values(regions_['right_front'])
    right_back_fuzzifier.calc_membership_values(regions_['right_back'])

    firing_strengths = right_edge_inference.compute_firing_strengths()

    defuzzifier = Defuzzifier(firing_strengths, output_peaks_RF)
    crisp_output = defuzzifier.defuzzify()

    # ==================== DEBUG PRINTS ====================
    print(f"\n{'='*60}")
    print(f"RF MEMBERSHIP VALUES DEBUG")
    print(f"{'='*60}")
    
    # Right Edge Following memberships
    print(f"\n[RIGHT EDGE FOLLOWING]")
    print(f"Right Front ({regions_['right_front']:.2f}m):")
    for label, val in right_front_fuzzifier.membership_values.items():
        print(f"  {label:8s}: {val:.3f} {'█' * int(val * 20)}")
    
    print(f"Right Back ({regions_['right_back']:.2f}m):")
    for label, val in right_back_fuzzifier.membership_values.items():
        print(f"  {label:8s}: {val:.3f} {'█' * int(val * 20)}")


    # Rule firing strengths
    print(f"\n[FIRING STRENGTHS]")
    print(f"Right Edge Following - Top 3 Rules:")
    sorted_right = sorted(firing_strengths, key=lambda x: x[1], reverse=True)[:3]
    for rule, strength in sorted_right:
        if strength > 0:
            rule_str = " AND ".join([f"{a[0]}={a[1]}" for a in rule[:-2]])
            action_str = f"speed={rule[-2][1]}, dir={rule[-1][1]}"
            print(f"  [{strength:.3f}] IF {rule_str} THEN {action_str}")

    # ==================== END DEBUG =========================

    return crisp_output

def avoid_obstacle():
    global regions_
    
    front_fuzzifier.calc_membership_values(regions_['front'])
    front_left_fuzzifier.calc_membership_values(regions_['front_left'])
    front_right_fuzzifier.calc_membership_values(regions_['front_right'])

    firing_strengths = obstacle_avoidance_inference.compute_firing_strengths()

    defuzzifier = Defuzzifier(firing_strengths, output_peaks_OA)
    crisp_output = defuzzifier.defuzzify()

    # ==================== DEBUG PRINTS ====================
    # Obstacle Avoidance memberships
    print(f"\n{'='*60}")
    print(f"OA MEMBERSHIP VALUES DEBUG")
    print(f"{'='*60}")

    print(f"\n[OBSTACLE AVOIDANCE]")
    print(f"Front ({regions_['front']:.2f}m):")
    for label, val in front_fuzzifier.membership_values.items():
        print(f"  {label:8s}: {val:.3f} {'█' * int(val * 20)}")
    
    print(f"Front Left ({regions_['front_left']:.2f}m):")
    for label, val in front_left_fuzzifier.membership_values.items():
        print(f"  {label:8s}: {val:.3f} {'█' * int(val * 20)}")
    
    print(f"Front Right ({regions_['front_right']:.2f}m):")
    for label, val in front_right_fuzzifier.membership_values.items():
        print(f"  {label:8s}: {val:.3f} {'█' * int(val * 20)}")
    
    # Rule firing strengths
    print(f"\n[FIRING STRENGTHS]")
    print(f"\nObstacle Avoidance - Top 3 Rules:")
    sorted_obstacle = sorted(firing_strengths, key=lambda x: x[1], reverse=True)[:3]
    for rule, strength in sorted_obstacle:
        if strength > 0:
            rule_str = " AND ".join([f"{a[0]}={a[1]}" for a in rule[:-2]])
            action_str = f"speed={rule[-2][1]}, dir={rule[-1][1]}"
            print(f"  [{strength:.3f}] IF {rule_str} THEN {action_str}")
    # ==================== END DEBUG =========================


    return crisp_output

def pid_controller(desired_distance=0.5, direction='right'):
    '''
    PID controller implemenation, 
    used for right or left edge following
    
    Args:
        desired_distance: desired distance from wall
        reference: Which sensor to use ('right' or 'left')
    
    Returns:
        Correction value
    '''
    global ei, e_previous
    msg = Twist()
    
    # Calculate error
    e = desired_distance - regions_[direction]
    
    # Integral term
    ei += e
    
    # Derivative term
    ed = e - e_previous
    e_previous = e
    
    # PID output
    output = (KP * e) + (KI * ei) + (KD * ed)
    
    msg.linear.x = 0.3
    msg.angular.z = float(output)  

    return msg

#Basic movement method
def movement():
    global regions_, mynode_
    regions = regions_
    
    print(f"Left: {regions['left']} | Front: {regions['front']} | Right: {regions['right']} \n")
    
    msg = Twist()

    msg.linear.x = float(1.0)
    msg.angular.z = float(0.0)

    return msg

def stop():
    global mynode_
    msg = Twist()
    msg.angular.z = float(0.0)
    msg.linear.x = float(0.0)
    return(msg)

def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    # publisher for twist velocity messages
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds 
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle interrupts
    try:
        rclpy.spin(mynode_)
    except Exception as e:
        print(e)
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()

if __name__ == '__main__':
    main()
