"""
GPS WAYPOINT NAV +++++++++++++++++++++++++++++++++++++
1) LAUNCH TEENSY
ros2 launch pavbot_teensy_control teensy_control.launch.py

2) LAUNCH IMU
ros2 launch pavbot_imu imu.launch.py

3) LAUNCH GPS
ros2 launch pavbot_gps gps.launch.py

4) LAUNCH GPS WAYPOINT NAV
ros2 launch pavbot_gps_nav gps_waypoints.launch.py

+++++++++++++++++++++++++++++++++++++++++++++++++++++++

IMU MOUNTING on PAVBot: ============================
*** MOUNT IMU WITH X = forward, Y = left, Z = up

IMU ORIENTATION:
          (USB side)
             ^
             |
             Y+

   X- <-  [ IMU ]   -> X+

             |
             v
             Y-

0 deg = North
90 deg = East
180 deg = South
270 deg = West
===================================================

Serial ID of GPS and Teensy:
- GPS: /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0
- Teensy: /dev/serial/by-id/usb-Teensyduino_USB_Serial_9147040-if00

BUILDING:
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

- Send waypoints 10 m apart for testing in gps_waypoints.launch.py: 
    - waypoints:="['34.62420833333333,-112.35278333333332','34.62430833333333,-112.35278333333332']" 
    *** USE PYTHON SCRIPT TO GET SQUARE WAYPOINTS



"""


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pavbot_gps_nav",
            executable="gps_waypoint_follower",
            name="gps_waypoint_follower",
            output="screen",
            parameters=[{
                "fix_topic": "/gps/fix",
                "cog_topic": "/gps/cog_deg",
                "sog_topic": "/gps/sog_mps",
                "cmd_vel_topic": "/cmd_vel",

                "control_rate_hz": 20.0,

                # Waypoints (lat,lon strings) *********************************************
                "waypoints": [
                    "34.61444667, -112.45063333"

                ],
                # *************************************************************************

                "goal_radius_m": 1.0, # make it a little lenient for testing
                "slow_radius_m": 2.0,
                "advance_hold_time_sec": 0.5,
                "stop_at_final_waypoint": True,

                "k_v": 0.18,
                "k_w": 0.70,
                "v_max": 0.18, # very small for testing
                "v_min": 0.04,
                "w_max": 0.35, # very small for testing
                "heading_turn_only_rad": 1.05,   # 60 deg

                "fix_timeout_sec": 1.0,
                "cog_timeout_sec": 1.0,
                "cog_min_speed_mps": 0.25,

                # GPS-only startup helper have to have a little speed to get it up and going with no IMU
                "creep_for_cog": False,
                "creep_v_mps": 0.15,
                "creep_time_sec": 0.6,

                "use_enu_ema": True,
                "enu_ema_alpha": 0.85,

                "imu_topic": "/sensors/imu/heading",
                "imu_timeout_sec": 0.5,
                "use_imu_yaw": True,
                "imu_prefer_below_speed_mps": 0.60,
                "imu_blend_weight": 0.0,
            }],
        )
    ])
