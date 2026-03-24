"""
1) LAUNCH GPS NAV: (USE SEPARATE TERMINALS)
ros2 launch pavbot_gps gps.launch.py
ros2 launch pavbot_gps_nav gps_waypoints.launch.py

2) LAUNCH IMU:
ros2 launch pavbot_imu imu.launch.py

3) LAUNCH TEENSY CONTROL:
ros2 launch pavbot_teensy_control teensy_transport_serial

4) LAUNCH ESTOP
ros2 launch pavbot_estop estop.launch.py

/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0



BUILDING:
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

TESTING FOR WAYPOINT NAVIGATION:
1. Check current GPS lat and lon: ros2 topic echo /gps/fix --once
    - Should give: 
        header:
            stamp:
                sec: 1771171287
                nanosec: 393463786
            frame_id: gps_link
            status:
            status: 0
            service: 1
            latitude: 34.62410833333333
            longitude: -112.35278333333332
            altitude: 0.0
            position_covariance:
            - 0.0
            - 0.0
            - 0.0
            - 0.0
            - 0.0
            - 0.0
            - 0.0
            - 0.0
            - 0.0
            position_covariance_type: 0
            ---
2. Send waypoints 10 m apart for testing in gps_waypoints.launch.py: 
    - waypoints:="['34.62420833333333,-112.35278333333332','34.62430833333333,-112.35278333333332']" ***(4th decimal point change)
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
                    "34.62418333,-112.35280667",
                    "34.62428333,-112.35280667",
                ],
                # *************************************************************************

                "goal_radius_m": 3.0, # make it a little lenient for testing
                "slow_radius_m": 6.0,
                "advance_hold_time_sec": 0.5,
                "stop_at_final_waypoint": True,

                "k_v": 0.35,
                "k_w": 1.8,
                "v_max": 0.3, # very small for testing
                "v_min": 0.0,
                "w_max": 0.6, # very small for testing
                "heading_turn_only_rad": 0.61,   # ~35 deg

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
                "imu_prefer_below_speed_mps": 0.30,
                "imu_blend_weight": 0.15,
            }],
        )
    ])
