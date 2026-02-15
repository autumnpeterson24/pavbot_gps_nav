"""
LAUNCH GPS NAV: (USE SEPARATE TERMINALS)
ros2 launch pavbot_gps gps.launch.py
ros2 launch pavbot_gps_nav gps_waypoints.launch.py

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
                "cmd_vel_topic": "/cmd_vel_nav",

                "control_rate_hz": 20.0,

                # Waypoints (lat,lon strings)
                "waypoints": [
                    "33.4212345,-111.9332100",
                    "33.4212800,-111.9330500",
                ],

                "goal_radius_m": 2.5,
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

                # GPS-only startup helper (optional) have to have a little speed to get it up and going with no IMU
                "creep_for_cog": False,
                "creep_v_mps": 0.15,
                "creep_time_sec": 0.6,

                "use_enu_ema": True,
                "enu_ema_alpha": 0.85,
            }],
        )
    ])
