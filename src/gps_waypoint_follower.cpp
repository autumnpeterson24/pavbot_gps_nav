// gps_waypoint_follower.cpp
//
// GPS-only waypoint navigation (no encoders).
// Uses NavSatFix + COG/SOG to publish cmd_vel for your cmd_vel_to_wheels node.
//
// Topics:
//   Sub: /gps/fix     (sensor_msgs/NavSatFix)
//   Sub: /gps/cog_deg (std_msgs/Float32) course-over-ground deg (0=N, 90=E)
//   Sub: /gps/sog_mps (std_msgs/Float32) speed-over-ground m/s
//   Pub: /cmd_vel_nav (geometry_msgs/Twist)
//
// Waypoints param: ["lat,lon", "lat,lon", ...]
//
// Coordinate convention used internally:
//   ENU meters: +E (east), +N (north)
//   yaw_enu: 0 rad = north, +pi/2 = east  (matches GPS COG convention)
//   bearing: atan2(E, N) (also matches yaw_enu)
//
// This is intentional so COG_deg can be used directly as yaw_enu.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <optional>
#include <algorithm>

struct WaypointLL {
  double lat_deg{0.0};
  double lon_deg{0.0};
};

static inline double deg2rad(double d) { return d * M_PI / 180.0; }
static inline double rad2deg(double r) { return r * 180.0 / M_PI; }

static inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

static inline double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// Local tangent plane approximation around origin (lat0/lon0).
// Output: e (east meters), n (north meters).
static inline void ll_to_enu_m(double lat_deg, double lon_deg,
                              double lat0_deg, double lon0_deg,
                              double &e_m, double &n_m)
{
  constexpr double R = 6378137.0; // meters
  const double lat  = deg2rad(lat_deg);
  const double lon  = deg2rad(lon_deg);
  const double lat0 = deg2rad(lat0_deg);
  const double lon0 = deg2rad(lon0_deg);

  e_m = R * std::cos(lat0) * (lon - lon0);
  n_m = R * (lat - lat0);
}

static inline std::vector<WaypointLL> parse_waypoints(const std::vector<std::string>& items) {
  std::vector<WaypointLL> out;
  out.reserve(items.size());
  for (const auto& s : items) {
    std::stringstream ss(s);
    std::string a, b;
    if (!std::getline(ss, a, ',')) continue;
    if (!std::getline(ss, b, ',')) continue;
    try {
      WaypointLL w;
      w.lat_deg = std::stod(a);
      w.lon_deg = std::stod(b);
      out.push_back(w);
    } catch (...) {
      // skip invalid entry
    }
  }
  return out;
}

class GPSWaypointFollower : public rclcpp::Node {
public:
  GPSWaypointFollower() : Node("gps_waypoint_follower") {
    // Topics
    fix_topic_     = declare_parameter<std::string>("fix_topic", "/gps/fix");
    cog_topic_     = declare_parameter<std::string>("cog_topic", "/gps/cog_deg");
    sog_topic_     = declare_parameter<std::string>("sog_topic", "/gps/sog_mps");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_nav");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/sensors/imu/heading");

    // Control loop
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);

    // Waypoints
    waypoint_strings_ = declare_parameter<std::vector<std::string>>("waypoints", std::vector<std::string>{});
    waypoints_ = parse_waypoints(waypoint_strings_);
    if (waypoints_.empty()) {
      RCLCPP_WARN(get_logger(),
        "No waypoints set. Example: ros2 param set /gps_waypoint_follower waypoints \"['33.42,-111.93','33.4201,-111.9302']\"");
    } else {
      RCLCPP_INFO(get_logger(), "Loaded %zu waypoint(s).", waypoints_.size());
    }

    // Goal logic
    goal_radius_m_ = declare_parameter<double>("goal_radius_m", 2.5);
    slow_radius_m_ = declare_parameter<double>("slow_radius_m", 6.0);
    advance_hold_time_sec_ = declare_parameter<double>("advance_hold_time_sec", 0.5);
    stop_at_final_waypoint_ = declare_parameter<bool>("stop_at_final_waypoint", true);

    // Gains/limits
    kv_    = declare_parameter<double>("k_v", 0.35);
    kw_    = declare_parameter<double>("k_w", 1.8);
    v_max_ = declare_parameter<double>("v_max", 0.6);
    v_min_ = declare_parameter<double>("v_min", 0.0);
    w_max_ = declare_parameter<double>("w_max", 1.4);

    // Turn-in-place threshold
    heading_turn_only_rad_ = declare_parameter<double>("heading_turn_only_rad", 35.0 * M_PI / 180.0);

    // Freshness and gating
    fix_timeout_sec_ = declare_parameter<double>("fix_timeout_sec", 1.0);
    cog_timeout_sec_ = declare_parameter<double>("cog_timeout_sec", 1.0);
    cog_min_speed_mps_ = declare_parameter<double>("cog_min_speed_mps", 0.25);

    //IMU freshness
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.5);
    use_imu_yaw_ = declare_parameter<bool>("use_imu_yaw", true);
    imu_prefer_below_speed_mps_ = declare_parameter<double>("imu_prefer_below_speed_mps", 0.30);
    imu_blend_weight_ = declare_parameter<double>("imu_blend_weight", 0.15);

    // Optional behavior: creep forward briefly to obtain COG if yaw unknown
    creep_for_cog_ = declare_parameter<bool>("creep_for_cog", false);
    creep_v_mps_   = declare_parameter<double>("creep_v_mps", 0.15);
    creep_time_sec_= declare_parameter<double>("creep_time_sec", 0.6);

    // Position smoothing
    use_enu_ema_   = declare_parameter<bool>("use_enu_ema", true);
    enu_ema_alpha_ = declare_parameter<double>("enu_ema_alpha", 0.85);

    // IMU params
    imu_yaw_offset_deg_ = declare_parameter<double>("imu_yaw_offset_deg", 0.0);

    // ROS I/O
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic_, rclcpp::QoS(10),
      std::bind(&GPSWaypointFollower::on_fix, this, std::placeholders::_1));

    cog_sub_ = create_subscription<std_msgs::msg::Float32>(
      cog_topic_, rclcpp::QoS(10),
      std::bind(&GPSWaypointFollower::on_cog, this, std::placeholders::_1));

    sog_sub_ = create_subscription<std_msgs::msg::Float32>(
      sog_topic_, rclcpp::QoS(10),
      std::bind(&GPSWaypointFollower::on_sog, this, std::placeholders::_1));

    imu_sub_ = create_subscription<std_msgs::msg::Float32>(
    imu_topic_, rclcpp::QoS(10),
    std::bind(&GPSWaypointFollower::on_imu_heading, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GPSWaypointFollower::tick, this));

    RCLCPP_INFO(get_logger(),
      "gps_waypoint_follower running. sub: %s %s %s  pub: %s",
      fix_topic_.c_str(), cog_topic_.c_str(), sog_topic_.c_str(), cmd_vel_topic_.c_str());
  }

private:
  // -------- Callbacks --------
  void on_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    last_fix_time_ = now();
    last_fix_ = *msg;

    if (!origin_set_ && fix_usable(*msg)) {
      origin_lat_deg_ = msg->latitude;
      origin_lon_deg_ = msg->longitude;
      origin_set_ = true;

      double e, n;
      ll_to_enu_m(msg->latitude, msg->longitude, origin_lat_deg_, origin_lon_deg_, e, n);
      enu_filt_e_ = e;
      enu_filt_n_ = n;

      RCLCPP_INFO(get_logger(), "Origin set: lat0=%.8f lon0=%.8f", origin_lat_deg_, origin_lon_deg_);
    }
  }

  void on_cog(const std_msgs::msg::Float32::SharedPtr msg) {
    last_cog_time_ = now();
    last_cog_deg_  = msg->data;
    has_cog_ = true;

    // Save last good yaw immediately (still gated by speed in current_yaw_enu())
    last_good_yaw_ = wrap_pi(deg2rad(last_cog_deg_));
    has_last_good_yaw_ = true;
  }

  void on_sog(const std_msgs::msg::Float32::SharedPtr msg) {
    last_sog_time_ = now();
    last_sog_mps_  = msg->data;
    has_sog_ = true;
  }

  void on_imu_heading(const std_msgs::msg::Float32::SharedPtr msg) {
    last_imu_time_ = now();
    last_imu_heading_deg_ = msg->data;
    has_imu_ = true;
  }

  // -------- Helpers --------
  bool fix_usable(const sensor_msgs::msg::NavSatFix& fix) const {
    if (fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) return false;
    if (!std::isfinite(fix.latitude) || !std::isfinite(fix.longitude)) return false;
    return true;
  }

  bool fresh(const rclcpp::Time& t, double timeout_sec) const {
    if (t.nanoseconds() == 0) return false;
    return (now() - t).seconds() <= timeout_sec;
  }

  std::optional<std::pair<double,double>> current_enu() {
    if (!origin_set_) return std::nullopt;
    if (!fix_usable(last_fix_)) return std::nullopt;
    if (!fresh(last_fix_time_, fix_timeout_sec_)) return std::nullopt;

    double e, n;
    ll_to_enu_m(last_fix_.latitude, last_fix_.longitude, origin_lat_deg_, origin_lon_deg_, e, n);

    if (use_enu_ema_) {
      const double a = clamp(enu_ema_alpha_, 0.0, 0.99);
      enu_filt_e_ = a * enu_filt_e_ + (1.0 - a) * e;
      enu_filt_n_ = a * enu_filt_n_ + (1.0 - a) * n;
      return std::make_pair(enu_filt_e_, enu_filt_n_);
    }
    return std::make_pair(e, n);
  }

  static inline double blend_angles(double a, double b, double w) {
    // wrap-aware blend from a toward b by weight w in [0,1]
    const double d = wrap_pi(b - a);
    return wrap_pi(a + clamp(w, 0.0, 1.0) * d);
  }

  // yaw_enu (rad): 0=north, +pi/2=east. Matches GPS COG degrees and your bearing atan2(E,N).
  std::optional<double> current_yaw_enu() {
    const auto imu_yaw_opt = current_imu_yaw_enu();

    const bool cog_ok = (has_cog_ && fresh(last_cog_time_, cog_timeout_sec_));
    const bool sog_ok = (has_sog_ && fresh(last_sog_time_, cog_timeout_sec_));

    // If no COG at all, fall back to IMU if available
    if (!cog_ok) {
      if (imu_yaw_opt) return *imu_yaw_opt;
      return std::nullopt;
    }

    // Compute COG yaw
    const double cog_yaw = wrap_pi(deg2rad(last_cog_deg_));

    // If speed is known and very low, prefer IMU (COG is noisy when stopped)
    if (sog_ok && last_sog_mps_ < imu_prefer_below_speed_mps_) {
      if (imu_yaw_opt) {
        last_good_yaw_ = *imu_yaw_opt;
        has_last_good_yaw_ = true;
        return *imu_yaw_opt;
      }
      // If no IMU, use last good yaw (your existing behavior)
      if (has_last_good_yaw_) return last_good_yaw_;
      return cog_yaw; // last resort
    }

    // Moving: optionally blend IMU with COG (keeps COG absolute, smooths with IMU)
    if (imu_yaw_opt) {
      const double fused = blend_angles(cog_yaw, *imu_yaw_opt, imu_blend_weight_);
      last_good_yaw_ = fused;
      has_last_good_yaw_ = true;
      return fused;
    }

    // No IMU: use COG
    last_good_yaw_ = cog_yaw;
    has_last_good_yaw_ = true;
    return cog_yaw;
  }

  void publish_stop() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  std::optional<double> current_imu_yaw_enu() {
    if (!use_imu_yaw_) return std::nullopt;
    if (!has_imu_ || !fresh(last_imu_time_, imu_timeout_sec_)) return std::nullopt;

    // Apply mount/calibration offset (degrees)
    double h = static_cast<double>(last_imu_heading_deg_) + imu_yaw_offset_deg_;

    // Wrap to [0, 360)
    while (h >= 360.0) h -= 360.0;
    while (h < 0.0) h += 360.0;

    // Convert to radians in your yaw_enu convention (0=N, +pi/2=E)
    return wrap_pi(deg2rad(h));
  }

  // -------- Control Loop --------
  void tick() {
    if (waypoints_.empty()) {
      publish_stop();
      return;
    }

    auto enu_opt = current_enu();
    if (!enu_opt) {
      publish_stop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "No usable GPS fix/origin yet. Stopping.");
      return;
    }
    const auto [cur_e, cur_n] = *enu_opt;

    // Current waypoint
    const auto& wp = waypoints_[std::min(wp_index_, waypoints_.size()-1)];
    double wp_e, wp_n;
    ll_to_enu_m(wp.lat_deg, wp.lon_deg, origin_lat_deg_, origin_lon_deg_, wp_e, wp_n);

    const double err_e = wp_e - cur_e;
    const double err_n = wp_n - cur_n;
    const double dist = std::hypot(err_e, err_n);

    // Goal reached?
    if (dist <= goal_radius_m_) {
      if (!in_goal_zone_) {
        in_goal_zone_ = true;
        goal_zone_enter_time_ = now();
      }

      const double inside_sec = (now() - goal_zone_enter_time_).seconds();
      publish_stop();

      if (inside_sec >= advance_hold_time_sec_) {
        if (wp_index_ + 1 < waypoints_.size()) {
          wp_index_++;
          in_goal_zone_ = false;
          RCLCPP_INFO(get_logger(), "Reached waypoint. Advancing to %zu/%zu.",
                      wp_index_+1, waypoints_.size());
        } else {
          if (stop_at_final_waypoint_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Final waypoint reached. Holding stop.");
          }
        }
      }
      return;
    } else {
      in_goal_zone_ = false;
    }

    // Need heading
    auto yaw_opt = current_yaw_enu();
    if (!yaw_opt) {
      // Optionally creep forward to get a valid COG estimate
      if (creep_for_cog_) {
        if (!creep_active_) {
          creep_active_ = true;
          creep_start_time_ = now();
        }
        const double t = (now() - creep_start_time_).seconds();
        if (t <= creep_time_sec_) {
          geometry_msgs::msg::Twist cmd;
          cmd.linear.x = creep_v_mps_;
          cmd.angular.z = 0.0;
          cmd_pub_->publish(cmd);
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "No COG yet; creeping forward to establish heading...");
          return;
        } else {
          creep_active_ = false;
        }
      }

      publish_stop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "No valid COG heading yet (needs motion). Stopping.");
      return;
    }
    creep_active_ = false;

    const double yaw = *yaw_opt;

    // Bearing to waypoint in yaw_enu convention
    const double bearing = std::atan2(err_e, err_n); // atan2(E, N)
    const double heading_err = wrap_pi(bearing - yaw);

    // Control
    double w = clamp(kw_ * heading_err, -w_max_, w_max_);

    double v = 0.0;
    if (std::abs(heading_err) > heading_turn_only_rad_) {
      v = 0.0; // turn-in-place until pointed
    } else {
      // base speed from distance
      double v_cmd = clamp(kv_ * dist, v_min_, v_max_);

      // slow down within slow_radius
      if (slow_radius_m_ > goal_radius_m_ && dist < slow_radius_m_) {
        const double s = clamp((dist - goal_radius_m_) / (slow_radius_m_ - goal_radius_m_), 0.15, 1.0);
        v_cmd *= s;
      }

      // slow down when turning hard
      const double turn_slow = clamp(1.0 - (std::abs(w) / w_max_) * 0.6, 0.25, 1.0);
      v = v_cmd * turn_slow;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "WP %zu/%zu dist=%.2f m bearing=%.1f yaw=%.1f err=%.1f cmd(v=%.2f,w=%.2f)",
      wp_index_+1, waypoints_.size(),
      dist, rad2deg(bearing), rad2deg(yaw), rad2deg(heading_err),
      v, w);
  }

  // Topics
  std::string fix_topic_, cog_topic_, sog_topic_, cmd_vel_topic_;

  // Params
  double control_rate_hz_{20.0};

  std::vector<std::string> waypoint_strings_;
  std::vector<WaypointLL> waypoints_;
  size_t wp_index_{0};

  double goal_radius_m_{2.5};
  double slow_radius_m_{6.0};
  double advance_hold_time_sec_{0.5};
  bool stop_at_final_waypoint_{true};

  double kv_{0.35}, kw_{1.8};
  double v_max_{0.6}, v_min_{0.0}, w_max_{1.4};
  double heading_turn_only_rad_{35.0 * M_PI / 180.0};

  double fix_timeout_sec_{1.0};
  double cog_timeout_sec_{1.0};
  double cog_min_speed_mps_{0.25};

  bool creep_for_cog_{false};
  double creep_v_mps_{0.15};
  double creep_time_sec_{0.6};

  bool use_enu_ema_{true};
  double enu_ema_alpha_{0.85};

  // State: origin and filtered ENU
  bool origin_set_{false};
  double origin_lat_deg_{0.0}, origin_lon_deg_{0.0};
  double enu_filt_e_{0.0}, enu_filt_n_{0.0};

  // State: GPS fix / COG / SOG
  sensor_msgs::msg::NavSatFix last_fix_{};
  rclcpp::Time last_fix_time_{};

  bool has_cog_{false};
  float last_cog_deg_{0.0f};
  rclcpp::Time last_cog_time_{};

  bool has_sog_{false};
  float last_sog_mps_{0.0f};
  rclcpp::Time last_sog_time_{};

  bool has_last_good_yaw_{false};
  double last_good_yaw_{0.0};

  // Goal-zone debounce
  bool in_goal_zone_{false};
  rclcpp::Time goal_zone_enter_time_{};

  // Creep helper
  bool creep_active_{false};
  rclcpp::Time creep_start_time_{};

  // ---- IMU params/state ----
  std::string imu_topic_;
  double imu_timeout_sec_{0.5};
  bool use_imu_yaw_{true};
  double imu_prefer_below_speed_mps_{0.30};  // when slow/stopped, use IMU
  double imu_blend_weight_{0.15};            // 0=COG only, 1=IMU only (when blending)

  // IMU state
  bool has_imu_{false};
  float last_imu_heading_deg_{0.0f};
  rclcpp::Time last_imu_time_{};

  double imu_yaw_offset_deg_{0.0}; // param for offset (point true north)



  // ROS
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cog_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sog_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr imu_sub_; //IMU sub
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSWaypointFollower>());
  rclcpp::shutdown();
  return 0;
}
