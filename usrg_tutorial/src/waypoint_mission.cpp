#include "usrg_tutorial/waypoint_mission.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/qos.hpp>

enum class ControlState {
    TAKEOFF,     // 이륙 및 첫 WP 이동 (Yaw 고정)
    ROTATING,    // 웨이포인트 도달 후 다음 WP 바라보기
    MOVING       // 다음 WP로 이동 중
};

ControlState state_ = ControlState::TAKEOFF;

using namespace std::chrono_literals;
template <typename T>
T clamp(const T& v, const T& lo, const T& hi)
{
    return std::min(std::max(v, lo), hi);
}

double normalize_angle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

Waypoint_vel_tracker::Waypoint_vel_tracker() : Node("pa_control"),
current_index_(0), offboard_active_(false), has_pose_(false), has_initial_yaw_(false), initial_yaw_(0.0) {
  declare_parameter<std::string>("mission_csv", "test.csv");
  get_parameter("mission_csv", mission_csv_path_);
  RCLCPP_INFO(this->get_logger(), "Mission CSV path: %s", mission_csv_path_.c_str());

  vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/mavros/setpoint_velocity/cmd_vel", rclcpp::SensorDataQoS());

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose",
    rclcpp::SensorDataQoS(), 
    std::bind(&Waypoint_vel_tracker::pose_callback, this, std::placeholders::_1));

    rclcpp::QoS state_qos(10);
    state_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", state_qos,
        std::bind(&Waypoint_vel_tracker::state_callback, this, std::placeholders::_1));

  load_waypoints();
  arming_cli_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  timer_ = create_wall_timer(20ms, std::bind(&Waypoint_vel_tracker::control_loop, this));
  /*marker_timer_ = create_wall_timer(100ms, std::bind(&Waypoint_vel_tracker::publish_waypoint_markers, this));*/

}

void Waypoint_vel_tracker::load_waypoints() {
  /*TODO: Implement loading waypoints from CSV*/
  /*waypoint 초기화*/
  waypoints_.clear();
  /*waypoint 불러오기*/
  std::ifstream file(mission_csv_path_);
  if(!file.is_open()){
    RCLCPP_ERROR(get_logger(), "Failed to open mission CSV: %s", mission_csv_path_.c_str());
    return;
  }

  std::string line;
  /*waypoint x,y,z로 자르고 waypoint저장*/
  while (std::getline (file, line)){
    line.erase(0, line.find_first_not_of(" \t\r\n"));
    line.erase(line.find_last_not_of(" \t\r\n") + 1);
    if (line.empty()) continue;
    if (line[0] == '#') continue;

    std::stringstream ss(line);
    std::string token;
    std::vector<double> cols;
    cols.reserve(3);

    while (std::getline(ss, token, ',')) {
      token.erase(0, token.find_first_not_of(" \t\r\n"));
      token.erase(token   .find_last_not_of(" \t\r\n") + 1);
      if (token.empty()) continue;

      try {
        cols.push_back(std::stod(token));
      } catch (...) {
        cols.clear();
        break;
      }
    }
    if (cols.size()<3) continue;

    Waypoint wp;
    wp.x = cols[0];
    wp.y = cols[1];
    wp.z = cols[2];
    waypoints_.push_back(wp);
  }

  RCLCPP_INFO(this->get_logger(), "loaded %zu waypoints", waypoints_.size());

  if (!waypoints_.empty()){
    RCLCPP_INFO(this->get_logger(), "First waypoint: (%.2f %.2f %.2f)", waypoints_[0].x, waypoints_[0].y, waypoints_[0].z);
  }
}

//when offboard is detected from QGC, drone is armed automatically
void Waypoint_vel_tracker::state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
  if (!offboard_active_ && msg->mode == "OFFBOARD") {
    RCLCPP_INFO(this->get_logger(), "Detected OFFBOARD mode via QGC, arming...");
    offboard_active_ = true;

    for (int i = 0; i < 100; ++i) {
      geometry_msgs::msg::TwistStamped dummy;
      dummy.header.stamp = this->now();
      dummy.twist.linear.x = 0.0;
      dummy.twist.linear.y = 0.0;
      dummy.twist.linear.z = 1.0;
      vel_pub_->publish(dummy);
      rclcpp::sleep_for(10ms); 
    }

    auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_req->value = true;

    arming_cli_->async_send_request(arm_req,
      [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture result) {
        if (result.get()->success) {
          RCLCPP_INFO(this->get_logger(), "Drone armed by code.");
          this->offboard_active_ = true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Arming failed.");
        }
      });
  }
}

void Waypoint_vel_tracker::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  current_pose_ = *msg;
  has_pose_ = true;

  if (!has_initial_yaw_) {
    tf2::Quaternion q;
    tf2::fromMsg(current_pose_.pose.orientation, q);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    initial_yaw_ = yaw;
    has_initial_yaw_ = true;
    RCLCPP_INFO(this->get_logger(), "Captured initial yaw: %.3f rad", initial_yaw_);
  }
}

void Waypoint_vel_tracker::control_loop() {

  geometry_msgs::msg::TwistStamped dummy;
  dummy.header.stamp = now();
  dummy.twist.linear.x = 0.0;
  dummy.twist.linear.y = 0.0;
  dummy.twist.linear.z = 0.0;
  dummy.twist.angular.z = 0.0;
  if (!offboard_active_) {
    vel_pub_->publish(dummy);
    return;
  }
  if (!has_pose_ || !has_initial_yaw_) {
    vel_pub_->publish(dummy);
    return;
  }
  if(waypoints_.empty()){
    vel_pub_->publish(dummy);
    return;
  }
  /*TODO: Implement control logic for waypoints*/
  /*p controller*/
  const double kp_xy = 0.5;
  const double kp_z = 0.5;
  const double vmax_xy = 1;
  const double vmax_z = 0.5;
  const double th = 0.2;
  const double kp_yaw = 1.0;
  const double vmax_yaw = 0.5;

  const double x = current_pose_.pose.position.x;
  const double y = current_pose_.pose.position.y;
  const double z = current_pose_.pose.position.z;

  tf2::Quaternion q;
  tf2::fromMsg(current_pose_.pose.orientation, q);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  const double yaw_error = normalize_angle(initial_yaw_ - yaw);
  dummy.twist.angular.z = clamp(kp_yaw * yaw_error, -vmax_yaw, vmax_yaw);

  size_t target_idx = current_index_;
  bool is_last_point_reached = false;

  if(target_idx >= waypoints_.size()){
    target_idx = waypoints_.size() -1;
  }

  const Waypoint &wp = waypoints_[target_idx];

  const double del_x = wp.x - x;
  const double del_y = wp.y - y;
  const double del_z = wp.z - z;

  const double dist = std::sqrt(del_x*del_x + del_y*del_y + del_z*del_z);

  if(state_ == ControlState::TAKEOFF){
    dummy.twist.linear.x = clamp(kp_xy * del_x, -vmax_xy, vmax_xy);
    dummy.twist.linear.y = clamp(kp_xy * del_y, -vmax_xy, vmax_xy);
    dummy.twist.linear.z = clamp(kp_z * del_z, -vmax_z, vmax_z);

    if(dist<th){
      RCLCPP_INFO(get_logger(), "Takeoff complete. Switching to MOVING");
      state_=ControlState::MOVING;
      if(current_index_ < waypoints_.size()-1){
        current_index_++;
        const Waypoint &next_wp = waypoints_[current_index_];
        RCLCPP_INFO(get_logger(), "Next target: WP %zu (%.2f, %.2f, %.2f)", 
                   current_index_ + 1, next_wp.x, next_wp.y, next_wp.z);
      }
    }
  }

  /*다음 waypoint 도달 했으면 다음 waypoint로 waypoint변경*/
  else if(state_==ControlState::MOVING){
    if(dist<th){
      if (current_index_<waypoints_.size()-1){
        RCLCPP_INFO(get_logger(), "Reached WP %zu", current_index_ + 1);
        current_index_++;
        const Waypoint &next_wp = waypoints_[current_index_];
        RCLCPP_INFO(get_logger(), "Next target: WP %zu (%.2f, %.2f, %.2f)", 
                current_index_ + 1, next_wp.x, next_wp.y, next_wp.z);
        vel_pub_->publish(dummy);
        return;
      }
      else{
        static bool final_reached = false;
        if(!final_reached){
          RCLCPP_INFO(get_logger(), "Reached Final WP! Maintaining position.");
          final_reached = true;
          current_index_=waypoints_.size();
        }
    }
    }
  }

  dummy.twist.linear.x = clamp(kp_xy * del_x, -vmax_xy, vmax_xy);
  dummy.twist.linear.y = clamp(kp_xy * del_y, -vmax_xy, vmax_xy);
  dummy.twist.linear.z = clamp(kp_z * del_z, -vmax_z, vmax_z);

  vel_pub_->publish(dummy);
  return;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Waypoint_vel_tracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
