#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <thread>
#include <mutex>
#include <algorithm>
#include <cmath>

namespace gazebo
{

// Gazebo model plugin that applies commanded reaction-wheel torque to a body link.
// A normalized command in [-1, 1] is received over ROS 2 and mapped to [-tau_max, tau_max].
class ContinuousReactionWheel : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();

    body_link_name_ = GetSdfStr(sdf, "body_link", "base_link");
    ros_ns_         = GetSdfStr(sdf, "ros_namespace", "");
    topic_cmd_      = GetSdfStr(sdf, "command_topic", "reaction_wheel/cmd");

    tau_max_        = GetSdfDouble(sdf, "tau_max", 0.05);
    spool_tau_      = GetSdfDouble(sdf, "spool_tau", 0.05);

    axis_ = GetSdfVec3(sdf, "axis", ignition::math::Vector3d(0,0,1));
    if (axis_.Length() < 1e-9)
      axis_ = ignition::math::Vector3d(0,0,1);
    axis_.Normalize();

    body_link_ = model_->GetLink(body_link_name_);
    if (!body_link_)
      gzthrow("ContinuousReactionWheel: body_link not found: " << body_link_name_);

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>("continuous_reaction_wheel", ros_ns_);

    // Subscribe to commanded wheel effort and clamp to safe normalized range.
    sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      topic_cmd_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Float32::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        requested_cmd_ = std::clamp(msg->data, -1.0f, 1.0f);
      });

    // Spin ROS callbacks on a dedicated thread so Gazebo update loop stays non-blocking.
    ros_spin_thread_ = std::thread([this]()
    {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(node_);
      while (rclcpp::ok())
        exec.spin_some();
    });

    last_update_time_ = world_->SimTime();

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ContinuousReactionWheel::OnUpdate, this));

    gzmsg << "ContinuousReactionWheel loaded. topic=" << topic_cmd_
          << " tau_max=" << tau_max_
          << " spool_tau=" << spool_tau_
          << " axis=" << axis_ << "\n";
  }

  void OnUpdate()
  {
    // Compute simulation timestep since last update.
    common::Time now = world_->SimTime();
    double dt = (now - last_update_time_).Double();
    dt = std::min(dt, 0.1);  // prevent huge time jumps
    last_update_time_ = now;

    // Read the latest normalized command atomically.
    float cmd;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      cmd = requested_cmd_;
    }

    // Convert normalized command to physical torque.
    double tau_target = tau_max_ * static_cast<double>(cmd);

    // Optionally model wheel spool dynamics as a first-order lag.
    if (spool_tau_ > 1e-6 && dt > 0.0)
    {
      double alpha = std::exp(-dt / spool_tau_);
      tau_current_ = alpha * tau_current_ + (1.0 - alpha) * tau_target;
    }
    else
    {
      tau_current_ = tau_target;
    }

    // Build body-frame torque vector along configured wheel axis.
    ignition::math::Vector3d tau_body = axis_ * tau_current_;

    // Rotate to world frame and apply to the link.
    ignition::math::Pose3d pose = body_link_->WorldPose();
    ignition::math::Vector3d tau_world = pose.Rot().RotateVector(tau_body);

    body_link_->AddTorque(tau_world);
  }

  ~ContinuousReactionWheel() override
  {
    if (ros_spin_thread_.joinable())
      ros_spin_thread_.join();
  }

private:

  static std::string GetSdfStr(sdf::ElementPtr sdf,
                               const std::string& key,
                               const std::string& def)
  {
    return sdf->HasElement(key) ? sdf->Get<std::string>(key) : def;
  }

  static double GetSdfDouble(sdf::ElementPtr sdf,
                             const std::string& key,
                             double def)
  {
    return sdf->HasElement(key) ? sdf->Get<double>(key) : def;
  }

  static ignition::math::Vector3d GetSdfVec3(
    sdf::ElementPtr sdf,
    const std::string& key,
    const ignition::math::Vector3d& def)
  {
    return sdf->HasElement(key)
      ? sdf->Get<ignition::math::Vector3d>(key)
      : def;
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr body_link_;
  event::ConnectionPtr update_conn_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  std::thread ros_spin_thread_;
  std::mutex mtx_;

  std::string body_link_name_;
  std::string ros_ns_;
  std::string topic_cmd_;

  ignition::math::Vector3d axis_{0,0,1};

  double tau_max_{0.05};
  double spool_tau_{0.05};

  float requested_cmd_{0.0f};
  double tau_current_{0.0};

  common::Time last_update_time_;
};

GZ_REGISTER_MODEL_PLUGIN(ContinuousReactionWheel)

} // namespace gazebo
