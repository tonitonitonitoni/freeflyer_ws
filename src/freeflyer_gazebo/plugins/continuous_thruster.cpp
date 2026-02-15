#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

#include <thread>
#include <mutex>
#include <algorithm>

namespace gazebo
{
class ContinuousThruster : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();

    body_link_name_ = GetSdfStr(sdf, "body_link", "base_link");
    command_topic_  = GetSdfStr(sdf, "command_topic", "thrusters/0");
    ros_ns_         = GetSdfStr(sdf, "ros_namespace", "");
    max_thrust_     = GetSdfDouble(sdf, "max_thrust", 1.0);

    thrust_axis_ = GetSdfVec3(sdf, "thrust_axis", ignition::math::Vector3d(1,0,0));
    app_point_   = GetSdfVec3(sdf, "application_point", ignition::math::Vector3d(0,0,0));

    if (thrust_axis_.Length() < 1e-9)
      thrust_axis_ = ignition::math::Vector3d(1,0,0);

    thrust_axis_.Normalize();

    body_link_ = model_->GetLink(body_link_name_);
    if (!body_link_)
      gzthrow("ContinuousThruster: body_link not found: " << body_link_name_);

    enforce_planar_ = GetSdfBool(sdf, "enforce_planar", true);
    plane_z_ = GetSdfDouble(sdf, "plane_z", body_link_->WorldPose().Pos().Z());

    publish_odom_ = GetSdfBool(sdf, "publish_odom", false);
    odom_topic_ = GetSdfStr(sdf, "odom_topic", "/odom");
    odom_frame_id_ = GetSdfStr(sdf, "odom_frame_id", "odom");
    odom_child_frame_id_ = GetSdfStr(sdf, "odom_child_frame_id", "base_link");

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    std::string node_name = "continuous_thruster_" + sdf->Get<std::string>("name");
    node_ = std::make_shared<rclcpp::Node>(node_name, ros_ns_);

    sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      command_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Float32::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        requested_cmd_ = std::clamp(msg->data, 0.0f, 1.0f);
      });

    if (publish_odom_)
      odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    ros_spin_thread_ = std::thread([this]()
    {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(node_);
      while (rclcpp::ok())
        exec.spin_some();
    });

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ContinuousThruster::OnUpdate, this));

    gzmsg << "ContinuousThruster loaded. topic=" << command_topic_
          << " Fmax=" << max_thrust_ << "\n";
  }

  void OnUpdate()
  {
    float cmd;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      cmd = requested_cmd_;
    }

    if (cmd > 1e-5)
    {
      ignition::math::Pose3d pose = body_link_->WorldPose();

      double thrust = max_thrust_ * cmd;
      ignition::math::Vector3d F_world =
        pose.Rot().RotateVector(thrust_axis_ * thrust);

      ignition::math::Vector3d p_world =
        pose.Pos() + pose.Rot().RotateVector(app_point_);

      body_link_->AddForceAtWorldPosition(F_world, p_world);
    }

    if (enforce_planar_)
    {
      auto v = body_link_->WorldLinearVel();
      v.Z(0.0);
      body_link_->SetLinearVel(v);

      auto w = body_link_->WorldAngularVel();
      w.X(0.0);
      w.Y(0.0);
      body_link_->SetAngularVel(w);

      auto pose = body_link_->WorldPose();
      double yaw = pose.Rot().Yaw();
      pose.Pos().Z(plane_z_);
      pose.Rot() = ignition::math::Quaterniond(0.0, 0.0, yaw);
      body_link_->SetWorldPose(pose);
    }
  }

  ~ContinuousThruster() override
  {
    if (ros_spin_thread_.joinable())
      ros_spin_thread_.join();
  }

private:
  static std::string GetSdfStr(sdf::ElementPtr sdf, const std::string& key, const std::string& def)
  {
    return sdf->HasElement(key) ? sdf->Get<std::string>(key) : def;
  }

  static double GetSdfDouble(sdf::ElementPtr sdf, const std::string& key, double def)
  {
    return sdf->HasElement(key) ? sdf->Get<double>(key) : def;
  }

  static bool GetSdfBool(sdf::ElementPtr sdf, const std::string& key, bool def)
  {
    return sdf->HasElement(key) ? sdf->Get<bool>(key) : def;
  }

  static ignition::math::Vector3d GetSdfVec3(
    sdf::ElementPtr sdf,
    const std::string& key,
    const ignition::math::Vector3d& def)
  {
    if (!sdf->HasElement(key))
      return def;

    auto e = sdf->GetElement(key);
    if (e->HasAttribute("xyz"))
      return ignition::math::Vector3d(e->Get<ignition::math::Vector3d>("xyz"));

    return sdf->Get<ignition::math::Vector3d>(key);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr body_link_;
  event::ConnectionPtr update_conn_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::thread ros_spin_thread_;
  std::mutex mtx_;

  std::string body_link_name_;
  std::string command_topic_;
  std::string ros_ns_;
  std::string odom_topic_;
  std::string odom_frame_id_;
  std::string odom_child_frame_id_;

  ignition::math::Vector3d thrust_axis_;
  ignition::math::Vector3d app_point_;

  double max_thrust_{1.0};
  bool enforce_planar_{true};
  double plane_z_{0.0};
  bool publish_odom_{false};

  float requested_cmd_{0.0f};
};

GZ_REGISTER_MODEL_PLUGIN(ContinuousThruster)

} // namespace gazebo
