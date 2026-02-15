#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <thread>
#include <mutex>

namespace gazebo
{
class BangBangReactionWheel : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();

    body_link_name_ = GetSdfStr(sdf, "body_link", "base_link");
    ros_ns_         = GetSdfStr(sdf, "ros_namespace", "");
    topic_cw_       = GetSdfStr(sdf, "topic_cw", "reaction_wheel/cw");
    topic_ccw_      = GetSdfStr(sdf, "topic_ccw", "reaction_wheel/ccw");
    tau_max_        = GetSdfDouble(sdf, "tau_max", 0.05);

    // Axis in BODY frame (default z)
    axis_ = GetSdfVec3(sdf, "axis", ignition::math::Vector3d(0,0,1));
    if (axis_.Length() < 1e-9) axis_ = ignition::math::Vector3d(0,0,1);
    axis_.Normalize();

    body_link_ = model_->GetLink(body_link_name_);
    if (!body_link_)
      gzthrow("BangBangReactionWheel: body_link not found: " << body_link_name_);

    // ROS 2 init (safe inside Gazebo process)
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>("bang_bang_reaction_wheel", ros_ns_);

    sub_cw_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic_cw_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        cw_on_ = msg->data;
      });

    sub_ccw_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic_ccw_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        ccw_on_ = msg->data;
      });

    ros_spin_thread_ = std::thread([this]()
    {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(node_);
      while (rclcpp::ok())
        exec.spin_some();
    });

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BangBangReactionWheel::OnUpdate, this));

    gzmsg << "BangBangReactionWheel loaded. ns=" << ros_ns_
          << " cw=" << topic_cw_ << " ccw=" << topic_ccw_
          << " tau_max=" << tau_max_ << " axis=" << axis_ << "\n";
  }

  void OnUpdate()
  {
    bool cw, ccw;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      cw = cw_on_;
      ccw = ccw_on_;
    }

    // If both pressed, cancel out (safe)
    double s = 0.0;
    if (cw && !ccw) s = -1.0;
    else if (ccw && !cw) s = +1.0;
    else return;

    // Torque vector in BODY frame
    ignition::math::Vector3d tau_body = axis_ * (tau_max_ * s);

    // Convert to WORLD frame
    ignition::math::Pose3d pose = body_link_->WorldPose();
    ignition::math::Vector3d tau_world = pose.Rot().RotateVector(tau_body);

    // Apply torque
    body_link_->AddTorque(tau_world);
  }

  ~BangBangReactionWheel() override
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
  static ignition::math::Vector3d GetSdfVec3(sdf::ElementPtr sdf, const std::string& key,
                                            const ignition::math::Vector3d& def)
  {
    if (!sdf->HasElement(key)) return def;
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cw_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ccw_;
  std::thread ros_spin_thread_;
  std::mutex mtx_;

  std::string body_link_name_;
  std::string ros_ns_;
  std::string topic_cw_;
  std::string topic_ccw_;
  ignition::math::Vector3d axis_{0,0,1};
  double tau_max_{0.05};

  bool cw_on_{false};
  bool ccw_on_{false};
};

GZ_REGISTER_MODEL_PLUGIN(BangBangReactionWheel)
} // namespace gazebo
