#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <thread>
#include <mutex>
#include <cmath>
#include <cctype>

namespace gazebo
{
namespace
{
std::string SanitizeNodeName(const std::string &raw)
{
  std::string out;
  out.reserve(raw.size());
  for (const unsigned char c : raw)
  {
    if (std::isalnum(c) || c == '_')
      out.push_back(static_cast<char>(c));
    else
      out.push_back('_');
  }
  return out;
}
}  // namespace

class BangBangThruster : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();

    body_link_name_ = sdf->Get<std::string>("body_link", "base_link").first;
    command_topic_  = sdf->Get<std::string>("command_topic", "thrusters/0").first;
    ros_ns_         = sdf->Get<std::string>("ros_namespace", "").first;
    max_thrust_     = sdf->Get<double>("max_thrust", 2.0).first;
    spool_tau_      = sdf->Get<double>("spool_tau", 0.1).first;

    thrust_axis_ = sdf->Get<ignition::math::Vector3d>(
        "thrust_axis", ignition::math::Vector3d(1,0,0)).first;

    if (thrust_axis_.Length() < 1e-9)
      thrust_axis_ = ignition::math::Vector3d(1,0,0);

    thrust_axis_.Normalize();

    body_link_ = model_->GetLink(body_link_name_);
    if (!body_link_)
      gzthrow("BangBangThruster: body_link not found.");

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>(
        "bang_bang_thruster_" + SanitizeNodeName(command_topic_), ros_ns_);

    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      command_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        requested_on_ = msg->data;
      });

    ros_thread_ = std::thread([this]()
    {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(node_);
      while (rclcpp::ok())
        exec.spin_some();
    });

    last_time_ = world_->SimTime();

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BangBangThruster::OnUpdate, this));

    gzmsg << "BangBangThruster loaded. Fmax="
          << max_thrust_ << " tau=" << spool_tau_ << "\n";
  }

  void OnUpdate()
  {
    common::Time now = world_->SimTime();
    double dt = (now - last_time_).Double();
    last_time_ = now;

    bool on;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      on = requested_on_;
    }

    double F_target = on ? max_thrust_ : 0.0;

    // First-order spool lag
    if (spool_tau_ > 0.0 && dt > 0.0)
    {
      double a = std::exp(-dt / spool_tau_);
      F_actual_ = a * F_actual_ + (1.0 - a) * F_target;
    }
    else
    {
      F_actual_ = F_target;
    }

    ignition::math::Vector3d F_body = thrust_axis_ * F_actual_;
    body_link_->AddRelativeForce(F_body);
  }

  ~BangBangThruster() override
  {
    if (ros_thread_.joinable())
      ros_thread_.join();
  }

private:
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr body_link_;
  event::ConnectionPtr update_conn_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::thread ros_thread_;
  std::mutex mtx_;

  std::string body_link_name_;
  std::string command_topic_;
  std::string ros_ns_;

  ignition::math::Vector3d thrust_axis_;

  double max_thrust_{2.0};
  double spool_tau_{0.1};

  bool requested_on_{false};
  double F_actual_{0.0};

  common::Time last_time_;
};

GZ_REGISTER_MODEL_PLUGIN(BangBangThruster)

} // namespace gazebo
