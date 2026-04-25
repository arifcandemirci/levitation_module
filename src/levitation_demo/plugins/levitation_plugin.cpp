#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <geometry_msgs/msg/twist.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>

namespace gazebo
{
class LevitationPlugin : public ModelPlugin
{
public:
  LevitationPlugin() = default;

  ~LevitationPlugin() override
  {
    if (this->executor_) {
      this->executor_->cancel();
    }
    if (this->executor_thread_.joinable()) {
      this->executor_thread_.join();
    }
    this->executor_.reset();
    this->ros_node_.reset();
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = model;
    this->world_ = model->GetWorld();

    this->link_name_ = sdf->HasElement("link_name") ? sdf->Get<std::string>("link_name") : "base_link";
    this->cmd_topic_ = sdf->HasElement("cmd_topic") ? sdf->Get<std::string>("cmd_topic") : "/levitation/cmd_vel";
    this->plate_model_name_ = sdf->HasElement("plate_model_name") ? sdf->Get<std::string>("plate_model_name") : "copper_plate";
    this->plate_link_name_ = sdf->HasElement("plate_link_name") ? sdf->Get<std::string>("plate_link_name") : "plate_link";

    this->plate_thickness_ = sdf->HasElement("plate_thickness") ? sdf->Get<double>("plate_thickness") : 0.005;
    this->plate_size_x_ = sdf->HasElement("plate_size_x") ? sdf->Get<double>("plate_size_x") : 4.0;
    this->plate_size_y_ = sdf->HasElement("plate_size_y") ? sdf->Get<double>("plate_size_y") : 6.0;
    this->module_top_offset_ = sdf->HasElement("module_top_offset") ? sdf->Get<double>("module_top_offset") : 0.0025;
    this->target_gap_ = sdf->HasElement("target_gap") ? sdf->Get<double>("target_gap") : 0.02;

    this->z_kp_ = sdf->HasElement("z_kp") ? sdf->Get<double>("z_kp") : 180.0;
    this->z_kd_ = sdf->HasElement("z_kd") ? sdf->Get<double>("z_kd") : 30.0;
    this->z_force_max_ = sdf->HasElement("z_force_max") ? sdf->Get<double>("z_force_max") : 120.0;

    this->planar_force_gain_ = sdf->HasElement("planar_force_gain") ? sdf->Get<double>("planar_force_gain") : 8.0;
    this->planar_damping_ = sdf->HasElement("planar_damping") ? sdf->Get<double>("planar_damping") : 6.0;
    this->planar_force_max_ = sdf->HasElement("planar_force_max") ? sdf->Get<double>("planar_force_max") : 12.0;

    this->rot_kp_ = sdf->HasElement("rot_kp") ? sdf->Get<double>("rot_kp") : 80.0;
    this->rot_kd_ = sdf->HasElement("rot_kd") ? sdf->Get<double>("rot_kd") : 18.0;

    this->body_link_ = this->ResolveModelLink(this->link_name_);
    if (!this->body_link_) {
      this->body_link_ = this->model_->GetLink();
      gzerr << "[LevitationPlugin] Could not find link [" << this->link_name_
            << "], falling back to canonical link.\n";
    }
    if (!this->body_link_) {
      gzerr << "[LevitationPlugin] No valid body link found. Plugin not started.\n";
      return;
    }

    auto plate_model = this->world_->ModelByName(this->plate_model_name_);
    if (!plate_model) {
      gzerr << "[LevitationPlugin] Could not find plate model [" << this->plate_model_name_ << "].\n";
      return;
    }

    this->plate_link_ = plate_model->GetLink(this->plate_link_name_);
    if (!this->plate_link_) {
      gzerr << "[LevitationPlugin] Could not find plate link [" << this->plate_link_name_ << "].\n";
      return;
    }

    this->front_coil_link_ = this->ResolveModelLink("front_coil");
    this->back_coil_link_ = this->ResolveModelLink("back_coil");
    this->left_coil_link_ = this->ResolveModelLink("left_coil");
    this->right_coil_link_ = this->ResolveModelLink("right_coil");
    if (!this->front_coil_link_ || !this->back_coil_link_ || !this->left_coil_link_ || !this->right_coil_link_) {
      gzerr << "[LevitationPlugin] Missing one or more coil links. Required: [front_coil, back_coil, left_coil, right_coil].\n";
      const auto links = this->model_->GetLinks();
      std::string available_links;
      for (const auto & link : links) {
        if (link) {
          if (!available_links.empty()) {
            available_links += ", ";
          }
          available_links += link->GetName();
        }
      }
      gzerr << "[LevitationPlugin] Available links: [" << available_links << "]\n";
      return;
    }

    this->total_mass_ = 0.0;
    const auto & links = this->model_->GetLinks();
    for (const auto & link : links) {
      if (link && link->GetInertial()) {
        this->total_mass_ += link->GetInertial()->Mass();
      }
    }
    if (this->total_mass_ <= 0.0 && this->body_link_->GetInertial()) {
      this->total_mass_ = this->body_link_->GetInertial()->Mass();
    }

    this->gravity_mag_ = std::abs(this->world_->Gravity().Z());
    this->locked_orientation_ = this->body_link_->WorldPose().Rot();

    const ignition::math::AxisAlignedBox collision_bbox = this->body_link_->CollisionBoundingBox();
    const double collision_top_offset = collision_bbox.Max().Z() - this->body_link_->WorldCoGPose().Pos().Z();
    this->effective_module_top_offset_ = this->module_top_offset_;
    if (std::isfinite(collision_top_offset) && collision_top_offset > this->effective_module_top_offset_) {
      gzmsg << "[LevitationPlugin] module_top_offset (" << this->module_top_offset_
            << " m) is smaller than collision top offset (" << collision_top_offset
            << " m). Using collision-derived value to preserve planar mobility.\n";
      this->effective_module_top_offset_ = collision_top_offset;
    }

    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }

    const std::string node_name = this->model_->GetName() + std::string("_levitation_plugin");
    this->ros_node_ = std::make_shared<rclcpp::Node>(node_name);
    this->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->executor_->add_node(this->ros_node_);

    this->cmd_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      this->cmd_topic_, 10,
      std::bind(&LevitationPlugin::OnCmdVel, this, std::placeholders::_1));

    this->executor_thread_ = std::thread([this]() {
      this->executor_->spin();
    });

    this->last_update_time_ = this->world_->SimTime();
    this->last_debug_time_ = this->last_update_time_;
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LevitationPlugin::OnUpdate, this));

    gzmsg << "[LevitationPlugin] Loaded for model [" << this->model_->GetName()
          << "] total_mass=" << this->total_mass_ << " kg"
          << " target_gap=" << this->target_gap_ << " m"
          << " plate_size=(" << this->plate_size_x_ << ", " << this->plate_size_y_ << ") m"
          << " module_top_offset=" << this->effective_module_top_offset_ << " m"
          << " cmd_topic=" << this->cmd_topic_ << "\n";
  }

private:
  physics::LinkPtr ResolveModelLink(const std::string & link_name) const
  {
    if (!this->model_) {
      return nullptr;
    }

    auto link = this->model_->GetLink(link_name);
    if (link) {
      return link;
    }

    return this->model_->GetLink(this->model_->GetName() + "::" + link_name);
  }

  static double Clamp(double value, double min_value, double max_value)
  {
    return std::max(min_value, std::min(value, max_value));
  }

  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(this->cmd_mutex_);
    this->last_cmd_ = *msg;
  }

  void OnUpdate()
  {
    if (!this->body_link_ || !this->plate_link_ ||
        !this->front_coil_link_ || !this->back_coil_link_ ||
        !this->left_coil_link_ || !this->right_coil_link_) {
      return;
    }

    const common::Time now = this->world_->SimTime();
    const double dt = (now - this->last_update_time_).Double();
    if (dt <= 0.0) {
      return;
    }
    this->last_update_time_ = now;

    const ignition::math::Pose3d body_pose = this->body_link_->WorldCoGPose();
    const ignition::math::Vector3d linear_vel = this->body_link_->WorldLinearVel();
    const ignition::math::Vector3d angular_vel = this->body_link_->WorldAngularVel();
    const ignition::math::Vector3d body_center = body_pose.Pos();

    const ignition::math::Vector3d plate_center = this->plate_link_->WorldCoGPose().Pos();
    const double plate_center_z = plate_center.Z();
    const double plate_bottom_z = plate_center_z - 0.5 * this->plate_thickness_;
    const double target_body_z = plate_bottom_z - this->target_gap_ - this->effective_module_top_offset_;

    const double z_error = target_body_z - body_pose.Pos().Z();
    double fz_global = this->total_mass_ * this->gravity_mag_ + this->z_kp_ * z_error - this->z_kd_ * linear_vel.Z();
    fz_global = Clamp(fz_global, 0.0, this->z_force_max_);
    const double half_plate_x = 0.5 * this->plate_size_x_;
    const double half_plate_y = 0.5 * this->plate_size_y_;

    const ignition::math::Vector3d front_coil_pos = this->front_coil_link_->WorldCoGPose().Pos();
    const ignition::math::Vector3d back_coil_pos = this->back_coil_link_->WorldCoGPose().Pos();
    const ignition::math::Vector3d left_coil_pos = this->left_coil_link_->WorldCoGPose().Pos();
    const ignition::math::Vector3d right_coil_pos = this->right_coil_link_->WorldCoGPose().Pos();

    const bool front_inside = std::abs(front_coil_pos.X() - plate_center.X()) <= half_plate_x &&
      std::abs(front_coil_pos.Y() - plate_center.Y()) <= half_plate_y;
    const bool back_inside = std::abs(back_coil_pos.X() - plate_center.X()) <= half_plate_x &&
      std::abs(back_coil_pos.Y() - plate_center.Y()) <= half_plate_y;
    const bool left_inside = std::abs(left_coil_pos.X() - plate_center.X()) <= half_plate_x &&
      std::abs(left_coil_pos.Y() - plate_center.Y()) <= half_plate_y;
    const bool right_inside = std::abs(right_coil_pos.X() - plate_center.X()) <= half_plate_x &&
      std::abs(right_coil_pos.Y() - plate_center.Y()) <= half_plate_y;

    const double fz_front = front_inside ? 0.25 * fz_global : 0.0;
    const double fz_back = back_inside ? 0.25 * fz_global : 0.0;
    const double fz_left = left_inside ? 0.25 * fz_global : 0.0;
    const double fz_right = right_inside ? 0.25 * fz_global : 0.0;
    const double fz_total = fz_front + fz_back + fz_left + fz_right;

    const ignition::math::Vector3d force_front(0.0, 0.0, fz_front);
    const ignition::math::Vector3d force_back(0.0, 0.0, fz_back);
    const ignition::math::Vector3d force_left(0.0, 0.0, fz_left);
    const ignition::math::Vector3d force_right(0.0, 0.0, fz_right);
    const ignition::math::Vector3d tau_total =
      (front_coil_pos - body_center).Cross(force_front) +
      (back_coil_pos - body_center).Cross(force_back) +
      (left_coil_pos - body_center).Cross(force_left) +
      (right_coil_pos - body_center).Cross(force_right);

    geometry_msgs::msg::Twist cmd_copy;
    {
      std::lock_guard<std::mutex> lock(this->cmd_mutex_);
      cmd_copy = this->last_cmd_;
    }

    double fx = this->planar_force_gain_ * Clamp(cmd_copy.linear.x, -1.0, 1.0) - this->planar_damping_ * linear_vel.X();
    double fy = this->planar_force_gain_ * Clamp(cmd_copy.linear.y, -1.0, 1.0) - this->planar_damping_ * linear_vel.Y();

    fx = Clamp(fx, -this->planar_force_max_, this->planar_force_max_);
    fy = Clamp(fy, -this->planar_force_max_, this->planar_force_max_);

    const ignition::math::Vector3d applied_force(fx, fy, fz_total);
    this->body_link_->AddForce(applied_force);

    if ((now - this->last_debug_time_).Double() >= 1.0) {
      this->last_debug_time_ = now;
      gzmsg << "[LevitationPlugin] coils: "
            << "F=" << (front_inside ? "IN " : "OUT ") << fz_front << "N, "
            << "B=" << (back_inside ? "IN " : "OUT ") << fz_back << "N, "
            << "L=" << (left_inside ? "IN " : "OUT ") << fz_left << "N, "
            << "R=" << (right_inside ? "IN " : "OUT ") << fz_right << "N"
            << " | F_total=" << applied_force << "N"
            << " | Tau_total=" << tau_total << "Nm\n";
    }

    const ignition::math::Quaterniond q_err = this->locked_orientation_ * body_pose.Rot().Inverse();
    const ignition::math::Vector3d angle_err = q_err.Euler();
    const ignition::math::Vector3d torque(
      this->rot_kp_ * angle_err.X() - this->rot_kd_ * angular_vel.X(),
      this->rot_kp_ * angle_err.Y() - this->rot_kd_ * angular_vel.Y(),
      this->rot_kp_ * angle_err.Z() - this->rot_kd_ * angular_vel.Z());

    this->body_link_->AddTorque(torque);
  }

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr body_link_;
  physics::LinkPtr plate_link_;
  physics::LinkPtr front_coil_link_;
  physics::LinkPtr back_coil_link_;
  physics::LinkPtr left_coil_link_;
  physics::LinkPtr right_coil_link_;
  event::ConnectionPtr update_connection_;

  std::shared_ptr<rclcpp::Node> ros_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::thread executor_thread_;
  std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist last_cmd_;

  std::string link_name_;
  std::string cmd_topic_;
  std::string plate_model_name_;
  std::string plate_link_name_;

  double plate_thickness_{0.005};
  double plate_size_x_{4.0};
  double plate_size_y_{6.0};
  double module_top_offset_{0.0025};
  double effective_module_top_offset_{0.0025};
  double target_gap_{0.02};

  double z_kp_{180.0};
  double z_kd_{30.0};
  double z_force_max_{120.0};

  double planar_force_gain_{8.0};
  double planar_damping_{6.0};
  double planar_force_max_{12.0};

  double rot_kp_{80.0};
  double rot_kd_{18.0};

  double total_mass_{0.0};
  double gravity_mag_{9.81};
  ignition::math::Quaterniond locked_orientation_;
  common::Time last_update_time_;
  common::Time last_debug_time_;
};

GZ_REGISTER_MODEL_PLUGIN(LevitationPlugin)
}  // namespace gazebo
