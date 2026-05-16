#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Collision.hh>
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
    this->module_top_offset_ = sdf->HasElement("module_top_offset") ? sdf->Get<double>("module_top_offset") : 0.0025;
    this->target_gap_ = sdf->HasElement("target_gap") ? sdf->Get<double>("target_gap") : 0.02;

    this->z_kp_ = sdf->HasElement("z_kp") ? sdf->Get<double>("z_kp") : 180.0;
    this->z_kd_ = sdf->HasElement("z_kd") ? sdf->Get<double>("z_kd") : 30.0;
    this->z_force_max_ = sdf->HasElement("z_force_max") ? sdf->Get<double>("z_force_max") : 120.0;
    this->coil_force_max_ = sdf->HasElement("coil_force_max") ? sdf->Get<double>("coil_force_max") : 45.0;
    this->roll_force_sign_ = sdf->HasElement("roll_force_sign") ? sdf->Get<double>("roll_force_sign") : 1.0;
    this->pitch_force_sign_ = sdf->HasElement("pitch_force_sign") ? sdf->Get<double>("pitch_force_sign") : 1.0;

    this->planar_force_gain_ = sdf->HasElement("planar_force_gain") ? sdf->Get<double>("planar_force_gain") : 8.0;
    this->planar_damping_ = sdf->HasElement("planar_damping") ? sdf->Get<double>("planar_damping") : 6.0;
    this->planar_force_max_ = sdf->HasElement("planar_force_max") ? sdf->Get<double>("planar_force_max") : 12.0;

    this->rot_kp_ = sdf->HasElement("rot_kp") ? sdf->Get<double>("rot_kp") : 80.0;
    this->rot_kd_ = sdf->HasElement("rot_kd") ? sdf->Get<double>("rot_kd") : 18.0;
    this->attitude_torque_max_xy_ =
      sdf->HasElement("attitude_torque_max_xy") ? sdf->Get<double>("attitude_torque_max_xy") : 3.0;
    this->yaw_torque_max_ =
      sdf->HasElement("yaw_torque_max") ? sdf->Get<double>("yaw_torque_max") : 1.0;

    this->body_link_ = this->ResolveModelLink(this->link_name_);
    if (!this->body_link_) {
      this->body_link_ = this->model_->GetLink();
    }
    
    auto plate_model = this->world_->ModelByName(this->plate_model_name_);
    if (!plate_model) {
      gzerr << "[LevitationPlugin] Plate model [" << this->plate_model_name_ << "] not found.\n";
      return;
    }

    this->plate_link_ = plate_model->GetLink(this->plate_link_name_);
    if (!this->plate_link_) {
      gzerr << "[LevitationPlugin] Plate link [" << this->plate_link_name_
            << "] not found in model [" << this->plate_model_name_ << "].\n";
      return;
    }

    if (!this->BuildPlateFootprintFromCollisions()) {
      gzerr << "[LevitationPlugin] Failed to derive plate footprint from collision geometry.\n";
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
    this->next_debug_time_ = this->last_update_time_ + common::Time(1, 0);
    this->on_update_running_logged_ = false;
    this->warned_sim_time_stalled_ = false;

    std::cout << "[LevitationPlugin] Load(): model=" << this->model_->GetName()
              << " link=" << this->body_link_->GetName()
              << " plate=" << this->plate_model_name_ << "::" << this->plate_link_name_
              << " cmd_topic=" << this->cmd_topic_ << std::endl;

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LevitationPlugin::OnUpdate, this));
    if (this->update_connection_) {
      std::cout << "[LevitationPlugin] OnUpdate connected." << std::endl;
    } else {
      std::cout << "[LevitationPlugin] WARNING: failed to connect OnUpdate." << std::endl;
    }

    gzmsg << "[LevitationPlugin] Loaded for model [" << this->model_->GetName()
          << "] total_mass=" << this->total_mass_ << " kg"
          << " target_gap=" << this->target_gap_ << " m"
          << " plate_footprint=" << this->plate_footprint_.size() << "_collisions"
          << " plate_thickness=" << this->effective_plate_thickness_ << " m"
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

  static bool PointInRect(double x, double y, double cx, double cy, double sx, double sy)
  {
    return std::abs(x - cx) <= sx * 0.5 &&
      std::abs(y - cy) <= sy * 0.5;
  }

  bool BuildPlateFootprintFromCollisions()
  {
    this->plate_footprint_.clear();

    if (!this->plate_link_) {
      return false;
    }

    const auto collisions = this->plate_link_->GetCollisions();
    if (collisions.empty()) {
      return false;
    }

    const ignition::math::Pose3d plate_pose = this->plate_link_->WorldCoGPose();
    double max_collision_thickness = 0.0;

    for (const auto & collision : collisions) {
      if (!collision) {
        continue;
      }

      const ignition::math::Pose3d collision_pose = collision->WorldPose();
      const ignition::math::Vector3d collision_center_local =
        plate_pose.Rot().RotateVectorReverse(collision_pose.Pos() - plate_pose.Pos());
      const ignition::math::AxisAlignedBox bbox = collision->BoundingBox();

      PlateFootprintRect rect;
      rect.name = collision->GetName();
      rect.center_x = collision_center_local.X();
      rect.center_y = collision_center_local.Y();
      rect.size_x = bbox.XLength();
      rect.size_y = bbox.YLength();

      if (rect.size_x <= 0.0 || rect.size_y <= 0.0) {
        continue;
      }

      max_collision_thickness = std::max(max_collision_thickness, bbox.ZLength());
      this->plate_footprint_.push_back(rect);
    }

    if (this->plate_footprint_.empty()) {
      return false;
    }

    if (max_collision_thickness > 0.0) {
      this->effective_plate_thickness_ = max_collision_thickness;
      if (std::abs(this->effective_plate_thickness_ - this->plate_thickness_) > 1e-6) {
        gzmsg << "[LevitationPlugin] plate_thickness (" << this->plate_thickness_
              << " m) differs from collision-derived thickness (" << this->effective_plate_thickness_
              << " m). Using collision-derived value.\n";
      }
    } else {
      this->effective_plate_thickness_ = this->plate_thickness_;
    }

    return true;
  }

  bool PointInsideCopperPlateFootprint(double local_x, double local_y) const
  {
    for (const auto & rect : this->plate_footprint_) {
      if (PointInRect(local_x, local_y, rect.center_x, rect.center_y, rect.size_x, rect.size_y)) {
        return true;
      }
    }
    return false;
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
    if (!this->on_update_running_logged_) {
      this->on_update_running_logged_ = true;
      std::cout << "[LevitationPlugin] OnUpdate running at sim_time=" << now.Double() << " s" << std::endl;
    }

    const double dt = (now - this->last_update_time_).Double();
    if (dt <= 0.0) {
      if (!this->warned_sim_time_stalled_) {
        this->warned_sim_time_stalled_ = true;
        std::cout << "[LevitationPlugin] WARNING: simulation time is not advancing; 1 Hz debug output is paused."
                  << std::endl;
      }
      return;
    }
    this->last_update_time_ = now;

    const ignition::math::Pose3d body_pose = this->body_link_->WorldCoGPose();
    const ignition::math::Vector3d linear_vel = this->body_link_->WorldLinearVel();
    const ignition::math::Vector3d angular_vel = this->body_link_->WorldAngularVel();
    const ignition::math::Vector3d body_center = body_pose.Pos();

    const ignition::math::Pose3d plate_pose = this->plate_link_->WorldCoGPose();
    const ignition::math::Vector3d plate_center = plate_pose.Pos();
    const double plate_center_z = plate_center.Z();
    const double plate_bottom_z = plate_center_z - 0.5 * this->effective_plate_thickness_;
    const double target_body_z = plate_bottom_z - this->target_gap_ - this->effective_module_top_offset_;

    const double z_error = target_body_z - body_pose.Pos().Z();
    double fz_global = this->total_mass_ * this->gravity_mag_ + this->z_kp_ * z_error - this->z_kd_ * linear_vel.Z();
    fz_global = Clamp(fz_global, 0.0, this->z_force_max_);

    const ignition::math::Vector3d front_coil_pos = this->front_coil_link_->WorldCoGPose().Pos();
    const ignition::math::Vector3d back_coil_pos = this->back_coil_link_->WorldCoGPose().Pos();
    const ignition::math::Vector3d left_coil_pos = this->left_coil_link_->WorldCoGPose().Pos();
    const ignition::math::Vector3d right_coil_pos = this->right_coil_link_->WorldCoGPose().Pos();

    const ignition::math::Vector3d front_local =
      plate_pose.Rot().RotateVectorReverse(front_coil_pos - plate_center);
    const ignition::math::Vector3d back_local =
      plate_pose.Rot().RotateVectorReverse(back_coil_pos - plate_center);
    const ignition::math::Vector3d left_local =
      plate_pose.Rot().RotateVectorReverse(left_coil_pos - plate_center);
    const ignition::math::Vector3d right_local =
      plate_pose.Rot().RotateVectorReverse(right_coil_pos - plate_center);

    const bool front_inside = PointInsideCopperPlateFootprint(front_local.X(), front_local.Y());
    const bool back_inside = PointInsideCopperPlateFootprint(back_local.X(), back_local.Y());
    const bool left_inside = PointInsideCopperPlateFootprint(left_local.X(), left_local.Y());
    const bool right_inside = PointInsideCopperPlateFootprint(right_local.X(), right_local.Y());

    const ignition::math::Quaterniond q_err = this->locked_orientation_ * body_pose.Rot().Inverse();
    const ignition::math::Vector3d angle_err = q_err.Euler();
    ignition::math::Vector3d attitude_torque_cmd(
      this->rot_kp_ * angle_err.X() - this->rot_kd_ * angular_vel.X(),
      this->rot_kp_ * angle_err.Y() - this->rot_kd_ * angular_vel.Y(),
      this->rot_kp_ * angle_err.Z() - this->rot_kd_ * angular_vel.Z());
    attitude_torque_cmd.X() = Clamp(
      attitude_torque_cmd.X(), -this->attitude_torque_max_xy_, this->attitude_torque_max_xy_);
    attitude_torque_cmd.Y() = Clamp(
      attitude_torque_cmd.Y(), -this->attitude_torque_max_xy_, this->attitude_torque_max_xy_);
    attitude_torque_cmd.Z() = Clamp(
      attitude_torque_cmd.Z(), -this->yaw_torque_max_, this->yaw_torque_max_);

    const double base_fz = 0.25 * fz_global;
    const double arm_epsilon = 1e-4;
    const double front_back_arm =
      std::max(std::abs(front_coil_pos.X() - back_coil_pos.X()), arm_epsilon);
    const double left_right_arm =
      std::max(std::abs(left_coil_pos.Y() - right_coil_pos.Y()), arm_epsilon);

    double pitch_delta = -2.0 * attitude_torque_cmd.Y() / front_back_arm;
    double roll_delta = 2.0 * attitude_torque_cmd.X() / left_right_arm;
    pitch_delta *= this->pitch_force_sign_;
    roll_delta *= this->roll_force_sign_;

    const double pair_delta_limit = std::max(0.0, 2.0 * base_fz);
    pitch_delta = Clamp(pitch_delta, -pair_delta_limit, pair_delta_limit);
    roll_delta = Clamp(roll_delta, -pair_delta_limit, pair_delta_limit);

    double fz_front = base_fz + 0.5 * pitch_delta;
    double fz_back = base_fz - 0.5 * pitch_delta;
    double fz_left = base_fz + 0.5 * roll_delta;
    double fz_right = base_fz - 0.5 * roll_delta;

    fz_front = Clamp(fz_front, 0.0, this->coil_force_max_);
    fz_back = Clamp(fz_back, 0.0, this->coil_force_max_);
    fz_left = Clamp(fz_left, 0.0, this->coil_force_max_);
    fz_right = Clamp(fz_right, 0.0, this->coil_force_max_);

    if (!front_inside) {
      fz_front = 0.0;
    }
    if (!back_inside) {
      fz_back = 0.0;
    }
    if (!left_inside) {
      fz_left = 0.0;
    }
    if (!right_inside) {
      fz_right = 0.0;
    }

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
    const double pitch_sign_check = attitude_torque_cmd.Y() * tau_total.Y();
    const double roll_sign_check = attitude_torque_cmd.X() * tau_total.X();

    geometry_msgs::msg::Twist cmd_copy;
    {
      std::lock_guard<std::mutex> lock(this->cmd_mutex_);
      cmd_copy = this->last_cmd_;
    }

    double fx = this->planar_force_gain_ * Clamp(cmd_copy.linear.x, -1.0, 1.0) - this->planar_damping_ * linear_vel.X();
    double fy = this->planar_force_gain_ * Clamp(cmd_copy.linear.y, -1.0, 1.0) - this->planar_damping_ * linear_vel.Y();

    fx = Clamp(fx, -this->planar_force_max_, this->planar_force_max_);
    fy = Clamp(fy, -this->planar_force_max_, this->planar_force_max_);

    this->body_link_->AddForceAtWorldPosition(force_front, front_coil_pos);
    this->body_link_->AddForceAtWorldPosition(force_back, back_coil_pos);
    this->body_link_->AddForceAtWorldPosition(force_left, left_coil_pos);
    this->body_link_->AddForceAtWorldPosition(force_right, right_coil_pos);

    const ignition::math::Vector3d planar_force(fx, fy, 0.0);
    this->body_link_->AddForce(planar_force);

    const ignition::math::Vector3d yaw_torque(0.0, 0.0, attitude_torque_cmd.Z());
    this->body_link_->AddTorque(yaw_torque);

    if (now >= this->next_debug_time_) {
      do {
        this->next_debug_time_ += common::Time(1, 0);
      } while (now >= this->next_debug_time_);

      const ignition::math::Vector3d rpy = body_pose.Rot().Euler();
      std::cout << "[LevitationPlugin] coils:" << std::endl
                << "  Front Coil =" << fz_front << "N" << std::endl
                << "  Back Coil =" << fz_back << "N" << std::endl
                << "  Left Coil =" << fz_left << "N" << std::endl
                << "  Right Coil =" << fz_right << "N" << std::endl
                << "  Force Delta = "
                << "pitch_delta=" << pitch_delta << " roll_delta=" << roll_delta << std::endl
                << "  Total Force = "
                << "X axis: " << fx << "N "
                << "Y axis: " << fy << "N "
                << "Z axis: " << fz_total << "N " << std::endl
                << "  Desired Attitude Torque = "
                << "Roll/X: " << attitude_torque_cmd.X() << "Nm "
                << "Pitch/Y: " << attitude_torque_cmd.Y() << "Nm "
                << "Yaw/Z: " << attitude_torque_cmd.Z() << "Nm " << std::endl
                << "  Coil Torque = "
                << "X axis: " << tau_total.X() << "Nm "
                << "Y axis: " << tau_total.Y() << "Nm "
                << "Z axis: " << tau_total.Z() << "Nm " << std::endl
                << "  Sign Check = "
                << "pitch_product=" << pitch_sign_check << " roll_product=" << roll_sign_check << std::endl
                << "  Artificial Yaw Torque = " << yaw_torque.Z() << "Nm" << std::endl
                << "  Coil Footprint = " << std::endl
                << "    front local_x=" << front_local.X() << " local_y=" << front_local.Y()
                << " overPlate=" << (front_inside ? "true" : "false") << std::endl
                << "    back local_x=" << back_local.X() << " local_y=" << back_local.Y()
                << " overPlate=" << (back_inside ? "true" : "false") << std::endl
                << "    left local_x=" << left_local.X() << " local_y=" << left_local.Y()
                << " overPlate=" << (left_inside ? "true" : "false") << std::endl
                << "    right local_x=" << right_local.X() << " local_y=" << right_local.Y()
                << " overPlate=" << (right_inside ? "true" : "false") << std::endl;
    }
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

  struct PlateFootprintRect
  {
    std::string name;
    double center_x{0.0};
    double center_y{0.0};
    double size_x{0.0};
    double size_y{0.0};
  };

  std::vector<PlateFootprintRect> plate_footprint_;

  double plate_thickness_{0.005};
  double effective_plate_thickness_{0.005};
  double module_top_offset_{0.0025};
  double effective_module_top_offset_{0.0025};
  double target_gap_{0.02};

  double z_kp_{180.0};
  double z_kd_{30.0};
  double z_force_max_{120.0};
  double coil_force_max_{45.0};
  double roll_force_sign_{1.0};
  double pitch_force_sign_{1.0};

  double planar_force_gain_{8.0};
  double planar_damping_{6.0};
  double planar_force_max_{12.0};

  double rot_kp_{80.0};
  double rot_kd_{18.0};
  double attitude_torque_max_xy_{3.0};
  double yaw_torque_max_{1.0};

  double total_mass_{0.0};
  double gravity_mag_{9.81};
  ignition::math::Quaterniond locked_orientation_;
  common::Time last_update_time_;
  common::Time next_debug_time_;
  bool on_update_running_logged_{false};
  bool warned_sim_time_stalled_{false};
};

GZ_REGISTER_MODEL_PLUGIN(LevitationPlugin)
}  // namespace gazebo
