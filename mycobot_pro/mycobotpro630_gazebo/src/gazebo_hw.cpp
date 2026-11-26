#include <algorithm>
#include <atomic>
#include <cmath>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/physics/ode/ODEJoint.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

namespace pro630_gazebo
{
namespace
{
constexpr double kArmPositionGain = 35.0;  // Position error (rad) to effort (N m).
constexpr double kGripperPositionGain = 14.0;
constexpr double kMimicPositionGain = 6.0;
constexpr double kGripperDamping = 0.025;
constexpr double kUrdfArmDamping = 1.0;
constexpr double kGroundDampingHoldSec = 0.2;
constexpr double kGroundClearDelaySec = 0.1;
constexpr double kRigidSwitchSlewRate = 0.5;  // rad/s ramp when handing back to rigid tracking.
constexpr double kSettleDampingScale = 4.0;    // Extra damping while a STOP hold sheds contact force.
constexpr unsigned int kGroundCategoryBit = 0x1u;
const std::map<std::string, double> kSimulationEffortLimits = {
    {"joint1_to_base", 8.0}, {"joint2_to_joint1", 8.0}, {"joint3_to_joint2", 5.0},
    {"joint4_to_joint3", 3.0}, {"joint5_to_joint4", 2.0}, {"joint6_to_joint5", 2.0},
    {"gripper_controller", 1.2}};
// Joint damping (N m per rad/s) applied by the physics step while Force Execute
// holds the arm off the floor.
const std::map<std::string, double> kArmVelocityGains = {
    {"joint1_to_base", 2.5}, {"joint2_to_joint1", 4.0}, {"joint3_to_joint2", 2.5},
    {"joint4_to_joint3", 1.0}, {"joint5_to_joint4", 0.6}, {"joint6_to_joint5", 0.6}};
constexpr double kMimicEffortLimit = 0.5;
const char* kGroundModel = "ground_plane";

std::string modelNameOf(const gazebo::physics::Collision* collision)
{
  if (!collision)
    return {};
  const gazebo::physics::LinkPtr link = collision->GetLink();
  if (!link)
    return {};
  const gazebo::physics::ModelPtr model = link->GetModel();
  return model ? model->GetName() : std::string();
}

bool isMountedBase(const gazebo::physics::Collision* collision)
{
  if (!collision)
    return false;
  const gazebo::physics::LinkPtr link = collision->GetLink();
  if (!link)
    return false;
  const std::string name = link->GetName();
  return name == "base" || name == "world";
}

void driveLimited(const gazebo::physics::JointPtr& joint, double target, double gain, double effort_limit)
{
  const double error = target - joint->Position(0);
  const double effort = std::isfinite(error) ?
      std::max(-effort_limit, std::min(effort_limit, gain * error)) : 0.0;
  joint->SetForce(0, effort);
}
}  // namespace

class PositionRobotHWSim : public gazebo_ros_control::RobotHWSim
{
  enum class Mode
  {
    Rigid,
    Limited,
    RigidRamp
  };

  struct Joint
  {
    gazebo::physics::JointPtr sim;
    gazebo::physics::ODEJoint* ode = nullptr;
    double position = 0.0, velocity = 0.0, effort = 0.0, target = 0.0;
    double lower = 0.0, upper = 0.0;
    double effort_limit = 0.0;
    double position_gain = kArmPositionGain;
    double velocity_gain = 0.0;
    double urdf_damping = kUrdfArmDamping;
    double orig_stop_cfm = 0.0;
    double orig_stop_erp = 0.0;
    double applied_damping = -1.0;
    double switch_offset = 0.0;
    double hold_position = 0.0;
  };
  struct Mimic
  {
    gazebo::physics::JointPtr sim;
    gazebo::physics::ODEJoint* ode = nullptr;
    Joint* leader;
    double multiplier, offset;
    double lower = 0.0, upper = 0.0;
    double urdf_damping = kUrdfArmDamping;
    double orig_stop_cfm = 0.0;
    double orig_stop_erp = 0.0;
    double switch_offset = 0.0;
  };

public:
  bool initSim(const std::string&, ros::NodeHandle nh, gazebo::physics::ModelPtr model,
               const urdf::Model* const urdf,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override
  {
    std::map<std::string, double> initial;
    std::string environment = "simulation";
    model_ = model;
    nh.param<std::string>("/pro630/environment", environment, "simulation");
    allow_limited_tracking_ = (environment == "simulation");
    if (!urdf || !model_ || !nh.getParam("/pro630/initial_positions", initial))
    {
      ROS_ERROR("Pro630 Gazebo is missing its URDF or startup positions");
      return false;
    }
    for (const auto& transmission : transmissions)
    {
      if (transmission.joints_.size() != 1)
        return false;
      const auto& name = transmission.joints_.front().name_;
      const auto definition = urdf->getJoint(name);
      auto& joint = joints_[name];
      joint.sim = model->GetJoint(name);
      const auto seed = initial.find(name);
      if (!joint.sim || !definition || !definition->limits || seed == initial.end() ||
          !std::isfinite(seed->second))
      {
        ROS_ERROR_STREAM("Invalid Pro630 Gazebo joint or seed: " << name);
        return false;
      }
      joint.ode = dynamic_cast<gazebo::physics::ODEJoint*>(joint.sim.get());
      joint.lower = definition->limits->lower;
      joint.upper = definition->limits->upper;
      joint.effort_limit = std::min(definition->limits->effort, kSimulationEffortLimits.at(name));
      const auto damping = kArmVelocityGains.find(name);
      if (damping != kArmVelocityGains.end())
        joint.velocity_gain = damping->second;
      if (name == "gripper_controller")
        joint.position_gain = kGripperPositionGain;
      joint.urdf_damping = joint.sim->GetDamping(0);
      joint.orig_stop_cfm = joint.sim->GetParam("stop_cfm", 0);
      joint.orig_stop_erp = joint.sim->GetParam("stop_erp", 0);
      if (seed->second < joint.lower || seed->second > joint.upper)
        return false;
      joint.position = joint.target = seed->second;
      hardware_interface::JointStateHandle state(name, &joint.position, &joint.velocity, &joint.effort);
      states_.registerHandle(state);
      positions_.registerHandle(hardware_interface::JointHandle(state, &joint.target));
    }
    for (const auto& entry : urdf->joints_)
    {
      const auto mimic = entry.second->mimic;
      if (!mimic)
        continue;
      const auto leader = joints_.find(mimic->joint_name);
      auto sim = model->GetJoint(entry.first);
      if (!sim || leader == joints_.end())
      {
        ROS_ERROR_STREAM("Invalid Pro630 Gazebo mimic: " << entry.first);
        return false;
      }
      Mimic item{sim, dynamic_cast<gazebo::physics::ODEJoint*>(sim.get()),
                 &leader->second, mimic->multiplier, mimic->offset,
                 sim->LowerLimit(0), sim->UpperLimit(0),
                 sim->GetDamping(0), sim->GetParam("stop_cfm", 0), sim->GetParam("stop_erp", 0), 0.0};
      mimics_.push_back(item);
    }
    if (joints_.size() != 6 && joints_.size() != 7)
      return false;
    registerInterface(&states_);
    registerInterface(&positions_);
    // Seed the initial pose once, while Gazebo is still paused.
    for (const auto& entry : joints_)
    {
      const auto& joint = entry.second;
      joint.sim->SetPosition(0, joint.position, true);
      joint.sim->SetVelocity(0, 0.0);
    }
    for (const auto& mimic : mimics_)
    {
      mimic.sim->SetPosition(0, mimic.leader->position * mimic.multiplier + mimic.offset, true);
      mimic.sim->SetVelocity(0, 0.0);
    }
    get_limited_srv_ = nh.advertiseService("/pro630/get_limited_tracking",
                                           &PositionRobotHWSim::getLimitedTrackingCb, this);
    set_limited_srv_ = nh.advertiseService("/pro630/set_limited_tracking",
                                           &PositionRobotHWSim::setLimitedTrackingCb, this);
    set_hold_srv_ = nh.advertiseService("/pro630/set_limited_hold",
                                        &PositionRobotHWSim::setLimitedHoldCb, this);
    return true;
  }

  void readSim(ros::Time, ros::Duration period) override
  {
    for (auto& entry : joints_)
    {
      auto& joint = entry.second;
      const double position = joint.sim->Position(0);
      joint.velocity = period.toSec() > 0.0 ? (position - joint.position) / period.toSec() : 0.0;
      joint.position = position;
      joint.effort = joint.sim->GetForce(0);
    }
  }

  void writeSim(ros::Time time, ros::Duration) override
  {
    bool desired_limited = false;
    bool hold_requested = false;
    {
      std::lock_guard<std::mutex> lock(mode_mutex_);
      desired_limited = desired_limited_;
      hold_requested = hold_requested_;
    }

    double dt = 0.0;
    if (has_last_write_time_)
      dt = std::max(0.0, std::min(0.05, (time - last_write_time_).toSec()));
    last_write_time_ = time;
    has_last_write_time_ = true;

    if (desired_limited)
    {
      if (active_mode_ != Mode::Limited)
        enterLimitedMode();
    }
    else if (active_mode_ == Mode::Limited)
    {
      if (readyForRigidSwitch(time))
        enterRigidRamp();
    }

    if (active_mode_ == Mode::Limited)
    {
      if (hold_requested)
      {
        if (!holding_)
          engageHold();
      }
      else if (holding_)
      {
        holding_ = false;
        for (auto& entry : joints_)
          entry.second.applied_damping = -1.0;
      }
      // Ground contact uses the URDF damping so a moving arm can rest on the
      // floor. A STOP hold keeps the stronger damping so that unloading the
      // contact force settles instead of bouncing.
      const bool suppress_arm_damping = holding_ ? false : groundDampingHeld(time);
      for (auto& entry : joints_)
      {
        auto& joint = entry.second;
        const double target = std::isfinite(joint.target) && !stopped_ ? joint.target : joint.position;
        const double bounded = std::max(joint.lower, std::min(joint.upper, target));
        const double goal = holding_ ? std::max(joint.lower, std::min(joint.upper, joint.hold_position)) : bounded;
        if (joint.velocity_gain > 0.0)
        {
          const double damping = holding_ ? joint.velocity_gain * kSettleDampingScale
                                           : (suppress_arm_damping ? joint.urdf_damping : joint.velocity_gain);
          if (damping != joint.applied_damping)
          {
            joint.sim->SetDamping(0, damping);
            joint.applied_damping = damping;
          }
        }
        driveLimited(joint.sim, goal, joint.position_gain, joint.effort_limit);
      }
      for (const auto& mimic : mimics_)
      {
        const double target = mimic.leader->sim->Position(0) * mimic.multiplier + mimic.offset;
        driveLimited(mimic.sim, target, kMimicPositionGain, kMimicEffortLimit);
      }
      return;
    }

    if (active_mode_ == Mode::RigidRamp)
    {
      bool any_offset = false;
      const double step = kRigidSwitchSlewRate * dt;
      for (auto& entry : joints_)
      {
        auto& joint = entry.second;
        const double target = std::isfinite(joint.target) && !stopped_ ? joint.target : joint.position;
        const double bounded = std::max(joint.lower, std::min(joint.upper, target));
        if (joint.switch_offset != 0.0)
        {
          if (step > 0.0 && std::abs(joint.switch_offset) <= step)
            joint.switch_offset = 0.0;
          else
          {
            if (step > 0.0)
              joint.switch_offset -= std::copysign(step, joint.switch_offset);
            if (joint.switch_offset != 0.0)
              any_offset = true;
          }
        }
        const double command = std::max(joint.lower, std::min(joint.upper, bounded + joint.switch_offset));
        joint.sim->SetPosition(0, command, false);
        joint.sim->SetVelocity(0, 0.0);
      }
      for (auto& mimic : mimics_)
      {
        if (mimic.switch_offset != 0.0)
        {
          if (step > 0.0 && std::abs(mimic.switch_offset) <= step)
            mimic.switch_offset = 0.0;
          else
          {
            if (step > 0.0)
              mimic.switch_offset -= std::copysign(step, mimic.switch_offset);
            if (mimic.switch_offset != 0.0)
              any_offset = true;
          }
        }
        const double target = mimic.leader->sim->Position(0) * mimic.multiplier + mimic.offset;
        const double command = std::max(mimic.lower, std::min(mimic.upper, target + mimic.switch_offset));
        mimic.sim->SetPosition(0, command, false);
        mimic.sim->SetVelocity(0, 0.0);
      }
      if (!any_offset)
      {
        // The ramp has reached the measured pose. Drop the velocity left by
        // limited tracking before the first rigid position update.
        if (model_)
        {
          for (const auto& link : model_->GetLinks())
          {
            if (link)
              link->ResetPhysicsStates();
          }
        }
        for (auto& entry : joints_)
        {
          auto& joint = entry.second;
          const double damping =
              joint.velocity_gain > 0.0 ? joint.urdf_damping : kGripperDamping;
          joint.sim->SetDamping(0, damping);
          joint.sim->SetVelocity(0, 0.0);
        }
        for (auto& mimic : mimics_)
        {
          mimic.sim->SetDamping(0, kGripperDamping);
          mimic.sim->SetVelocity(0, 0.0);
        }
        active_mode_ = Mode::Rigid;
        ROS_INFO("Pro630 switched back to rigid position tracking");
      }
      return;
    }

    for (auto& entry : joints_)
    {
      auto& joint = entry.second;
      const double target = std::isfinite(joint.target) && !stopped_ ? joint.target : joint.position;
      const double bounded = std::max(joint.lower, std::min(joint.upper, target));
      joint.sim->SetPosition(0, bounded, true);
      joint.sim->SetVelocity(0, 0.0);
    }
    for (const auto& mimic : mimics_)
    {
      const double target = mimic.leader->sim->Position(0) * mimic.multiplier + mimic.offset;
      mimic.sim->SetPosition(0, target, true);
      mimic.sim->SetVelocity(0, 0.0);
    }
  }

  void eStopActive(bool active) override { stopped_ = active; }

private:
  bool getLimitedTrackingCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    res.success = desired_limited_ || (active_mode_ != Mode::Rigid);
    res.message = res.success ? "limited" : "rigid";
    return true;
  }

  bool setLimitedTrackingCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    if (req.data && !allow_limited_tracking_)
    {
      res.success = false;
      res.message = "Limited tracking is allowed only in simulation.";
      return true;
    }
    std::lock_guard<std::mutex> lock(mode_mutex_);
    desired_limited_ = req.data;
    // Enter limited mode holding the measured pose until the slider confirms
    // the new controller trajectory. Never track the previous rigid command.
    hold_requested_ = req.data;
    res.success = true;
    if (req.data)
      res.message = "limited";
    else
      res.message = (active_mode_ == Mode::Rigid) ? "rigid" : "switching";
    return true;
  }

  bool setLimitedHoldCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    if (req.data && !desired_limited_ && active_mode_ == Mode::Rigid)
    {
      res.success = false;
      res.message = "STOP hold is available only during limited tracking.";
      return true;
    }
    hold_requested_ = req.data;
    res.success = true;
    res.message = req.data ? "hold" : "tracking";
    return true;
  }

  void enterLimitedMode()
  {
    active_mode_ = Mode::Limited;
    holding_ = false;
    ground_seen_ = false;
    ground_touching_ = false;
    contact_free_since_valid_ = false;
    if (model_)
    {
      for (const auto& link : model_->GetLinks())
      {
        if (link)
          link->ResetPhysicsStates();
      }
    }
    // The base keeps its filter. Moving links no longer receive a ground reaction.
    suspendGroundCollision();
    for (auto& entry : joints_)
    {
      auto& joint = entry.second;
      joint.switch_offset = 0.0;
      const double initial_damping =
          joint.velocity_gain > 0.0 ? joint.velocity_gain : kGripperDamping;
      joint.sim->SetDamping(0, initial_damping);
      joint.applied_damping = joint.velocity_gain > 0.0 ? initial_damping : -1.0;
      if (joint.ode)
        joint.ode->UseImplicitSpringDamper(true);
      joint.sim->SetVelocity(0, 0.0);
    }
    for (auto& mimic : mimics_)
    {
      mimic.switch_offset = 0.0;
      mimic.sim->SetDamping(0, kGripperDamping);
      if (mimic.ode)
        mimic.ode->UseImplicitSpringDamper(true);
      mimic.sim->SetVelocity(0, 0.0);
    }
    ROS_INFO("Pro630 switched to limited tracking for Force Execute");
  }

  void engageHold()
  {
    holding_ = true;
    for (auto& entry : joints_)
    {
      auto& joint = entry.second;
      const double position = joint.sim->Position(0);
      joint.hold_position = std::isfinite(position) ? position : joint.position;
      joint.sim->SetVelocity(0, 0.0);
      if (joint.velocity_gain > 0.0)
      {
        const double damping = joint.velocity_gain * kSettleDampingScale;
        joint.sim->SetDamping(0, damping);
        joint.applied_damping = damping;
      }
    }
    for (auto& mimic : mimics_)
      mimic.sim->SetVelocity(0, 0.0);
    ROS_INFO("Pro630 holding the current pose after STOP");
  }

  bool readyForRigidSwitch(const ros::Time& time)
  {
    // Stay limited for the whole time a moving link is still inside the floor.
    // A short clear delay is only the gap after the geometry has come back out.
    if (movingLinkBelowGround() || hasMovingLinkGroundContact())
    {
      contact_free_since_valid_ = false;
      return false;
    }
    if (!contact_free_since_valid_)
    {
      contact_free_since_ = time;
      contact_free_since_valid_ = true;
      return false;
    }
    return (time - contact_free_since_).toSec() >= kGroundClearDelaySec;
  }

  void enterRigidRamp()
  {
    restoreGroundCollision();
    if (model_)
    {
      for (const auto& link : model_->GetLinks())
      {
        if (link)
          link->ResetPhysicsStates();
      }
    }
    for (auto& entry : joints_)
    {
      auto& joint = entry.second;
      if (joint.ode)
        joint.ode->UseImplicitSpringDamper(false);
      // The gripper's URDF damping is far above its inertia. Keep the limited
      // damping so the rigid handoff cannot blow the gripper links apart.
      joint.sim->SetDamping(0, joint.velocity_gain > 0.0 ? joint.urdf_damping : kGripperDamping);
      joint.sim->SetLowerLimit(0, joint.lower);
      joint.sim->SetUpperLimit(0, joint.upper);
      joint.sim->SetParam("stop_cfm", 0, joint.orig_stop_cfm);
      joint.sim->SetParam("stop_erp", 0, joint.orig_stop_erp);
      joint.sim->SetVelocity(0, 0.0);
      joint.applied_damping = -1.0;
      const double target = std::isfinite(joint.target) && !stopped_ ? joint.target : joint.position;
      const double bounded = std::max(joint.lower, std::min(joint.upper, target));
      const double live_pos = joint.sim->Position(0);
      joint.switch_offset = std::isfinite(live_pos) ? (live_pos - bounded) : 0.0;
    }
    for (auto& mimic : mimics_)
    {
      if (mimic.ode)
        mimic.ode->UseImplicitSpringDamper(false);
      mimic.sim->SetDamping(0, kGripperDamping);
      mimic.sim->SetLowerLimit(0, mimic.lower);
      mimic.sim->SetUpperLimit(0, mimic.upper);
      mimic.sim->SetParam("stop_cfm", 0, mimic.orig_stop_cfm);
      mimic.sim->SetParam("stop_erp", 0, mimic.orig_stop_erp);
      mimic.sim->SetVelocity(0, 0.0);
      const double leader_pos = mimic.leader->sim->Position(0);
      const double target = leader_pos * mimic.multiplier + mimic.offset;
      const double live_pos = mimic.sim->Position(0);
      mimic.switch_offset = (std::isfinite(live_pos) && std::isfinite(target)) ? (live_pos - target) : 0.0;
    }
    ground_seen_ = false;
    ground_touching_ = false;
    contact_free_since_valid_ = false;
    active_mode_ = Mode::RigidRamp;
  }

  bool hasMovingLinkGroundContact() const
  {
    if (!model_ || !model_->GetWorld() || !model_->GetWorld()->Physics())
      return false;
    const gazebo::physics::ContactManager* contacts = model_->GetWorld()->Physics()->GetContactManager();
    if (!contacts)
      return false;
    const std::string robot = model_->GetName();
    for (unsigned int i = 0; i < contacts->GetContactCount(); ++i)
    {
      const gazebo::physics::Contact* contact = contacts->GetContact(i);
      if (!contact || !contact->collision1 || !contact->collision2)
        continue;
      const std::string first = modelNameOf(contact->collision1);
      const std::string second = modelNameOf(contact->collision2);
      const bool robot_on_ground = (first == kGroundModel && second == robot) ||
                                   (second == kGroundModel && first == robot);
      if (!robot_on_ground)
        continue;
      const gazebo::physics::Collision* robot_collision =
          first == robot ? contact->collision1 : contact->collision2;
      if (!isMountedBase(robot_collision))
        return true;
    }
    return false;
  }

  bool movingLinkBelowGround() const
  {
    if (!model_)
      return false;
    // The mounted base rests on the floor. Only a moving link counts.
    for (const auto& link : model_->GetLinks())
    {
      const std::string name = link->GetName();
      if (name == "base" || name == "world")
        continue;
      for (const auto& collision : link->GetCollisions())
      {
        if (collision && collision->BoundingBox().Min().Z() <= 0.0)
          return true;
      }
    }
    return false;
  }

  void suspendGroundCollision()
  {
    if (ground_filters_saved_ || !model_ || !model_->GetWorld())
      return;
    const gazebo::physics::ModelPtr ground = model_->GetWorld()->ModelByName(kGroundModel);
    if (!ground)
      return;
    // Give the floor its own category bit, then remove that bit from the
    // moving links. The fixed base and any other object keep colliding.
    // Gazebo filters with the surface bitmask, so that field has to change too.
    for (const auto& link : ground->GetLinks())
    {
      if (!link)
        continue;
      for (const auto& collision : link->GetCollisions())
        rememberFilter(collision, kGroundCategoryBit, collisionFilterBits(collision));
    }
    for (const auto& link : model_->GetLinks())
    {
      if (!link)
        continue;
      const std::string name = link->GetName();
      if (name == "base" || name == "world")
        continue;
      for (const auto& collision : link->GetCollisions())
      {
        const unsigned int bits = collisionFilterBits(collision);
        rememberFilter(collision, bits, bits & ~kGroundCategoryBit);
      }
    }
    ground_filters_saved_ = !saved_collision_filters_.empty();
    if (ground_filters_saved_)
      ROS_INFO("Pro630 moving links can pass through the ground plane");
  }

  void restoreGroundCollision()
  {
    if (!ground_filters_saved_)
      return;
    for (const auto& saved : saved_collision_filters_)
    {
      if (!saved.collision)
        continue;
      if (saved.collision->GetSurface())
        saved.collision->GetSurface()->collideBitmask = saved.category_bits;
      saved.collision->SetCategoryBits(saved.category_bits);
      saved.collision->SetCollideBits(saved.collide_bits);
    }
    saved_collision_filters_.clear();
    ground_filters_saved_ = false;
    ROS_INFO("Pro630 restored ground-plane collision");
  }

  static unsigned int collisionFilterBits(const gazebo::physics::CollisionPtr& collision)
  {
    if (!collision || !collision->GetSurface())
      return 0xFFFFu;
    return collision->GetSurface()->collideBitmask;
  }

  void rememberFilter(const gazebo::physics::CollisionPtr& collision,
                      unsigned int category_bits, unsigned int collide_bits)
  {
    if (!collision)
      return;
    const unsigned int original = collisionFilterBits(collision);
    saved_collision_filters_.push_back(CollisionFilter{collision, original, original});
    if (collision->GetSurface())
      collision->GetSurface()->collideBitmask = category_bits & collide_bits;
    collision->SetCategoryBits(category_bits);
    collision->SetCollideBits(collide_bits);
  }

  bool groundDampingHeld(const ros::Time& time)
  {
    const bool touching = hasMovingLinkGroundContact();
    if (touching != ground_touching_)
    {
      ground_touching_ = touching;
      ROS_INFO_STREAM("Pro630 moving-link ground contact " << (touching ? "on" : "off"));
    }
    if (touching)
    {
      ground_seen_ = true;
      ground_left_at_ = time;
    }
    if (!ground_seen_)
      return false;
    return (time - ground_left_at_).toSec() <= kGroundDampingHoldSec;
  }

  hardware_interface::JointStateInterface states_;
  hardware_interface::PositionJointInterface positions_;
  struct CollisionFilter
  {
    gazebo::physics::CollisionPtr collision;
    unsigned int category_bits;
    unsigned int collide_bits;
  };

  gazebo::physics::ModelPtr model_;
  std::map<std::string, Joint> joints_;
  std::vector<Mimic> mimics_;
  ros::ServiceServer get_limited_srv_;
  ros::ServiceServer set_limited_srv_;
  ros::ServiceServer set_hold_srv_;
  std::mutex mode_mutex_;
  std::atomic<Mode> active_mode_{Mode::Rigid};
  bool desired_limited_ = false;
  bool hold_requested_ = false;
  bool holding_ = false;
  bool allow_limited_tracking_ = true;
  ros::Time ground_left_at_;
  ros::Time contact_free_since_;
  ros::Time last_write_time_;
  bool ground_seen_ = false;
  bool ground_touching_ = false;
  bool ground_filters_saved_ = false;
  std::vector<CollisionFilter> saved_collision_filters_;
  bool contact_free_since_valid_ = false;
  bool has_last_write_time_ = false;
  bool stopped_ = false;
};
}  // namespace pro630_gazebo

PLUGINLIB_EXPORT_CLASS(pro630_gazebo::PositionRobotHWSim, gazebo_ros_control::RobotHWSim)
