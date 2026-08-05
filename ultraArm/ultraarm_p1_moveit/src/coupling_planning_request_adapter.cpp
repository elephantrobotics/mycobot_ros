#include <cmath>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace ultraarm_p1_moveit
{
namespace
{
constexpr double J2_MIN = -18.0;
constexpr double J2_MAX = 85.0;
constexpr double J3_MIN = -1.0;
constexpr double J3_MAX = 110.0;
constexpr double ZERO_EPS_DEG = 0.1;

double snapZeroDeg(double angle_deg)
{
  return std::abs(angle_deg) < ZERO_EPS_DEG ? 0.0 : angle_deg;
}

bool validRegion(double j2_deg, double j3_deg)
{
  const double a = snapZeroDeg(j2_deg);
  const double b = snapZeroDeg(j3_deg);
  if (a < J2_MIN || a > J2_MAX || b < J3_MIN || b > J3_MAX)
    return false;

  if (a >= -18.0 && a < 0.0)
  {
    if (b >= 42.0)
      return false;
    const double cond1 =
        std::cos((-a + b) * M_PI / 180.0) - std::sin((45.0 + a) * M_PI / 180.0);
    const double cond2 = std::abs(std::cos((-a + b) * M_PI / 180.0));
    return cond1 <= 7.0 / 30.0 && cond2 >= 15.4 / 30.0;
  }
  if (a >= 0.0 && a <= 50.87)
    return std::cos((a - b) * M_PI / 180.0) >= 15.4 / 30.0;
  if (a > 50.87 && a < 76.72)
    return true;
  if (a >= 76.72 && a <= 85.0)
    return std::abs(std::cos((a - b) * M_PI / 180.0)) >= 6.89 / 30.0;
  return false;
}

bool getJointRad(const moveit_msgs::RobotState& state, const std::string& name, double& value)
{
  for (size_t i = 0; i < state.joint_state.name.size(); ++i)
  {
    if (state.joint_state.name[i] == name)
    {
      value = state.joint_state.position[i];
      return true;
    }
  }
  return false;
}

bool getGoalJointRad(const planning_interface::MotionPlanRequest& req, const std::string& name,
                     double& value)
{
  for (const auto& constraints : req.goal_constraints)
  {
    for (const auto& jc : constraints.joint_constraints)
    {
      if (jc.joint_name == name)
      {
        value = jc.position;
        return true;
      }
    }
  }
  return false;
}

bool checkJ2J3(double j2_rad, double j3_rad, const char* label)
{
  const double j2_deg = j2_rad * 180.0 / M_PI;
  const double j3_deg = j3_rad * 180.0 / M_PI;
  if (!validRegion(j2_deg, j3_deg))
  {
    ROS_WARN("CouplingPlanningRequestAdapter: invalid %s J2/J3 (%.2f, %.2f deg)",
             label, j2_deg, j3_deg);
    return false;
  }
  return true;
}

bool validateRequest(const planning_interface::MotionPlanRequest& req)
{
  double j2 = 0.0, j3 = 0.0;
  bool have_j2 = getJointRad(req.start_state, "J2", j2);
  bool have_j3 = getJointRad(req.start_state, "J3", j3);
  if (have_j2 && have_j3 && !checkJ2J3(j2, j3, "start"))
    return false;

  have_j2 = getGoalJointRad(req, "J2", j2);
  have_j3 = getGoalJointRad(req, "J3", j3);
  if (have_j2 && have_j3 && !checkJ2J3(j2, j3, "goal"))
    return false;

  return true;
}
}  // namespace

class CouplingPlanningRequestAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  // Melodic MoveIt has no initialize(); Noetic has it as pure virtual.
  // Keep empty impl without override so both distros compile.
  void initialize(const ros::NodeHandle& /*node_handle*/) {}

  bool adaptAndPlan(const planning_request_adapter::PlanningRequestAdapter::PlannerFn& planner,
                    const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req,
                    planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& added_path_index) const override
  {
    if (!validateRequest(req))
    {
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }
    return planner(planning_scene, req, res);
  }
};

}  // namespace ultraarm_p1_moveit

PLUGINLIB_EXPORT_CLASS(ultraarm_p1_moveit::CouplingPlanningRequestAdapter,
                         planning_request_adapter::PlanningRequestAdapter)
