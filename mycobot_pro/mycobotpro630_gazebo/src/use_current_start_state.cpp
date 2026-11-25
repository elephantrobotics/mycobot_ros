#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <pluginlib/class_list_macros.h>

namespace pro630_gazebo
{
// RViz's query start can remain at the launch pose while the monitored robot
// moves. Resolve every new plan against the scene snapshot acquired by MoveIt
// for that request, without changing the user's goal or the collision scene.
class UseCurrentStartState : public planning_request_adapter::PlanningRequestAdapter
{
public:
  void initialize(const ros::NodeHandle&) override
  {
  }

  std::string getDescription() const override
  {
    return "Use the current Pro630 planning-scene state as the start";
  }

  bool adaptAndPlan(const PlannerFn& planner,
                    const planning_scene::PlanningSceneConstPtr& scene,
                    const planning_interface::MotionPlanRequest& request,
                    planning_interface::MotionPlanResponse& response,
                    std::vector<std::size_t>&) const override
  {
    auto current_request = request;
    // An empty diff means inherit the current scene state, including mimic
    // joints, multi-DOF transforms and attached bodies. A full empty state
    // would instead reset joints or lose attached-body information.
    current_request.start_state = moveit_msgs::RobotState();
    current_request.start_state.is_diff = true;
    return planner(scene, current_request, response);
  }
};
}  // namespace pro630_gazebo

PLUGINLIB_EXPORT_CLASS(pro630_gazebo::UseCurrentStartState,
                      planning_request_adapter::PlanningRequestAdapter)
