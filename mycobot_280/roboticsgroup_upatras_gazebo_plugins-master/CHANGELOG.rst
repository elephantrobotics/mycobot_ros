^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboticsgroup_upatras_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2020-08-21)
------------------
* Rename package
* Clean up package

0.1.0 (2020-08-14)
------------------
* Added DisableLink Model Plugin
* Added sensitiveness parameter to MimicJointPlugin
* Added maxEffort parameter to MimicJoint plugin
* Added PID control capability to mimic joint plugin
* Move catkin_package macro so it is called before targets are defined.
  Fixes plugins not getting found when doing isolated builds
* Add missing setForce() call (otherwise PID option doesn't do anything)
* Support of Gazebo 7 was added
* Support all PID gain parameters, dynamic_reconfigure
  This change does the following:
  * the PID controllers will read all PID gain parameters (p, i, d, i_clamp, antiwindup, publish_state, ...)
  * a warning will be printed if none of those parameters could be found
  * it's possible to adjust the parameters using dynamic_reconfigure
* Adjust to Gazebo 8 API
  Note about the DisconnectWorldUpdateBegin: This function was deprecated
  in favor of resetting the ConnectionPtr, see here:
    https://bitbucket.org/osrf/gazebo/pull-requests/2329/deprecate-event-disconnect-connectionptr/diff
* Add fix for gazebo_ros_pkgs#612
  This issue also affects the mimic joint plugin:
    https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612
  The commit here fixes that issue for Gazebo 9. We should change the
  GAZEBO_MAJOR_VERSION check to >= 7 if the following PR gets backported
  to Gazebo 7 and 8:
    https://bitbucket.org/osrf/gazebo/pull-requests/2814/fix-issue-2111-by-providing-options-to/diff
* Add warning when triggering gazebo_ros_pkgs#612
* Update parameters
  * Default max effort to limit from sdf model
  * Default namespace to empty string
  * Fix sensitiveness calculation
* Add option to change the namespace of the pid
* Set CMP0054 for building with Gazebo9
* Use SetParam for effort limit
* Add license notice
