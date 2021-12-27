#!/usr/bin/env python
import time
import os
import sys
import signal
import threading
import math
from itertools import izip_longest # TODO: python3 is zip_longest
zip_longest = izip_longest

from pymycobot.mycobot import MyCobot

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import SetBool, SetBoolResponse, Empty
import tf
import numpy as np


class MycobotInterface(object):

    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = rospy.get_param("~baud", 115200)
        self.vel_rate = rospy.get_param("~vel_rate", 32.0) # 32 bit/rad = 32
        rospy.loginfo("Connect mycobot on %s,%s" % (port, baud))
        self.mc = MyCobot(port, baud)
        self.lock = threading.Lock()

        self.joint_angle_pub = rospy.Publisher("joint_states", JointState, queue_size=5)
        self.real_angles = None
        self.joint_command_sub = rospy.Subscriber("joint_command", JointState, self.joint_command_cb)

        self.pub_end_coord = rospy.get_param("~pub_end_coord", False)
        if self.pub_end_coord:
            self.end_coord_pub = rospy.Publisher("end_coord", PoseStamped, queue_size=5)

        self.servo_srv = rospy.Service("set_servo", SetBool, self.set_servo_cb)

        # TODO: implement actionlib for gripper
        self.open_gripper_srv = rospy.Service("open_gripper", Empty, self.open_gripper_cb)
        self.close_gripper_srv = rospy.Service("close_gripper", Empty, self.close_gripper_cb)

        self.joint_as = actionlib.SimpleActionServer("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction, execute_cb=self.joint_as_cb)
        self.joint_as.start()

    def run(self):

        r = rospy.Rate(rospy.get_param("~joint_state_rate", 20.0)) # hz

        while not rospy.is_shutdown():

            # get real joint from MyCobot
            self.real_angles = self.mc.get_angles()
            if self.real_angles:
                rospy.logdebug_throttle(1.0, "get real angles from mycobot")

                msg = JointState()
                msg.header.stamp = rospy.get_rostime()

                for i, ang in enumerate(self.real_angles):
                   msg.name.append('joint' + str(i+1))
                   msg.position.append(ang / 180.0 * math.pi)
                self.joint_angle_pub.publish(msg)

            if self.pub_end_coord:
                coords = self.mc.get_coords()
                if coords:
                    msg = PoseStamped
                    msg.header.stamp = rospy.get_rostime()
                    msg.pose.position.x = coords[0]
                    msg.pose.position.y = coords[1]
                    msg.pose.position.z = coords[2]
                    q = tf.transformations.quaternion_from_euler(coords[3], coords[4], coords[5])
                    msg.poseq.quaternion.x = q[0]
                    msg.poseq.quaternion.y = q[1]
                    msg.poseq.quaternion.z = q[2]
                    msg.poseq.quaternion.w = q[3]
                    self.end_coord_pub.publish(msg)

            r.sleep()

    def joint_command_cb(self, msg):
        angles = self.real_angles
        vel = 50 # deg/s, hard-coding
        for n, p, v in zip_longest(msg.name, msg.position, msg.velocity):
            id = int(n[-1]) - 1
            if 'joint' in n and id >= 0 and id < len(angles):
                if math.fabs(p) < 190.0 / 180 * math.pi: # 190 should be  retrieved from API
                    angles[id] = p * 180 / math.pi
                else:
                    rospy.logwarn("%s exceeds the limit, %f", n, p)
            if v:
                v = v * 180 / math.pi
                if v < vel:
                    vel = v

        print(angles, vel)
        self.lock.acquire()
        self.mc.send_angles(angles, vel)
        self.lock.release()

    def set_servo_cb(self, req):
        if req.data:
            self.lock.acquire()
            self.mc.send_angles(self.real_angles, 0)
            self.lock.release()
            rospy.loginfo("servo on")
        else:
            self.lock.acquire()
            self.mc.release_all_servos()
            self.lock.release()
            rospy.loginfo("servo off")

        return SetBoolResponse(True, "")

    def open_gripper_cb(self, req):
        self.lock.acquire()
        self.mc.set_gripper_state(0, 80)
        self.lock.release()
        rospy.loginfo("open gripper")

    def close_gripper_cb(self, req):
        self.lock.acquire()
        self.mc.set_gripper_state(1, 80)
        self.lock.release()
        rospy.loginfo("close gripper")

    def joint_as_cb(self, goal):

        # Error case1
        if not self.real_angles:
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            msg = "Real joint angles are empty!"
            rospy.logerr(msg);
            self.joint_as.set_aborted(res, msg)
            return

        # Error case2
        if len(self.real_angles) != len(goal.trajectory.joint_names):
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            msg = "Incoming trajectory joints do not match the joints of the controller"
            rospy.logerr(msg);
            self.joint_as.set_aborted(res, msg)
            return

        # Error case3: make sure trajectory is not empty
        if len(goal.trajectory.points) == 0:
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            msg = "Incoming trajectory is empty"
            rospy.logerr(msg);
            self.joint_as.set_aborted(res, msg)
            return


        # correlate the joints we're commanding to the joints in the message
        durations = []

        # num_points = len(goal.trajectory.points);
        points = goal.trajectory.points
        # find out the duration of each segment in the trajectory
        durations.append(points[0].time_from_start)

        for i in range(1, len(goal.trajectory.points)):
            durations.append(points[i].time_from_start - points[i-1].time_from_start);

        # Error case4: empty
        if not points[0].positions:
             msg = "First point of trajectory has no positions"
             res = FollowJointTrajectoryResult()
             res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
             rospy.logerr(msg);
             self.joint_as.set_aborted(res, msg)
             return

        trajectory = []
        time = rospy.Time.now() + rospy.Duration(0.01)

        for i in range(len(points)):
            seg_start_time = 0;
            seg_duration = 0;
            seg = {}

            if goal.trajectory.header.stamp == rospy.Time():
                seg['start_time'] = (time + points[i].time_from_start) - durations[i]
            else:
                seg['start_time'] = (goal.trajectory.header.stamp + points[i].time_from_start) - durations[i]

            seg['end_time'] = seg['start_time'] + durations[i];

            # Checks that the incoming segment has the right number of elements.
            if len(points[i].velocities) > 0 and len(points[i].velocities) != len(goal.trajectory.joint_names):
                msg = "Command point " + str(i+1) + " has wrong amount of velocities"
                res = FollowJointTrajectoryResult()
                res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
                rospy.logerr(msg);
                self.joint_as.set_aborted(res, msg)
                return

            if len(points[i].positions) != len(goal.trajectory.joint_names):
                msg = "Command point " + str(i+1) + " has wrong amount of positions"
                res = FollowJointTrajectoryResult()
                res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
                rospy.logerr(msg);
                self.joint_as.set_aborted(res, msg)
                return

            if len(points[i].velocities) > 0:
                seg['velocities'] = points[i].velocities
            seg['positions'] = points[i].positions

            trajectory.append(seg);


        ## wait for start
        rospy.loginfo("Trajectory start requested at %.3lf, waiting...", goal.trajectory.header.stamp.to_sec())
        r = rospy.Rate(100)
        while (goal.trajectory.header.stamp - time).to_sec() > 0:
            time = rospy.Time.now()
            r.sleep()
        total_duration = sum(map(lambda du: du.to_sec(), durations))
        rospy.loginfo("Trajectory start time is %.3lf, end time is %.3lf, total duration is %.3lf", time.to_sec(),  time.to_sec() + total_duration, total_duration);


        feedback = FollowJointTrajectoryFeedback()
        feedback.joint_names = goal.trajectory.joint_names;
        feedback.header.stamp = time;

        for i, seg in enumerate(trajectory):

            if durations[i] == 0:
                rospy.logdebug("skipping segment %d with duration of 0 seconds", i);
                continue;

            target_angles =  np.array(seg['positions']) * 180 / np.pi
            actual_angles = np.array(self.real_angles);
            # workaround to solve bad joint velocity control
            if len(trajectory)  == 1: # only has the goal angle position, calculate the average velocity
                vel = (target_angles - np.array(self.real_angles)) / durations[i].to_sec() * np.pi / 180.0
                vel = int(np.max(np.abs(vel)) * self.vel_rate )
            else:
                # vel = int(np.max(np.abs(seg['velocities'])) * self.vel_rate) # theoretically, we should use the target velocity, but have bad following performance for mycobot280
                vel = 0 # zero is the max speed

            self.lock.acquire()
            self.mc.send_angles(target_angles.tolist(), vel)
            self.lock.release()

            while time.to_sec() < seg["end_time"].to_sec():

                rospy.logdebug("Current segment is %d time left %f cur time %f", i, (durations[i] - (time - seg["start_time"])).to_sec(), time.to_sec());

                # check if new trajectory was received, if so abort current trajectory execution
                # by setting the goal to the current position
                if self.joint_as.is_preempt_requested():

                    self.lock.acquire()
                    self.mc.send_angles(self.real_angles, 0)
                    self.lock.release()

                    self.joint_as.set_preempted()
                    if self.joint_as.is_new_goal_available():
                        rospy.logwarn("New trajectory received. Aborting old trajectory.");
                    else:
                        rospy.logwanr("Canceled trajectory following action");
                    return;


                if (time - feedback.header.stamp).to_sec() > 0.1: # 10 Hz

                    feedback.header.stamp = time;
                    feedback.desired.positions = (target_angles / 180 * np.pi).tolist()
                    feedback.actual.positions = (actual_angles / 180 * np.pi).tolist()
                    feedback.error.positions = ((target_angles - actual_angles) / 180 * np.pi).tolist()
                    self.joint_as.publish_feedback(feedback);


                r.sleep();
                time = rospy.Time.now();

            # Verify trajectory constraints
            for tol in goal.path_tolerance:
                index = goal.trajectory.joint_names.index(tol.name)
                pos_err = np.fabs(target_angles - actual_angles)[index]  / 180 * np.pi;

                if tol.position > 0 and pos_err > tol.position:
                    msg = "Unsatisfied position tolerance for " + tol.name + \
                          ", trajectory point"  + str(i+1) + ", " + str(pos_err) + \
                          " is larger than " + str(tol.position);


                    rospy.logwarn(msg);
                    res = FollowJointTrajectoryResult()
                    res.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                    rospy.logwarn(msg);
                    self.joint_as.set_aborted(res, msg)
                    return;

        # Checks that we have ended inside the goal constraints
        for tol in goal.goal_tolerance:
            index = goal.trajectory.joint_names.index(tol.name)

            pos_err = np.fabs(target_angles - actual_angles)[index] / 180 * np.pi
            if tol.position > 0 and pos_err > tol.positionl:
                msg = "Aborting because " + tol.name + \
                      " wound up outside the goal constraints, " + \
                      str(pos_err) + " is larger than " + str(tol.position)

                rospy.logwarn(msg);
                res = FollowJointTrajectoryResult()
                res.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                rospy.logwarn(msg);
                self.joint_as.set_aborted(res, msg)
                return;


        msg = "Trajectory execution successfully completed"
        rospy.loginfo(msg)
        res = FollowJointTrajectoryResult()
        res.error_code = FollowJointTrajectoryResult.SUCCESSFUL;
        self.joint_as.set_succeeded(res, msg);

if __name__ == "__main__":
    rospy.init_node("mycobot_topics")
    mc_inteface = MycobotInterface()
    mc_inteface.run()
    pass
