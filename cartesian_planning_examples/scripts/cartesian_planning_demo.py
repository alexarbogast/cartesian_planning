#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.wait_for_message import wait_for_message

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from cartesian_planning_msgs.msg import ErrorCodes
from cartesian_planning_msgs.srv import PlanCartesianTrajectory

NAME = "cartesian_planning_demo"
HOME = {
    "joint1": 0.0,
    "joint2": -1.125,
    "joint3": 2.275,
    "joint4": -1.15,
    "joint5": 1.571,
    "joint6": 0.0,
}


class CartesianPlanningDemo(Node):
    def __init__(self):
        super().__init__(NAME)

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
        )

        self._planning_client = self.create_client(
            PlanCartesianTrajectory,
            "cartesian_planning_server/plan_cartesian_trajectory",
        )

        self.get_logger().info("Waiting for plan_cartesian_trajectory server...")
        self._planning_client.wait_for_service()

        self.get_logger().info("Waiting for follow_trajectory_action server...")
        self._action_client.wait_for_server()

        self.get_logger().info("Ready to plan!")

    def move_home(self):
        _, start_state = wait_for_message(JointState, self, "/joint_states")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = start_state.name

        point = JointTrajectoryPoint()
        point.positions = [HOME[joint] for joint in goal.trajectory.joint_names]
        point.velocities = [0.0] * len(HOME)
        point.accelerations = [0.0] * len(HOME)
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]

        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle:
            self.get_logger().error("FollowJointTrajectory: goal rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

    def run(self):
        self.move_home()

        joint_state_topic = "/joint_states"
        _, start_state = wait_for_message(JointState, self, joint_state_topic)
        if start_state is None:
            msg = "Timed out waiting for JointState on topic: " + joint_state_topic
            self.get_logger().error(msg)
            raise RuntimeError(msg)

        request = PlanCartesianTrajectory.Request()
        request.start_state = start_state

        path = [
            (0.7, 0.3, 0.1),
            (0.7, -0.3, 0.1),
            (0.7, -0.3, 0.6),
            (0.3, -0.3, 0.6),
            (0.3, -0.3, 0.1),
            (0.3, 0.3, 0.1),
            (0.3, 0.3, 0.6),
            (0.7, 0.3, 0.6),
            (0.7, 0.3, 0.1),
        ]

        pose = Pose()
        pose.orientation.x = 0.5
        pose.orientation.y = 0.5
        pose.orientation.z = 0.5
        pose.orientation.w = 0.5

        for x, y, z in path:
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            request.path.append(deepcopy(pose))

        request.max_linear_velocity = 0.200
        request.max_angular_velocity = 1.0
        request.scaling = PlanCartesianTrajectory.Request.SCALING_FIFTH

        future = self._planning_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if not future.done():
            self.get_logger().error("Planning service call failed")
            return
        response = future.result()

        if response.error_code.val != ErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Failed to plan Cartesian trajectory. Error code: {response.error_code.val}"
            )
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = response.trajectory

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Planned trajectory goal rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("Trajectory execution completed")


if __name__ == "__main__":
    rclpy.init()
    demo = CartesianPlanningDemo()
    demo.run()
    demo.destroy_node()
    rclpy.shutdown()
