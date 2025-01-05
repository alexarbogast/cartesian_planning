from copy import deepcopy

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from cartesian_planning_msgs.msg import ErrorCodes
from cartesian_planning_msgs.srv import *

NAME = "cartesian_planning_demo"


class CartesianPlanningDemo(object):
    def __init__(self):
        self._action_client = actionlib.SimpleActionClient(
            "position_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )

        self._planning_client = rospy.ServiceProxy(
            "cartesian_planning_server/plan_cartesian_trajectory",
            PlanCartesianTrajectory,
        )

        rospy.loginfo("Waiting for plan_cartesian_trajectory server...")
        self._planning_client.wait_for_service()

        rospy.loginfo("Waiting for follow_trajectory_action server...")
        self._action_client.wait_for_server()

        rospy.loginfo("Ready to plan!")

    def run(self):
        req = PlanCartesianTrajectoryRequest()

        # set start state
        start_state = rospy.wait_for_message("/joint_states", JointState)
        req.start_state = start_state

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

        # create path
        pose = Pose()
        pose.orientation.x = 0.0
        pose.orientation.y = 0.707107
        pose.orientation.z = 0.707107
        pose.orientation.w = 0.0

        for point in path:
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = point[2]
            req.path.append(deepcopy(pose))

        # set velocity
        req.velocity = 0.200
        resp = PlanCartesianTrajectoryResponse()
        try:
            resp = self._planning_client(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"Cartesian planning service failed with exception: {e}")
            return

        if not resp.error_code.val == ErrorCodes.SUCCESS:
            rospy.logerr(
                "Failed to plan Cartesian trajectory. "
                + "Planning service returned with ERROR_CODE: "
                + str(resp.error_code.val)
            )
            return

        # send trajectory to action server
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = resp.trajectory
        self._action_client.send_goal(goal)


if __name__ == "__main__":
    rospy.init_node(NAME)

    demo = CartesianPlanningDemo()
    demo.run()
