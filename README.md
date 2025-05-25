# Cartesian Planning

This package offers a lightweight and efficient Cartesian motion planning
library for ROS. It uses inverse-Jacobian methods to track piecewise-linear
trajectories.

## Package Overview

The planner in this package returns a joint trajectory given a list of Cartesian
poses. The planner **does not** perform collision checking -- so make sure you
simulate the output trajectory before running on hardware.

The `CartesianPlanningServer` provides access to the base `CartesianPlanner`
through a [ROS service](https://wiki.ros.org/Services) that receives a planning
request and returns the calculated joint trajectory.  See the
`cartsian_planning_examples` class for reference on how to setup and interface
the planning server.

The server requires the following ROS parameters to be loaded.

| Parameter Name        | Type     | Default Value | Description                                       |
| --------------------- | -------- | ------------- | ------------------------------------------------- |
| `position_threshold`  | `double` | `0.0005`      | Distance threshold for position convergence (m).  |
| `rotation_threshold`  | `double` | `0.01`        | Angular threshold for rotation convergence (rad). |
| `max_sampling_step`   | `double` | `0.05`        | Maximum step size along the path (m).             |
| `max_step_iterations` | `int`    | `200`         | Maximum iterations per interpolated step.         |
| `damping`             | `double` | `0.0`         | Damping factor for inverse kinematics.            |
| `robot_base_link`     | `string` |               | Name of the robot base link.                      |
| `end_effector_link`   | `string` |               | Name of the end-effector link.                    |
| `joints`              | `list`   |               | Ordered joint names of the robot.                 |

## Running the Demos

Launch the demo robot system as follows

```bash
roslaunch cartesian_planning_examples example_bringup.launch
```

The planning server can be interfaced through Python or C++. Run the demo of
your choice with the following commands.

**Python demo**

```bash
rosrun cartesian_planning_examples cartesian_planning_demo.py
```

**C++ demo**

```bash
rosrun cartesian_planning_examples cartesian_planning_demo
```
