#! /usr/bin/env python3

import sys

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
import argparse
from argparse import RawTextHelpFormatter
from transforms3d.euler import euler2quat
from std_msgs.msg import String
import time

from nav_to_pose.robot_navigator import BasicNavigator, TaskResult

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

USE_GO_THROUGH_WAYPOINTS = True


def main(args=sys.argv):
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    
    parser = argparse.ArgumentParser(description="Navigate to Pose (and exit on success/error)",
                                     formatter_class=RawTextHelpFormatter)
    parser.add_argument('--start', help='Starting Pose "x, y, yaw"')
    parser.add_argument('--goal', help='Goal Pose "x, y, yaw"')
    parser.add_argument('--timeout', default=60., help='Maximum time in seconds to reach goal')
    parser.add_argument('--loop', type=lambda x: (str(x).lower() in ['true', '1', 'yes']), default=False, help='Restart goals after reaching last goal')
    parser.add_argument('--localization-mode', required=True, choices=['amcl', 'slamtoolbox', 'rtabmap', 'groundtruth'], help='Localization mode')
    parser.add_argument('--launch-navigation', default="False", help='Localization mode')
    parser.add_argument('--delay-after-first-localization', type=int, default=0., help='Delay after first localization in seconds')

    arguments = parser.parse_args(args_without_ros[1:])
        
    navigator = BasicNavigator()

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, navigator)
    
    pub_task_status = navigator.create_publisher(String, 'task_status', 10)
    
    def publish_task_status(status):
        pub_task_status.publish(String(data=f"{navigator.get_clock().now().nanoseconds};{time.time_ns()};{status}"))

    publish_task_status("Starting up")
    if not arguments.goal:
        navigator.error("Missing goal pose!")
        sys.exit(-1)

    
    if arguments.localization_mode == 'amcl':
        start_x = None
        start_y = None
        start_yaw = None
        if arguments.start:
            start_vals = arguments.start.split(',')
            if len(start_vals) != 3:
                navigator.error("Invalid start pose:", arguments.start)
                sys.exit(-1)
            try:
                start_x = float(start_vals[0])
                start_y = float(start_vals[1])
                start_yaw = float(start_vals[2])
            except ValueError:
                navigator.error("Invalid start values in:", arguments.start, "(expected floats)")
                sys.exit(-1)
        else:
            navigator.info("No start pose given, waiting for initial pose from rviz!")
            start_x, start_y, start_yaw = navigator.getInitialPose(float(arguments.timeout))
    
        if start_x is None or start_y is None or start_yaw is None:
            navigator.error(f"Invalid start position: {start_x}, {start_y}, {start_yaw}.")
            sys.exit(-1)
        navigator.info(f"Starting from: (x={start_x}, y={start_y}, yaw={start_yaw})")

    # parse goals
    goal_vals = []
    if ';' in arguments.goal:
        goal_vals = arguments.goal.split(';')
    else:
        goal_vals.append(arguments.goal)

    goals = []
    for goal_string in goal_vals:
        goal_split = goal_string.split(',')
        if len(goal_split) != 3:
            navigator.error("Invalid goal pose:", arguments.goal)
            sys.exit(-1)
        try:
            x = float(goal_split[0])
            y = float(goal_split[1])
            yaw = float(goal_split[2])
            goals.append((x,y,yaw))
        except ValueError:
            navigator.error("Invalid goal values in:", goal_split, "(expected floats)")
            sys.exit(-1)
    
    navigator.info("Navigating to:")
    for goal in goals:
        navigator.info(f"    x={goal[0]}, y={goal[1]}, yaw={goal[2]}")
    navigator.info(f"Timeout: {arguments.timeout}")
    if arguments.loop:
        navigator.info(f"Looping enabled")

    navigator.info("nav2 startup...")
    
    lifecycle_managers = []
    if arguments.localization_mode == 'amcl':
        lifecycle_managers.append("lifecycle_manager_localization")
    elif arguments.localization_mode == 'slamtoolbox':
        lifecycle_managers.append("lifecycle_manager_slam")
        
    if arguments.launch_navigation.lower() == 'true':
        lifecycle_managers.append("lifecycle_manager_navigation")       
    navigator.lifecycleStartup(lifecycle_managers)
     
    if arguments.localization_mode == 'amcl':
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = start_x
        initial_pose.pose.position.y = start_y
        quat = euler2quat(0., 0., start_yaw)
        initial_pose.pose.orientation.z = quat[3]
        initial_pose.pose.orientation.w = quat[0]
        
        wait_for_previous_pose=3
        while not navigator.initial_pose_received and wait_for_previous_pose > 0:
            navigator.info('Waiting for amcl_pose to be received...')
            rclpy.spin_once(navigator, timeout_sec=1.0)
            wait_for_previous_pose-=1
        if navigator.initial_pose_received:
            navigator.info("Initial pose was previously set. Skipping setInitialPose()")
        else:
            navigator.info("Set initial pose")
            navigator.setInitialPose(initial_pose)
        navigator.info("Waiting for amcl...")
        navigator._waitForNodeToActivate("amcl")
        navigator.info("Waiting for amcl: DONE")
    else:
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()        
     
    navigator.info("Waiting for bt_navigator...")
    try:
        navigator._waitForNodeToActivate('bt_navigator')
    except KeyboardInterrupt:
        navigator.info("Shutting down.")        
        sys.exit(1)
    navigator.info("Waiting for bt_navigator: DONE")

    goal_poses = []
    for goal in goals:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        quat = euler2quat(0., 0., goal[2])
        goal_pose.pose.orientation.z = quat[3]
        goal_pose.pose.orientation.w = quat[0]
        goal_poses.append(goal_pose)
    
    # publish_task_status("Wait for first localization")
    # while True:
    #     navigator.info('Waiting for intial map->odom transform...')
    #     if tf_buffer.can_transform('/map', '/odom', rclpy.time.Time()):
    #         break
    #     rclpy.spin_once(navigator, timeout_sec=1.0)
    # publish_task_status("First localization received")
    # if args.delay_after_first_localization:
    #     publish_task_status(f"Waiting after first localization for {args.delay_after_first_localization}s")
    #     delayed = navigator.get_clock().now() + rclpy.duration.Duration(seconds=args.delay_after_first_localization)
    #     while navigator.get_clock().now() < start:
    #         rclpy.spin_once(navigator, timeout_sec=0.01)
    
    while True:
        if len(goal_poses) == 1:
            publish_task_status("Checking path")
            path = navigator.getPath(initial_pose, goal_pose)
            publish_task_status("Checking path done")
            if not path:
                navigator.info("No valid path found.")
                sys.exit(-1)

            print("Start navigating to Pose!")
            navigator.info("Start navigating to Pose!")
            publish_task_status("Start navigating")
            navigator.goToPose(goal_pose)
        else:
            publish_task_status("Start navigating")
            # navigator.followWaypoints(goal_poses)
            navigator.goThroughPoses(goal_poses)

        i = 0
        while not navigator.isTaskComplete():
            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                if len(goal_poses) == 1:
                    navigator.info('Estimated time of arrival: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')
                    publish_task_status('Navigation time remaining: ' + '{0:.1f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9))

                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=float(arguments.timeout)):
                        navigator.warn(f'Timeout {arguments.timeout} reached. Cancelling navigation.')
                        publish_task_status(f'Navigation timed out after {arguments.timeout}')
                        navigator.cancelTask()
                else:
                    if USE_GO_THROUGH_WAYPOINTS:
                        pose = f"{feedback.current_pose.pose.position.x:.2f},{feedback.current_pose.pose.position.y:.2f}, distance_remaining={feedback.distance_remaining:.2f}, number_of_poses_remaining={feedback.number_of_poses_remaining}, number_of_recoveries={feedback.number_of_recoveries}, estimated_time_remaining={feedback.estimated_time_remaining}"
                        navigator.info(f'Current pose: {pose}')
                        publish_task_status(f'Current pose: {pose}')
                    else:
                        navigator.info(f'Current waypoint: {feedback.current_waypoint}')
                        publish_task_status(f'Current waypoint: {feedback.current_waypoint}')

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            navigator.info('Goal succeeded!')
            publish_task_status(f'Navigation finished: Success')
            if not arguments.loop:
                break
        elif result == TaskResult.CANCELED:
            navigator.info('Goal was canceled!')
            publish_task_status(f'Navigation finished: Canceled')
            sys.exit(-1)
        elif result == TaskResult.FAILED:
            navigator.info('Goal failed!')
            publish_task_status(f'Navigation finished: Failed')
            sys.exit(-2)
        else:
            navigator.info('Goal has an invalid return status!')
            publish_task_status(f'Navigation finished: Invalid')
            sys.exit(-3)

    exit(0)


if __name__ == '__main__':
    main()
