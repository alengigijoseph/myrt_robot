#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from energy_tracker.energy_tracker.detect_energy import DetectEnergy
from energy_tracker.energy_tracker.follow_energy import FollowEnergy
from std_msgs.msg import String

def main():
    rclpy.init()
    energy_node = rclpy.create_node('energy_type_publisher')

    energy_type_pub = energy_node.create_publisher(String, '/energy_type', 10)

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.697578
    initial_pose.pose.position.y = -0.133036
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 2.684054
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 10.15
    goal_pose1.pose.position.y = -0.77
    goal_pose1.pose.orientation.w = 1.0
    goal_pose1.pose.orientation.z = 0.0
    navigate_to_pose_and_detect_energy(navigator,goal_pose1,energy_type_pub,'yellow')
    

    """ goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 17.86
    goal_pose2.pose.position.y = -0.77
    goal_pose2.pose.orientation.w = 1.0
    goal_pose2.pose.orientation.z = 0.0
    navigate_to_pose_and_detect_energy(navigator,goal_pose2, green_tuning_params)
 

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 21.58
    goal_pose3.pose.position.y = -3.5
    goal_pose3.pose.orientation.w = 1.0
    goal_pose3.pose.orientation.z = 0.0
    navigate_to_pose_and_detect_energy(navigator,goal_pose3, green_tuning_params) """


def navigate_to_pose_and_detect_energy(navigator,goal_pose,pub,color):

    publish_energy_type(pub, color)

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = 0.0
                goal_pose.pose.position.y = 0.0
                navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        follow_ball = FollowEnergy()
        rclpy.spin(follow_ball)
        follow_ball.destroy_node()
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)

def publish_energy_type(publisher, energy_type):
    msg = String()
    msg.data = energy_type
    publisher.publish(msg)
    print(f'Published energy type: {msg.data}')

if __name__ == '__main__':
    main()