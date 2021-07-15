#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosService, RosSubscriber
from geometry_msgs.msg import Pose
from icub_ros.srv import MoveService
from moveit_msgs.msg import RobotTrajectory


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)
    rospy.init_node(ros_node_name, anonymous=True)

    tcp_server.start({
        'target_position': RosPublisher('target_position', Pose, queue_size=10),
        'Trajectory': RosSubscriber('iCubTrajectory', RobotTrajectory, tcp_server),
        'icub_moveit_RightArm': RosService('icub_trajectory_planner/RightArm/plan_trajectory', MoveService),
        'icub_moveit_LeftEye': RosService('icub_trajectory_planner/LeftEye/plan_trajectory', MoveService),
        'icub_moveit_RightEye': RosService('icub_trajectory_planner/RightEye/plan_trajectory', MoveService),
        'icub_moveit_Head': RosService('icub_trajectory_planner/Head/plan_trajectory', MoveService),
    })

    rospy.spin()


if __name__ == "__main__":
    main()
