#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher
from icub_ros.msg import Target


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)
    rospy.init_node(ros_node_name, anonymous=True)

    tcp_server.start({
        'target_position': RosPublisher('target_position', Target, queue_size=10),
    })

    rospy.spin()


if __name__ == "__main__":
    main()
