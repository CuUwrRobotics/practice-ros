#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def callback(data):
    """This is called every time chatter has new data sent to it"""
    # data will be a ROS message string type, meaning there is a field inside
    # of data, also called data. hence `data.data`.
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listener():
    """Set up ROS and wait for data"""
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('py_listener')

    # Tell ROS that this is a subscriber to the chatter topic, and wants to have
    # callback called when it happens.
    rospy.Subscriber("chatter", String, callback)

    print('Waiting for messages...')

    # spin() simply keeps python from exiting until this node is stopped manually
    # This will loop infinitely. You can also use spinOnce() to only run once,
    # in case you need other code running
    rospy.spin()


if __name__ == '__main__':
    listener()
