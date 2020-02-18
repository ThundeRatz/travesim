#!/usr/bin/env python
# coding=utf-8
"""
    File:
        vision_proxy.py

    Description:
        Simple python routine to separe the information published
    from gazebo into several topics
"""

import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates, ModelState

MODELS = ["vss_ball",
          "robot_0",
          "robot_1",
          "robot_2",
          "foe_0",
          "foe_1",
          "foe_2"]

pubs = {}

for model in MODELS:
    pubs[model] = rospy.Publisher("/vision/" + model, ModelState, queue_size=1)


def callback(data):
    """
    Function called when a message is received

    :param data: Message received in the ROS topic
    :type data: ModelStates
    """
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.name)
    for i, model in enumerate(data.name):
        if model in MODELS:
            msg = ModelState(model_name=model,
                             pose=data.pose[i],
                             twist=data.twist[i])
            pubs[model].publish(msg)


def listener():
    """
    Main function, a simple ROS listener
    """
    rospy.init_node("gzb_proxy")
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback,
                     queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
