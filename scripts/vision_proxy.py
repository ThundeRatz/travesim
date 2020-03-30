#!/usr/bin/env python
# coding=utf-8
"""
    File:
        vision_proxy.py

    Description:
        Simple python routine to separe the information published
    from gazebo into several topics.
        We remove any "vss_" prefixes when creating the topic name

    Params:
        - /vision/precision
        - /vision/std_dev

    Topics:
        - /vision/ball
        - /vision/robot1
        - /vision/robot2
        - /vision/robot3
        - /vision/foe1
        - /vision/foe2
        - /vision/foe3
"""

import rospy
import random
from math import sin, cos, pi
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates, ModelState

# We take out every "vss_" prefix from our models
MODELS_NAMES = ["vss_ball",
                "robot1",
                "robot2",
                "robot3",
                "foe1",
                "foe2",
                "foe3"]

pubs = {}


def clean_model_name(model):
    if model.split("_")[0] == "vss":
        model = "_".join(model.split("_")[1:])
    return model

for model in MODELS_NAMES:
    pubs[model] = rospy.Publisher("/vision/" + clean_model_name(model),
                                  ModelState, queue_size=1)


def apply_noise(data):
    precision = rospy.get_param("/vision/precision", 0)
    std_dev = rospy.get_param("/vision/std_dev", 0)

    theta = random.uniform(0, pi)
    radius = precision*random.gauss(0, std_dev)

    data.position.x += radius*cos(theta)
    data.position.y += radius*sin(theta)

    return data


def callback(data):
    """
    Function called when a message is received

    :param data: Message received in the ROS topic
    :type data: ModelStates
    """
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.name)
    for i, model in enumerate(data.name):
        if model in MODELS_NAMES:
            msg = ModelState(model_name=model,
                             pose=apply_noise(data.pose[i]),
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
