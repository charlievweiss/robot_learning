#!/usr/bin/env python
import rospy


"""
This takes the tracked object's (person) velocity and the size/position of the bounding box and spits out a robo velocity
"""

class Follower(object):
    def __init__(self):
        rospy.init_node("person_follower")
        # predicted velocity
        self.cmd_vel = None
        self.object_pos = None #x, y

    def matt_pls(self):
        print("Let's call that a stretch goal.")

    def run(self):
        while not rospy.is_shutdown():
            self.matt_pls()

if __name__ == '__main__':
    node = Follower()
    node.run()

    


