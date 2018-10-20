#!/usr/bin/env python
"""
Uses a neural network to determine the person's position on the screen and velocity.

TODO: Implement network.
"""

import rospy
from geometry_msgs.msg import Point

key_actions = {
'a': 'MOVE_LEFT',
'd': 'MOVE_RIGHT',
'w': 'MOVE_FORWARD',
's': 'MOVE_BACKWARD',
'z': "TURN_LEFT",
'x': "TURN_RIGHT",
'0': "E_STOP"
}

class PersonTracker(object):
    def __init__(self):
        rospy.init_node('tracker_net')

        self.key_pressed = None
        self.settings = termios.tcgetattr(sys.stdin)

        self.mock_point = Point(0,1,0)  # TODO: Once the neural net is implemented, stop publishing mock_point

        self.person_pub = rospy.Publisher("/person_position", Point, queue_size=10)


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key_pressed = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    