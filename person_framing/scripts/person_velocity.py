#!/usr/bin/env python
"""
Uses a neural network to determine the person's position on the screen and velocity.

TODO: Implement network.
"""

import rospy
from geometry_msgs.msg import Point
import sys, select, termios, tty

key_actions = {
    'a': 'MOVE_LEFT',
    'd': 'MOVE_RIGHT',
    'w': 'MOVE_FORWARD',
    's': 'MOVE_BACKWARD',
}

class PersonTracker(object):
    def __init__(self):
        rospy.init_node('tracker_net')

        self.key_pressed = None
        self.settings = termios.tcgetattr(sys.stdin)
        self.movement_magnitude = 0.3

        self.mock_point = Point(0,1,0)  # TODO: Once the neural net is implemented, stop publishing mock_point

        self.person_pub = rospy.Publisher("/person_position", Point, queue_size=10)


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key_pressed = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def run(self):
        # Wait for a key to be pressed. Then, update the cmd_vel.
        while self.key_pressed != '\x03': # ctrl-C
            self.getKey()

            print "Pressed this key: {}".format(self.key_pressed)
            if(self.key_pressed in key_actions.keys()):
                action = key_actions[self.key_pressed]
                x_diff = 0
                y_diff = 0

                if action == 'MOVE_LEFT':
                    x_diff = -1 * self.movement_magnitude
                elif action == 'MOVE_RIGHT':
                    x_diff = self.movement_magnitude
                elif action == 'MOVE_FORWARD':
                    y_diff = self.movement_magnitude
                elif action == 'MOVE_BACKWARD':
                    y_diff = -1 * self.movement_magnitude

                self.mock_point = Point(self.mock_point.x + x_diff, 
                    self.mock_point.y + y_diff,
                    self.mock_point.z)

                self.person_pub.publish(self.mock_point)

            else:
                print "Invalid key: {}".format(self.key_pressed)

if __name__ == "__main__":
    # Print out teleop options
    print "Use the following keys to move the Neato around:"
    for k in key_actions.keys():
        print "'{}' : \t{}".format(k, key_actions[k])

    node = PersonTracker()
    node.run()