#!/usr/bin/env python
"""
Uses a neural network to determine the person's position on the screen and velocity.

TODO: Implement network.
"""

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import sys, select, termios, tty

key_actions = {
    'a': 'MOVE_LEFT',
    'd': 'MOVE_RIGHT',
    'w': 'MOVE_FORWARD',
    's': 'MOVE_BACKWARD',
}

class PersonTracker(object):
    @staticmethod
    def pose_from_xy(x, y):
        msg = PoseStamped(pose=Pose(position=Point(x, y, 0), orientation=Quaternion(0,0,0,0)))
        msg.header.frame_id = 'map'
        return msg

    @staticmethod
    def xy_from_pose(pose_stamped):
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        return (x, y)
    
    def __init__(self):
        rospy.init_node('tracker_net')

        self.key_pressed = None
        self.settings = termios.tcgetattr(sys.stdin)
        self.movement_magnitude = 0.3

        self.mock_point = PersonTracker.pose_from_xy(0,1)  # TODO: Once the neural net is implemented, stop publishing mock_point

        self.person_pub = rospy.Publisher("/person_position", PoseStamped, queue_size=10)
        self.person_pub.publish(self.mock_point)

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

                x, y = PersonTracker.xy_from_pose(self.mock_point)

                self.mock_point = PersonTracker.pose_from_xy(x + x_diff, y + y_diff)

                self.person_pub.publish(self.mock_point)

            else:
                print "Invalid key"

if __name__ == "__main__":
    # Print out teleop options
    print "Use the following keys to move the Neato around:"
    for k in key_actions.keys():
        print "'{}' : \t{}".format(k, key_actions[k])

    node = PersonTracker()
    node.run()