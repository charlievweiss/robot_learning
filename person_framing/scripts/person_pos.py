#!/usr/bin/env python
"""
Uses a neural network to determine the person's position on the screen and velocity.

TODO: 

- Implement network. (Working on it -Charlie)
- Apply same transforms to image as used for neural net
- Obtain image from neato
    CHECK SELF.PROCESS_IMAGE
    CHECK GET_PREDICTED_POS

"""

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan, Image
import sys, select, termios, tty
from keras.models import Sequential
from keras.layers import Dense
from keras.models import model_from_json, Model
import numpy
import os
from cv_bridge import CvBridge
import cv2
import json
import rospkg

key_actions = {
    'a': 'MOVE_LEFT',
    'd': 'MOVE_RIGHT',
    'w': 'MOVE_FORWARD',
    's': 'MOVE_BACKWARD',
}

class PersonTracker(object):
    # TODO: put in helper function
    @staticmethod
    def pose_from_xy(x, y):
        msg = PoseStamped(pose=Pose(position=Point(x, y, 0), orientation=Quaternion(0,0,0,0)))
        msg.header.frame_id = 'odom'
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

        # PUBLISHERS: publish point
        self.person_pub = rospy.Publisher("/person_position", PoseStamped, queue_size=10)
        self.person_pub.publish(self.mock_point)

        # SUBSCSRIBER: get camera image
        rospy.Subscriber('camera/image_raw', Image, self.get_image)
        self.image = None
        self.cv_bridge = CvBridge() # transfers ros images to opencv (python) images
        # shows image
        cv2.namedWindow('camera image')

        # load model
        # change model directory
        r = rospkg.RosPack()
        self.model_path = r.get_path('person_framing')
        model_name = 'gnarlynet_errorfix'
        json_file = open(self.model_path+'/models/{}.json'.format(model_name), 'r')
        loaded_model_json = json_file.read()
        json_file.close()
        self.model = model_from_json(loaded_model_json)
        # load weights into new model
        self.model.load_weights(self.model_path+"/models/{}.h5".format(model_name))
        print("Loaded model from disk")

    def get_image(self, m):
        im = self.cv_bridge.imgmsg_to_cv2(m, desired_encoding="bgr8")
        self.image = im

    """def get_predicted_pos(self):
        predicted_pos = self.model.predict(image)"""

    # for hypothetical point
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key_pressed = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def run(self):
        # Wait for a key to be pressed. Then, update the cmd_vel.
        while not rospy.is_shutdown():

            """# Really scarily shows a bunch of images, but hey, we get images.
            cv2.imshow("camera image",self.image)
            key = cv2.waitKey(100)
            cv2.destroyAllWindows()"""
            

        """
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
        """

if __name__ == "__main__":
    """# Print out teleop options
    print "Use the following keys to move the Neato around:"
    for k in key_actions.keys():
        print "'{}' : \t{}".format(k, key_actions[k])"""

    node = PersonTracker()
    node.run()