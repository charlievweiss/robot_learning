#!/usr/bin/env python
"""
Uses a neural network to determine the person's position on the screen

TODO: 

- low pass filter on past predictions
- black and white training images

"""

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan, Image
import sys, select, termios, tty
from keras.models import Sequential
from keras.layers import Dense
from keras.models import model_from_json, Model
import numpy as np
import os
from cv_bridge import CvBridge
import cv2
import json
import rospkg
from skimage.io import imread
from skimage.transform import resize

key_actions = {
    'a': 'MOVE_LEFT',
    'd': 'MOVE_RIGHT',
    'w': 'MOVE_FORWARD',
    's': 'MOVE_BACKWARD',
}

def is_grey(image, verbose=False, shortcut=True, portion_grey_threshold=0.15, plus_minus_deviation=3):
    """ Determine if an image is mostly grey due to wonky connection."""
    
    total_pixel = image.shape[0] * image.shape[1]
    num_required_grey_pixel = int(portion_grey_threshold * total_pixel)
    num_required_normal_pixel = int((1 - portion_grey_threshold) * total_pixel)
    
    grey_pixel_count = 0
    pixel_count = 0
    for row in range(image.shape[0]):
        for col in range(image.shape[1]):
            is_triple = True
            for channel in range(image.shape[2]):
                pixel = image[row][col][channel]
                
                # resizing image normalizes pixel values, so we need to map it back to 0-255
                if isinstance(pixel, float) and pixel < 1.0:
                    pixel *= 255 
                
                if not abs(pixel - 128) < plus_minus_deviation:
                    is_triple = False
                    continue
            pixel_count += 1
            
            if is_triple:
                grey_pixel_count += 1
                
            if shortcut:
                if grey_pixel_count > num_required_grey_pixel:
                    if verbose:
                        print("pixel_count: {}".format(pixel_count))
                        print("grey_pixel_count: {}".format(grey_pixel_count))
                        print("Enough grey pixels: {:.2f}%".format(100 * grey_pixel_count / total_pixel))
                    return True  # stop as soon as you find enough grey pixels
                elif (pixel_count - grey_pixel_count) > num_required_normal_pixel:
                    if verbose:
                        print("pixel_count: {}".format(pixel_count))
                        print("grey_pixel_count: {}".format(grey_pixel_count))
                        print("Enough normal pixels: {:.2f}%".format((100 * pixel_count - grey_pixel_count) / total_pixel))
                    return False  # or normal pixels
            
    portion_grey = grey_pixel_count * 1.0 / total_pixel
    if verbose:
        print("Portion grey: {}".format(portion_grey))

    return portion_grey > 0.6

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
        """
        Initializes:
            - node
            - point publisher
            - camera image subscriber
            - self.image (get from subscriber)
            - current image (hold one image)
            - CvBridge (converts ros images to opencv (python) images)
            - Image window
            - Neuralnet model
                - json
                - h5 (weights)
            - predicted pos
        """
        rospy.init_node('tracker_net')

        """ OLD: Used for previous digital position
        self.key_pressed = None
        self.settings = termios.tcgetattr(sys.stdin)
        self.movement_magnitude = 0.3

        self.mock_point = PersonTracker.pose_from_xy(0,1)  # TODO: Once the neural net is implemented, stop publishing mock_point
        """

        # PUBLISHERS: publish point
        self.person_pub = rospy.Publisher("/person_position", PoseStamped, queue_size=10)

        """ OLD
        self.person_pub.publish(self.mock_point)
        """

        # SUBSCSRIBER: get camera image
        rospy.Subscriber('camera/image_raw', Image, self.get_image)
        self.image = None
        self.current_image = None
        self.cv_bridge = CvBridge() # transfers ros images to opencv (python) images
        # shows image
        cv2.namedWindow('camera image')

        # LOAD MODEL
        # change model directory
        r = rospkg.RosPack()
        self.model_path = r.get_path('person_framing')
        model_name = 'neuralnet_conv_maxpool_dense'
        json_file = open(self.model_path+'/models/{}.json'.format(model_name), 'r')
        loaded_model_json = json_file.read()
        json_file.close()
        self.model = model_from_json(loaded_model_json)
        # load weights into new model
        self.model.load_weights(self.model_path+"/models/{}.h5".format(model_name))
        print("Loaded model from disk")

        # predicted pos
        self.predicted_pos = None

    # constantly updates current image from camera subscriber and converts to python image/np array
    def get_image(self, m):
        im = np.array(self.cv_bridge.imgmsg_to_cv2(m, desired_encoding="bgr8"))
        self.image = im

    # Runs the net on the image if valid img, returns pos (0,0) if not
    def get_predicted_pos(self,img):
        if img is None:
            print("No image")
            return (0,0)
        img = self.resize_img(img)
        x = 0
        y = 0
        img_array = [img]
        img_array = np.array(img_array)
        #print(img_array.shape)
        if not is_grey(img):
            predicted_pos = self.model.predict(img_array)
            x = predicted_pos[0,0]
            y = predicted_pos[0,1]
        else:
            print("Grey image ignored")
        return x,y

    # downsamples image
    def resize_img(self,img):
        resize_factor = 8
        resized_image = resize(img, (int(480/resize_factor), int(640/resize_factor)))
        return resized_image

    """ OLD:
    # for hypothetical point
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key_pressed = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    """

    def run(self):
        while not rospy.is_shutdown():
            # get single image to do stuff to
            self.current_image = self.image
            # Get array x,y for predicted pos
            x,y = self.get_predicted_pos(self.current_image)
            # Turn into PoseStamped
            self.predicted_pos = PersonTracker.pose_from_xy(x,y)
            self.person_pub.publish(self.predicted_pos)
            print(self.predicted_pos)

            """# Really scarily shows a bunch of images, but hey, we get images.
            cv2.imshow("camera image",self.image)
            key = cv2.waitKey(100)
            cv2.destroyAllWindows()"""
            

        """ OLD:
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
    """ OLD:
    # Print out teleop options
    print "Use the following keys to move the Neato around:"
    for k in key_actions.keys():
        print "'{}' : \t{}".format(k, key_actions[k])"""

    node = PersonTracker()
    node.run()