#!/usr/bin/env python
import rospy
from tf import TransformListener
from geometry_msgs.msg import Point, Twist, Vector3, PoseStamped
from person_pos import PersonTracker
import math

"""
This takes the tracked object's (person) velocity and the size/position of the bounding box and spits out a robo velocity
TODO:
- add PID to movement
"""

class Follower(object):
    def __init__(self):
        rospy.init_node("person_follower")
        # subscribe to person_position topic to get person's 2D point
        self.person_position_sub = rospy.Subscriber('/person_position', PoseStamped, self.set_object_pose)
        self.publish_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.tf_listener = TransformListener()

        # # velocity to be published
        # self.cmd_vel = None # type: Twist message
        # self.linear_vel = 0.5 #(m/s)
        # self.angular_vel = 0.5 #(rad/s)
        # movement is dependent on:
        # self.object_pos = None #x, y - type: Point (geometry_msg)
        self.object_pose = None # PoseStamped message
        self.turning_tolerance = .05 
        # change these if the robot's too slow:
        self.max_lin_vel = .5
        self.max_ang_vel = .5

    def set_object_pose(self, msg):
        # updates whenever something is published to /person_position
        self.object_pose = msg
        # x, y = PersonTracker.xy_from_pose(msg)
        # self.object_pos = Point(x=x, y=y, z=0)

    def get_polar_baselink_object_pos(self):
        baselink = self.tf_listener.transformPose('base_link', self.object_pose)
        x = baselink.pose.position.x
        y = baselink.pose.position.y
        return math.sqrt(x**2 + y**2), math.atan2(-y, x) # radius, angle

    def calculate_distance(self):
        # calculate the distance between the bot and the person
        x, y = self.object_pose
        # x = self.object_pos.x
        # y = self.object_pos.y
        return math.sqrt(x**2+y**2)

    def calc_movement(self):
        # TODO: Move this block to init.
        target_distance = 1
        distance_tolerance = 0.1  # don't move if distance is within this range
        angle_tolerance = math.radians(1)

        object_distance, object_angle = self.get_polar_baselink_object_pos()
        print("object_distance = {}\nobject_angle  = {}".format(object_distance, object_angle))



        if object_angle > angle_tolerance:
            angular_vel = -1 * self.max_ang_vel
            linear_vel = 0.0
        elif object_angle < -1 * angle_tolerance:
            angular_vel = self.max_ang_vel
            linear_vel = 0.0
        elif object_distance > target_distance + distance_tolerance / 2:
            angular_vel = 0.0
            linear_vel = self.max_lin_vel
        elif object_distance < target_distance - distance_tolerance / 2:
            angular_vel = 0.0
            linear_vel = -1 * self.max_lin_vel
        else:
            angular_vel = 0.0
            linear_vel = 0.0

        print("""cmd_vel:
    linear  = {}
    angular = {}\n""".format(linear_vel, angular_vel))

        return (linear_vel, angular_vel)

        # # really basic, just move based on high/low
        # x = self.object_pos.x # position of person
        # object_distance = calculate_distance() # person distance away
        # # first turn, then go forward/backward
        # if x > self.turning_tolerance: # turn left
        #     self.angular_vel = self.max_ang_vel
        #     self.linear_vel = 0.0
        # elif x < -1 * self.turning_tolerance: # turn right
        #     self.angular_vel = -1*self.max_ang_vel
        #     self.linear_vel = 0.0
        # elif object_distance > 1: # go forward
        #     self.linear_vel = self.max_lin_vel
        # elif object_distance < 1: # go backward
        #     self.linear_vel = -1*self.max_lin_vel
        # else:
        #     print("Well something's wrong with your movement")

    def matt_pls(self):
        print("Let's call that a stretch goal.")

    def run(self):
        while not rospy.is_shutdown():
            # self.matt_pls()
            if self.object_pose:
                linear_vel, angular_vel = self.calc_movement()
                cmd_vel = Twist(linear=Vector3(x=linear_vel), angular=Vector3(z=angular_vel))
                self.publish_vel.publish(cmd_vel)
                #print(cmd_vel)

if __name__ == '__main__':
    node = Follower()
    node.run()

    


