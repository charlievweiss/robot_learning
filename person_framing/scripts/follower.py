#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Twist
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
        rospy.Subscriber('/person_position',Point,self.get_object_pos)
        self.publish_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # velocity to be published
        self.cmd_vel = None # type: Twist message
        self.linear_vel = 0.5 #(m/s)
        self.angular_vel = 0.5 #(rad/s)
        # movement is dependent on:
        self.object_pos = None #x, y - type: Point (geometry_msg)
        self.object_distance = 0.0
        self.turning_tolerance = .05 
        # change these if the robot's too slow:
        self.max_lin_vel = .5
        self.max_ang_vel = .5

    def get_object_pos(self, p):
        # updates whenever something is published to /person_position
        self.object_pos = p

    def calculate_distance(self):
        # calculate the distance between the bot and the person
        x = self.object_pos.x
        y = self.object_pos.y
        self.object_distance = math.sqrt(x**2+y**2)

    def calc_movement(self):
        # really basic, just move based on high/low
        x = self.object_pos.x # position of person
        self.calculate_distance() # person distance away
        # first turn, then go forward/backward
        if x > self.turning_tolerance: # turn left
            self.angular_vel = self.max_ang_vel
            self.linear_vel = 0.0
        elif x < self.turning_tolerance: # turn right
            self.angular_vel = -1*self.max_ang_vel
            self.linear_vel = 0.0
        elif self.object_distance > 1: # go forward
            self.linear_vel = self.max_lin_vel
        elif self.object_distance < 1: # go backward
            self.linear_vel = -1*self.max_lin_vel
        else:
            print("Well something's wrong with your movement")

    def matt_pls(self):
        print("Let's call that a stretch goal.")

    def run(self):
        while not rospy.is_shutdown():
            # self.matt_pls()
            self.calc_movement()
            self.cmd_vel = Twist(linear=Vector3(x=self.linear_vel), angular=Vector3(z=self.angular_vel)))
            self.publish_vel.publish(self.cmd_vel)

if __name__ == '__main__':
    node = Follower()
    node.run()

    


