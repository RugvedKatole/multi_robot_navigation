#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from multi_robot_navigation.msg import ObsData, Obs
from tf.transformations import euler_from_quaternion


class Obstacles:

    def __init__(self):

        self.obs_pub = rospy.Publisher('/obs_data', ObsData, queue_size = 10)
        rospy.Subscriber('/tb3_0/odom', Odometry, self.callback_odom, '/tb3_0/')
        rospy.Subscriber('/tb3_1/odom', Odometry, self.callback_odom, '/tb3_1/')
        rospy.Subscriber('/tb3_2/odom', Odometry, self.callback_odom, '/tb3_2/')
        rospy.Subscriber('/tb3_3/odom', Odometry, self.callback_odom, '/tb3_3/')
        self.obs = {}
        self.obs['/tb3_0/'] = Odometry()
        self.obs['/tb3_1/'] = Odometry()
        self.obs['/tb3_2/'] = Odometry()
        self.obs['/tb3_3/'] = Odometry()

    def callback_odom(self, data, bot_id):
        self.obs[bot_id] = data
        # self.obs[bot_id].obs = bot_id
        # self.obs[bot_id].pose_x = data.pose.pose.position.x
        # self.obs[bot_id].pose_y = data.pose.pose.position.y
        # orient = data.pose.pose.orientation
        # (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        # if yaw < 0:
        #     yaw += 2 * math.pi
        # # self.obs[bot_id].pose.theta = yaw

        # lin_vel = data.twist.twist.linear.x
        # self.obs[bot_id].vel_x = lin_vel * math.cos(yaw)
        # self.obs[bot_id].vel_y = lin_vel * math.sin(yaw)

if __name__ == '__main__':
    rospy.init_node('collision_cone_obstacles', anonymous = True)
    s = Obstacles()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        obs = []
        bot_id = []
        for i, n in s.obs.items():
            obs.append(n)
            bot_id.append(i)

        obs_data = ObsData()
        obs_data.obstacles = obs
        obs_data.bot_id = bot_id
        print(obs_data)
        s.obs_pub.publish(obs_data)
        r.sleep() 
