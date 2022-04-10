#!/usr/bin/env python3

from sympy import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D , Twist
from RVO import RVO_update, compute_V_des, reach
from dynamic_reconfigure.server import Server
import math
from tf.transformations import euler_from_quaternion
import rospy
from PID_control import PID_control
from multi_robot_navigation.msg import Obs, ObsData

class RobotHRVO(object):
    def __init__(self, no_of_robots):
        self.node_name = rospy.get_name()
        self.namespace = rospy.get_namespace()
        rospy.loginfo("[%s] Initiazlizing " %self.node_name)
        self.total_bots = no_of_robots
        
        #pubs and subs
        self.obstacles = rospy.Subscriber('/obs_data',ObsData,self.update_obstacles)
        self.cmd_vel = rospy.Publisher(self.namespace + "cmd_vel",Twist,queue_size=1)
        # self.cmd_pub0 = rospy.Publisher("/tb3_0/cmd_vel",Twist,queue_size=1)
        # self.odom_sub0 =  rospy.Subscriber("/tb3_0/odom", Odometry, self.odom_update_0,queue_size=1)

        # self.cmd_pub1 = rospy.Publisher("/tb3_1/cmd_vel",Twist,queue_size=1)
        # self.odom_sub1 = rospy.Subscriber("/tb3_1/odom", Odometry, self.odom_update_1,queue_size=1)
        
        # self.cmd_pub2 = rospy.Publisher("/tb3_2/cmd_vel",Twist,queue_size=1)
        # self.odom_sub2 =  rospy.Subscriber("/tb3_2/odom", Odometry, self.odom_update_2,queue_size=1)

        # self.cmd_pub3 = rospy.Publisher("/tb3_3/cmd_vel",Twist,queue_size=1)
        # self.odom_sub3 = rospy.Subscriber("/tb3_3/odom", Odometry, self.odom_update_3,queue_size=1)


        
        #robot status
        self.bot_odom = [Odometry() for i in range(self.total_bots)]
        # self.cmd_vel = [Twist() for i in range(self.total_bots)]
        self.yaw = [0 for i in range(self.total_bots)]

        #initializing PID (tracking control law)
        self.PID = PID_control(self.namespace)

        
        #initializing HRVI env
        self.ws_model = dict()

        self.ws_model["robot_radius"] = 0.2
        self.ws_model["circular_obstacles"] = []
        self.ws_model['boundary'] = []

        self.bot1_goal = [2,2]
        self.bot2_goal = [0,2]
        self.bot3_goal = [0,0]
        self.bot4_goal = [2,0]
        self.goal = [self.bot1_goal,self.bot2_goal,self.bot3_goal, self.bot4_goal]

        self.position = []
        self.velocity = [[0,0] for i in range(self.total_bots)]
        self.velocity_detect = [[0,0] for i in range(self.total_bots)]
        self.v_max = [0.2 for i in range(self.total_bots)]

        self.delta_t = 0.2
        self.cur_bot_id_indx = 0
        self.timer = rospy.Timer(rospy.Duration(self.delta_t),self.hrvo)

    def hrvo(self, event):
        self.update_all()
        v_des = compute_V_des(self.position, self.goal,self.v_max)
        self.velocity = RVO_update(self.position, v_des, self.velocity_detect,self.ws_model)
        # print(self.velocity)

            # param_point = self.point_generator(self.velocity[i][0],self.velocity[i][0])
        cmd_vel = self.PID.Velocity_tracking_law(self.velocity[0][0],self.velocity[0][1])
        self.cmd_vel.publish(cmd_vel)
            # print(self.PID[i].name)

    # def point_generator(self,vx,vy):
    #     param_points= Pose2D
    #     param_points.x = vx*self.delta_t 
    #     param_points.x = vy*self.delta_t
    #     param_points.theta = 0
    #     print(param_points)
    #     return param_points

    # def process_ang_dis(self,vx,vy,yaw):
    #     dest_yaw = math.atan2(vy,vx)
    #     if dest_yaw > yaw:
    #         right_yaw = dest_yaw-yaw
    #         left_yaw = (2*math.pi-right_yaw)*-1
    #     else:
    #         left_yaw = dest_yaw-yaw
    #         right_yaw = (2*math.pi-(left_yaw*-1))
    #     #-1 < angle < 1 , find close side to turn
    #     angle = left_yaw if abs(left_yaw) < abs(right_yaw) else right_yaw
    #     angle = angle/math.pi
    #     dis = (((1 - abs(angle))-0.5) * 2) * pow(vx*vx + vy*vy,0.5)
    #     # what is the first term in above equation ??
    #     dis = max(min(dis,1),-1)
    #     angle = max(min(angle,1),0)
    #     return dis , angle


    def update_obstacles(self,data):
        bot_id = data.bot_id
        odoms = data.obstacles
        self.cur_bot_id_indx = bot_id.index(self.namespace)
        bot_odom = odoms.pop(self.cur_bot_id_indx)
        odoms.insert(0,bot_odom)
        self.bot_odom = odoms


    def update_all(self):
        self.position = []
        for i in range(self.total_bots):
            pos = [self.bot_odom[i].pose.pose.position.x,self.bot_odom[i].pose.pose.position.y]
            self.position.append((pos))

            #update orientation
            quat = (self.bot_odom[i].pose.pose.orientation.x,
                        self.bot_odom[i].pose.pose.orientation.y,
                        self.bot_odom[i].pose.pose.orientation.z,
                        self.bot_odom[i].pose.pose.orientation.w)
            euler = euler_from_quaternion(quat)
            self.yaw[i] = euler[2]

            #update velocity
            self.velocity_detect[i] = [self.bot_odom[i].twist.twist.linear.x,self.bot_odom[i].twist.twist.linear.y]
            #print()
        goal_cur = self.goal.pop(self.cur_bot_id_indx)
        self.goal.insert(0,goal_cur)
        

    def odom_update_0(self,data):
        self.bot_odom[0] = data

    def odom_update_1(self,data):
        self.bot_odom[1] = data

    def odom_update_2(self,data):
        self.bot_odom[2] =data

    def odom_update_3(self,data):
        self.bot_odom[3] =data


if __name__ == "__main__":
    rospy.init_node("Obstacle_avoidance_HRVO")
    avoid = RobotHRVO(4)
    rospy.spin()




    



        