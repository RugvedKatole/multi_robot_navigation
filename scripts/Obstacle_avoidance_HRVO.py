#!/usr/bin/env python3
 
from posixpath import dirname
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D , Twist
from RVO import RVO_update, compute_V_des, reach
from dynamic_reconfigure.server import Server
import math
from tf.transformations import euler_from_quaternion
import rospy
from PID_control import PID_control
from multi_robot_navigation.msg import Obs, ObsData
from multi_robot_navigation.srv import NextTaskBot, NextTaskBotResponse
import networkx as nx
import rospkg

class RobotHRVO(object):
    def __init__(self, no_of_robots):
        self.node_name = rospy.get_name()
        self.namespace = rospy.get_namespace()
        rospy.loginfo("[%s] Initiazlizing " %self.node_name)
        self.total_bots = no_of_robots
        
        #pubs and subs
        self.obstacles = rospy.Subscriber('/obs_data',ObsData,self.update_obstacles)
        self.cmd_vel = rospy.Publisher("cmd_vel",Twist,queue_size=1)

        #service to get path
        self.next_task_service = rospy.ServiceProxy("/bot_next_task",NextTaskBot )

        #
        self.route = []
        self.last_node = "0"
        
        #robot status
        self.bot_odom = [Odometry() for i in range(self.total_bots)]
        # self.cmd_vel = [Twist() for i in range(self.total_bots)]
        self.yaw = [0 for i in range(self.total_bots)]

        #initializing PID (tracking control law)
        self.PID = PID_control(self.namespace)
        
        #get the graph
        self.graph = nx.read_graphml(rospkg.RosPack().get_path('multi_robot_navigation') +"/"+ rospy.get_param("/graph")+".graphml")
        #initializing HRVO env
        self.ws_model = dict()
        self.ws_model["robot_radius"] = 0.2
        self.ws_model["circular_obstacles"] = []
        self.ws_model['boundary'] = []

        self.goal_nodes = rospy.get_param('/init_locations').split(' ')

        self.position = []
        self.velocity = [[0,0] for i in range(self.total_bots)]
        self.velocity_detect = [[0,0] for i in range(self.total_bots)]
        self.v_max = [0.1 for i in range(self.total_bots)]

        self.delta_t = 0.2
        self.cur_bot_id_indx = 0
        self.timer = rospy.Timer(rospy.Duration(self.delta_t),self.hrvo)

    
    def update_walk(self):
        print("update_walk")
        rospy.wait_for_service('/bot_next_task')
        print("updating walk")
        task_update = self.next_task_service(rospy.get_time(),self.namespace,self.last_node)
        self.route = task_update.task
        self.last_node = task_update.task[-1]

    def update_goal(self):
        self.last_node = self.goal_nodes[self.cur_bot_id_indx]
        if len(self.route)==0:
            self.update_walk()
        self.goal_nodes[self.cur_bot_id_indx] = self.route.pop(0)
        print("update Goal")
        

    def get_goal(self):
        goal = [[0,0] for i in range(self.total_bots)]
        goal[self.cur_bot_id_indx]=[self.graph.nodes[self.goal_nodes[self.cur_bot_id_indx]]["x"]/100-2, self.graph.nodes[self.goal_nodes[self.cur_bot_id_indx]]["y"]/100-2] 
        return goal

    def update_obstacles(self,data):
        bot_id = data.bot_id
        odoms = data.obstacles
        self.cur_bot_id_indx = bot_id.index("/bot_0/")
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

    def hrvo(self, event):
        self.update_all()
        
        goal = self.get_goal()
        v_des = compute_V_des(self.position, goal,self.v_max)
        #rospy.wait_for_service("Next_goal_location",)

        self.velocity = RVO_update(self.position, v_des, self.velocity_detect,self.ws_model)
        # print(self.velocity,self.namespace)
            # param_point = self.point_generator(self.velocity[i][0],self.velocity[i][0])
        cmd_vel = self.PID.Velocity_tracking_law(self.velocity[self.cur_bot_id_indx][0],self.velocity[self.cur_bot_id_indx][1])
        self.cmd_vel.publish(cmd_vel)
        # if reach(self.position[self.cur_bot_id_indx],goal[self.cur_bot_id_indx]):
        if v_des[self.cur_bot_id_indx] == [0,0] or reach(self.position[self.cur_bot_id_indx],goal[self.cur_bot_id_indx],0.3):
            cmd_vel = self.PID.Velocity_tracking_law(0,0)
            self.cmd_vel.publish(cmd_vel)
            self.update_goal()
            print("updating goal")
            goal = self.get_goal()
            # print(self.PID[i].name)

if __name__ == "__main__":
    rospy.init_node("Obstacle_avoidance_HRVO")
    avoid = RobotHRVO(4)
    rospy.spin()




    



        