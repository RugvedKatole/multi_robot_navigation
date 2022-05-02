#!/usr/bin/env python3


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D , Twist
from RVO import RVO_update, compute_V_des, reach
from dynamic_reconfigure.server import Server
import math
from tf.transformations import euler_from_quaternion
import rospy
from PID_control import PID_control
from multi_robot_navigation.msg import Obs, ObsData
from mrpp_sumo.srv import NextTaskBot, NextTaskBotResponse
import networkx as nx
import rospkg


class RobotHRVO(object):
    def __init__(self, no_of_robots):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initiazlizing " %self.node_name)
        self.total_bots = no_of_robots
        
        #pubs and subs
        self.obstacles = rospy.Subscriber('/obs_data',ObsData,self.update_obstacles)
        self.cmd_pub0 = rospy.Publisher("/tb3_0/cmd_vel",Twist,queue_size=1)
        self.cmd_pub1 = rospy.Publisher("/tb3_1/cmd_vel",Twist,queue_size=1)
        self.cmd_pub2 = rospy.Publisher("/tb3_2/cmd_vel",Twist,queue_size=1)
        self.cmd_pub3 = rospy.Publisher("/tb3_3/cmd_vel",Twist,queue_size=1)


        
        #robot status
        self.bot_odom = [Odometry() for i in range(self.total_bots)]
        self.cmd_vel = [Twist() for i in range(self.total_bots)]
        self.yaw = [0 for i in range(self.total_bots)]

        #initializing PID (tracking control law)
        self.PID = PID_control("baba_mahakal")

        #initializing HRVI env
        self.ws_model = dict()

        self.ws_model["robot_radius"] = 0.2
        self.ws_model["circular_obstacles"] = []
        self.ws_model['boundary'] = []

        self.goal_nodes = rospy.get_param('/init_locations').split(' ')
        self.position = []
        self.velocity = [[0,0] for i in range(self.total_bots)]
        self.velocity_detect = [[0,0] for i in range(self.total_bots)]
        self.v_max = [0.15 for i in range(self.total_bots)]

        self.route = dict()
        self.last_node = dict()

        #get the graph
        self.graph = nx.read_graphml(rospkg.RosPack().get_path('multi_robot_navigation') +"/"+ rospy.get_param("/graph")+".graphml")

        self.delta_t = 0.02
        self.cur_bot_id_indx = 0
        self.timer = rospy.Timer(rospy.Duration(self.delta_t), self.hrvo)

    def get_goal(self):
        goal = []
        for i in range(self.total_bots):
            goal.append([self.graph.nodes[self.goal_nodes[i]]["x"]/100-2, self.graph.nodes[self.goal_nodes[i]]["y"]/100-2])
        return goal

    def update_goal(self,indx):
        self.last_node[indx] = self.goal_nodes[indx]
        print(indx)
        if len(self.route[indx])==0:
            self.update_walk(indx)
        self.goal_nodes[indx] = self.route[indx].pop(0)
    
    def update_walk(self,indx):
        rospy.wait_for_service('/bot_next_task')
        task_update = self.next_task_service(rospy.get_time(),"bot_"+str(indx),self.last_node[indx])
        self.route[indx] = task_update.task
        self.last_node[indx] = task_update.task[-1]

    def update_obstacles(self,data):
        bot_id = data.bot_id
        odoms = data.obstacles
        self.bot_odom = odoms

    def hrvo(self, event):
        self.update_all()
        goal = self.get_goal()
        v_des = compute_V_des(self.position, goal,self.v_max)
        self.velocity = RVO_update(self.position, v_des, self.velocity_detect,self.ws_model)
        self.cmd_pub0.publish(self.PID.Velocity_tracking_law(self.velocity[0][0],self.velocity[0][1]))
        self.cmd_pub1.publish(self.PID.Velocity_tracking_law(self.velocity[1][0],self.velocity[1][1]))
        self.cmd_pub2.publish(self.PID.Velocity_tracking_law(self.velocity[2][0],self.velocity[2][1]))
        self.cmd_pub3.publish(self.PID.Velocity_tracking_law(self.velocity[3][0],self.velocity[3][1]))

        for i in range(self.total_bots):
            if v_des[i] == [0,0] or reach(self.position[i],goal[i],0.1):
                self.update_goal(i)

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

            self.velocity_detect[i] = [self.bot_odom[i].twist.twist.linear.x,self.bot_odom[i].twist.twist.linear.y]

if __name__ == "__main__":
    rospy.init_node("Obstacle_avoidance_HRVO")
    avoid = RobotHRVO(4)
    rospy.spin()




    



        