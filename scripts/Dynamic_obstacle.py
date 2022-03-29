#!/usr/bin/env python3

from ast import Return
from cmath import inf
import numpy as np 
from numpy import arccos, cos, sin, arcsin, arctan, arctan2, tan,sqrt
from geometry_msgs.msg import Pose2D,  Twist
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sympy import Intersection, Interval, S, Union
from PID_control import PID_control
# """Collision avoidance for dynamic obstacle with contant velocity
# Input variable: Rr and Ro (radius of robot and Obstacle respectively)
# alpha, beta:  velocities of robot and obstacle
# and obviously odometry of them"""
pi = np.pi
inf = np.inf
class dynamic_obstacle():
    def __init__(self,Ro,v=0.1):
        self.object_radius =  Ro
        # self.location = Pose2D
        # self.prev_location = Pose2D
        self.x = 0
        self.y = 0
        self.theta = 0

        self.prev_x = 0
        self.prev_y = 0
        self.prev_theta = 0

        self.tic = 0.0
        self.Vx = 0.0
        self.Vy = 0.0 
        self.V = v
    
    def locate(self,data):
        """ updates position of the object """
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        orient = data.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.theta = yaw
        # self.Vx = (self.x - self.prev_x)/(rospy.get_time()-self.tic)
        # self.Vy = (self.y - self.prev_y)/(rospy.get_time()-self.tic)
        self.prev_x = self.x
        self.prev_y = self.y
        self.tic = rospy.get_time()

        # self.V = np.sqrt(self.Vx**2 + self.Vy**2)
        self.V = data.twist.twist.linear.x

class robot():
    def __init__(self,Ro,cmd_vel,obst_odom):
        self.object_radius =  Ro
        self.obs_object_radius = Ro
        # self.location = Pose2D
        # self.prev_location = Pose2D
        self.x = 0
        self.y = 0
        self.theta = 0
        self.cmd_pub = rospy.Publisher(cmd_vel,Twist,queue_size=1)
        
        self.prev_x = 0
        self.prev_y = 0
        self.prev_theta = 0

        self.tic = 0.0
        self.Vx = 0.0
        self.Vy = 0.0 
        self.V = 0.1

        self.obs_Vx = 0.0
        self.obs_Vy = 0.0 
        self.obs_y = 0.0
        self.obs_x = 0.0
        self.obs_V = 0.0
        rospy.Subscriber(obst_odom,Odometry,self.locate_obstacle)
        

    def locate(self,data):
        """ updates position of the object """
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        orient = data.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.theta = yaw
        # self.Vx = (self.x - self.prev_x)/(rospy.get_time()-self.tic)
        # self.Vy = (self.y - self.prev_y)/(rospy.get_time()-self.tic)
        # self.prev_x = self.x
        # self.prev_y = self.y
        # self.tic = rospy.get_time()

        # self.V = np.sqrt(self.Vx**2 + self.Vy**2)
        self.V = data.twist.twist.linear.x

    def locate_obstacle(self,data):
        self.obs_x = data.pose.pose.position.x
        self.obs_y = data.pose.pose.position.y
        orient1 = data.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orient1.x, orient1.y, orient1.z, orient1.w])
        self.Obs_theta = yaw
        # self.Vx = (self.x - self.prev_x)/(rospy.get_time()-self.tic)
        # self.Vy = (self.y - self.prev_y)/(rospy.get_time()-self.tic)
        # self.prev_x = self.x
        # self.prev_y = self.y
        # self.tic = rospy.get_time()

        # self.V = np.sqrt(self.Vx**2 + self.Vy**2)
        self.obs_V = data.twist.twist.linear.x


# """Collision avoidance for dynamic obstacle with contant velocity
# Input variable: Rr and Ro (radius of robot and Obstacle respectively)
# alpha, beta: direction of velocities of robot and obstacle
# and obviously odometry of them"""

    def collision_cone(self,x,y,tolerance=0.15):
    # def collision_cone(v,ro,theta0,beta,R):
        """ Provides collision cone for a obstacle avoidance"""
        R = self.object_radius + self.obs_object_radius + tolerance
        alpha = np.arctan2(self.Vy, self.Vx)
        beta = np.arctan2(self.obs_Vy, self.obs_Vx)
        theta0 = np.arctan2((self.obs_y - self.y),(self.obs_x - self.x))
        ro = np.sqrt((self.x-self.obs_x)**2 + (self.y - self.obs_y)**2)

        # # # # refer eqn 25, 26 Chakravarthy and Ghose 1998
        v = self.obs_V/self.V
        u = beta - theta0
        v_pref = arctan2(y-self.y,x-self.x)
        print(v_pref)
        try:
            if ro > R:
                p = R/np.sqrt(ro**2 - R**2)
                if cos(u)/v >= 1:
                    N1 = S.EmptySet
                elif -1 <= cos(u)/v  and cos(u)/v <1:
                    N1 = Interval(-arccos(cos(u)/v),arccos(cos(u)/v))
                else:
                    N1 = Interval(-pi, pi)

                #calculation for N21
                zeta = arccos(1/sqrt(p**2 + 1))
                A = (p*cos(u) + sin(u))/sqrt(p**2 + 1)
                if A/v > 1 :
                    N21 = S.EmptySet
                elif 0 <= A/v and A/v <= 1:
                    eta1 = arcsin(A/v) - zeta
                    eta2 = pi -arcsin(A/v) - zeta
                    N21 = Interval(eta1,eta2)
                elif -1 < A/v and A/v < 0:
                    eta1 = arcsin(A/v) - zeta
                    eta2 = -pi -arcsin(A/v) - zeta
                    N21 = Interval(eta1,eta2)  #convert to list and add a bool for which case it has returned
                else:
                    eta1 = -pi
                    eta2 = pi
                    N21 = Interval(-pi,pi)

                zeta_prime = arccos(-1/sqrt(p**2 +1))
                A_prime = (p*cos(u) - sin(u))/sqrt(p**2 + 1)
                if A_prime/v > 1 :
                    N22 =  S.EmptySet
                elif 0 <= A_prime/v and A_prime/v <= 1:
                    eta1_prime = arcsin(A_prime/v) - zeta_prime
                    eta2_prime = pi -arcsin(A_prime/v) - zeta_prime
                    N22 = Interval(eta1_prime,eta2_prime)
                elif -1 < A_prime/v and A_prime/v < 0:
                    eta1_prime = arcsin(A_prime/v) - zeta_prime
                    eta2_prime = -pi -arcsin(A_prime/v) - zeta_prime
                    N22 = Union(Interval(-inf,eta1_prime),Interval(eta2_prime,inf))  #convert to list and add a bool for which case it has returned
                else:
                    eta1_prime = -pi
                    eta2_prime = pi
                    N22 = Interval(-pi,pi)


                if eta1_prime < -pi and eta2 < -pi:
                    eta2 += 2*pi
                    eta1_prime += 2*pi
                elif eta2 < -pi:
                    eta2 += 2*pi
                
                if eta1 < -pi and eta2_prime < -pi:
                    eta1 += 2*pi
                    eta2_prime += 2*pi
                elif eta2_prime < -pi:
                    eta2_prime += 2*pi


                if A/v >1 or A_prime/v >1:
                    N21_int_N22 = S.EmptySet
                elif -1 < A/v and A/v <= 1 and -1 < A_prime/v and A_prime/v <= 1:
                    if v > 1 and zeta >= 0.5*abs(arcsin(A/v)+arcsin(A_prime/v)):
                        N21_int_N22 = Interval(eta1,eta2_prime)
                    elif v < 1 and 0 <= zeta <= 0.5*abs(arcsin(A/v) + arcsin(A_prime/v)):
                        N21_int_N22 = Union(Interval(eta1,eta2_prime),Interval(eta1_prime,eta2))
                    else:
                        N21_int_N22 = S.EmptySet
                elif A/v <= -1 and -1 <= A_prime/v <= 1:
                    N21_int_N22 = N22
                elif -1 <= A/v <= 1 and A_prime/v <= -1:
                    N21_int_N22 = N21
                elif A/v <= -1 and A_prime/v <= -1:
                    N21_int_N22 = Interval(-pi,pi)
                    # takin intersection of all N1 , N21 and N22
                cone = Intersection(N1,N21_int_N22)
            if cone.contains(v_pref):
                while cone.contains(v_pref):
                    v_pref += 0.1
                return v_pref
            else:
                return v_pref
            # try:
            #     return cone.args[1].args[1] + theta0 + 0.1
            # except:
            #     return cone.args[1] + theta0 + 0.1
        except:
            return arctan2(y-self.y,x-self.x)
    
    def go_simon(self,x,y):
        law = PID_control("test",publisher_name="/tb3_0/cmd_vel",odom_name="/tb3_0/odom")
        param_points = Pose2D()
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            # v_pref = arctan2(y-self.y,x-self.x)
            v_des = self.collision_cone(x,y)
            print(v_des)
            # if not v_des == 10:
                # if v_des > pi/2:
                #     continue
            param_points.x = 0.1*np.cos(v_des) + self.x
            param_points.y = 0.1*np.sin(v_des) + self.y
            param_points.theta = 0
            law.Velocity_tracking_law(param_points)
            # else:
            #     param_points.x = 0.1*cos(v_pref) + self.x
            #     param_points.y = 0.1*sin(v_pref) + self.y
            #     law.Velocity_tracking_law(param_points)
            r.sleep()
        

    


if __name__ == '__main__':
    rospy.init_node('Dynamic_obstacle')
    # a = (collision_cone(0.8,10,pi/4,215*pi/180,3))
    # print(a)
    obstacle = dynamic_obstacle(0.3)
    robot1 = robot(0.3,"/tb3_0/cmd_vel","/tb3_1/odom")
    rospy.Subscriber("/tb3_0/odom",Odometry,robot1.locate)
    # rospy.Subscriber("/tb3_1/odom",Odometry,obstacle.locate)
    while not rospy.is_shutdown():
        robot1.go_simon(4,0)
        


    