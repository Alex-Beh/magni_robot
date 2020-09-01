#!/usr/bin/env python3

'''
(i) Hardcode the tf transform -> need to fix it 
    http://wiki.ros.org/tf
(ii) Follow target position technique
    -> Proportional to the difference: https://github.com/chapulina/dolly/blob/master/dolly_follow/src/dolly_follow.cpp
                                       https://github.com/UbiquityRobotics/demos/blob/52136f04c0f39fe6a1001a17e208e5ce5b4cda61/fiducial_follow/nodes/follow.py#L211
    -> PI Controller: https://github.com/Adlink-ROS/adlink_neuronbot/blob/v2.0/script/neuronbot_following_fused.py

Improvement: should change Vector3Stamped to other message type that including the frame_id? easier for tf transform(?) 

Question: Don't know why the KI controller don't work in angular speed? And why use acos? why not atan2 or atan?
'''

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
import math

class MagniMovementPublisherNode():
    def __init__(self):
        rospy.init_node("magni_movement_publisher_node")
        self.magni_movement_subscriber = rospy.Subscriber("/aruco_single/position", Vector3Stamped, self.magni_position_callback)

        self.magni_movement_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
       
        self.previous = 0
        self.sum_error_r = 0
        self.sum_error_th = 0
        
        # motion KI parameters
        self.Ki_linear_window  = 0.95
        self.Ki_angular_window = 0.95
        self.linear_bound  = 0.3
        self.angular_bound = 0.4
        self.following_distance = rospy.get_param('~following_distance', 0.6)
        self.dead_zone_radius   = rospy.get_param('~dead_zone_radius',  0.2) # +/- m
        self.dead_zone_theta    = rospy.get_param('~dead_zone_theta', 0.1) # +/- rad
        self.Kp_linear  = rospy.get_param('~Kp_linear', 0.8)
        self.Kp_angular = rospy.get_param('~Kp_angular', 1.5)
        self.Ki_linear  = rospy.get_param('~Ki_linear', 0.0)
        self.Ki_angular = rospy.get_param('~Ki_angular', 0.0)
        self.Ki_factor_linear  = rospy.get_param('~Ki_factor_linear',  0.95)
        self.Ki_factor_angular = rospy.get_param('~Ki_factor_angular', 0.95)

    def magni_position_callback(self, data):
        # print(data.vector.x, data.vector.y, data.vector.z)

        r,th = self.xyToPolar(data)

        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.previous
        # Calculate PI control
        error_r  = r - self.following_distance
        error_th = th - 0.0
        
        if abs(error_r) >= self.dead_zone_radius:
            self.sum_error_r  = self.Ki_factor_linear*self.Ki_linear_window*self.sum_error_r + error_r*dt
            follow_speed_x = self.Kp_linear * error_r + self.Ki_linear  * (self.sum_error_r)
            follow_speed_x = self.constrain(follow_speed_x, self.linear_bound, -self.linear_bound)
        else:
            follow_speed_x = 0.0
            # rospy.loginfo('Linear Dead Zone!')

        if abs(error_th) >= self.dead_zone_theta:
            follow_speed_z = error_th
            # self.sum_error_th = self.Ki_factor_angular*self.Ki_angular_window*self.sum_error_th + error_th*dt
            # follow_speed_z = self.Kp_angular * (th)   + self.Ki_angular * (self.sum_error_th)
            # print("Before: ",follow_speed_z," th: ",th)
            follow_speed_z = self.constrain(follow_speed_z, self.angular_bound, -self.angular_bound)
            # print("After: ",follow_speed_z)
        else:
            follow_speed_z = 0.0
            # rospy.loginfo('Angular Dead Zone!')
        
        # Pub vel cmd
        motion_cmd = Twist()
        motion_cmd.angular.z = follow_speed_z
        motion_cmd.linear.x  = follow_speed_x
        self.magni_movement_publisher.publish(motion_cmd)
        self.previous = current_time

    def constrain(self, num, limit_upper, limit_lower):
        if num >= limit_upper:
            num = limit_upper
            return num
        elif num <= limit_lower:
            num = limit_lower
            return num
        else:
            return num

    def xyToPolar(self,data_xyz):
        target_x = data_xyz.vector.x
        target_z = data_xyz.vector.z
        r = math.sqrt(math.pow(target_x, 2) + math.pow(target_z, 2) )
        # th = math.atan2(target_z,target_x)
        # if(target_x>0):
        #     th = -th
        th = self.get_angle(target_x,target_z)
        return r,th

    #get angle between the magni and the assigned position
    def get_angle(self, x, z):
        # using the fomula a.b = |a||b|cos(theta)
        # taking 2 vectors where 
        # a is (0, 1), the front where magni is facing
        # b is (x, z), the vector for the input position
        
        a_dot_b = z
        modulus_a = 1
        modulus_b = math.sqrt(x**2 + z**2)
        angle = math.acos(a_dot_b/(modulus_a * modulus_b))    #in radian
        # angle_degree = math.degrees(angle)                  #in degree

        angle_degree = angle

        if(x>0):
            angle_degree = -angle_degree
        return angle_degree

if __name__ == "__main__":

    try:
        MagniMovementPublisherNode()
    except rospy.ROSInterruptException:
        pass
   
    rospy.spin()