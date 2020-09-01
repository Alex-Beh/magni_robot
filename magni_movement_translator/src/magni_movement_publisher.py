#!/usr/bin/env python2

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
from geometry_msgs.msg import Twist,Vector3Stamped,PointStamped
import math
import tf
import numpy as np
from target_goal.msg import person_goal

class MagniMovementPublisherNode():
    def __init__(self):
        rospy.init_node("magni_movement_publisher_node")
        self.magni_movement_subscriber = rospy.Subscriber("/aruco_single/position", person_goal, self.magni_position_callback)

        self.magni_movement_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
       
        self.previous_time = 0
        self.sum_error_r = 0
        self.sum_error_th = 0
        
        # motion KI parameters
        self.Ki_linear_window  = 0.95
        self.Ki_angular_window = 0.95
        # Parameters in simulation: self.linear_bound = 0.3 self.angular_bound = 0.4
        self.linear_bound  = 0.3
        self.angular_bound = 0.4
        self.following_distance = rospy.get_param('~following_distance', 0.8)
        self.dead_zone_radius   = rospy.get_param('~dead_zone_radius',  0.2) # +/- m
        self.dead_zone_theta    = rospy.get_param('~dead_zone_theta', 0.1) # +/- rad
        self.Kp_linear  = rospy.get_param('~Kp_linear', 0.8)
        self.Kp_angular = rospy.get_param('~Kp_angular', 1.5)
        self.Ki_linear  = rospy.get_param('~Ki_linear', 0.0)
        self.Ki_angular = rospy.get_param('~Ki_angular', 0.0)
        self.Ki_factor_linear  = rospy.get_param('~Ki_factor_linear',  0.95)
        self.Ki_factor_angular = rospy.get_param('~Ki_factor_angular', 0.95)

        self.listener = tf.TransformListener()

    def magni_position_callback(self, data):
        # print(data.vector.x, data.vector.y, data.vector.z)
        pointstamped_msg = PointStamped()
        pointstamped_msg.header.frame_id = "odom"
        pointstamped_msg.header.stamp =rospy.Time(0)
        pointstamped_msg.point.x = data.goal.vector.x
        pointstamped_msg.point.y = data.goal.vector.y
        pointstamped_msg.point.z = 0

        transform_state = False

        try:
            # transform the tracked_person from odom from to base_link
            self.listener.waitForTransform("/base_link", "/odom", rospy.Time(0),rospy.Duration(4.0))
            transform_state = True
            transformed_point=self.listener.transformPoint("base_link",pointstamped_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Transformation got error!")
            # print(e)
            self.magni_movement_publisher.publish(Twist())
            return

        r,th = self.xyToPolar(transformed_point)

        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.previous_time

        # Calculate PI control
        error_r  = r - self.following_distance
        error_th = th - 0.0
        
        if abs(error_r) >= self.dead_zone_radius:
            self.sum_error_r  = self.Ki_factor_linear*self.Ki_linear_window*self.sum_error_r + error_r*dt
            follow_speed_x = self.Kp_linear * error_r + self.Ki_linear  * (self.sum_error_r)
            print("linear")
            follow_speed_x = self.constrain(follow_speed_x, self.linear_bound, -self.linear_bound)
        else:
            follow_speed_x = 0.0
            # rospy.loginfo('Linear Dead Zone!')

        if abs(error_th) >= self.dead_zone_theta:
            self.sum_error_th = self.Ki_factor_angular*self.Ki_angular_window*self.sum_error_th + error_th*dt
            follow_speed_z = self.Kp_angular * (th)   + self.Ki_angular * (self.sum_error_th)
            print("angular")
            follow_speed_z = self.constrain(follow_speed_z, self.angular_bound, -self.angular_bound)
        else:
            follow_speed_z = 0.0
            # rospy.loginfo('Angular Dead Zone!')
        
        # Pub vel cmd
        motion_cmd = Twist()
        if transform_state:
            #Parameters in simulation: ratio_angular_speed = 1.5 && ratio_linear_speed = 1 && vector_tf = 1
            ratio_angular_speed = 1
            ratio_linear_speed = 1

            vector_tf = -1
            add_person_twist = 0

            motion_cmd.angular.z = follow_speed_z*ratio_angular_speed
            if(data.robot_heading.data ==1):
                motion_cmd.linear.x  = (follow_speed_x*ratio_linear_speed + add_person_twist*0.8*data.goal.vector.z)*vector_tf
            # else:
            #     print("move_back")
            #     motion_cmd.linear.x  = -0.3

        self.magni_movement_publisher.publish(motion_cmd)
        self.previous_time = current_time

    def constrain(self, num, limit_upper, limit_lower):
        if num >= limit_upper:
            num = limit_upper
            print("upperbound")
            return num
        elif num <= limit_lower:
            print("lowerbound")
            num = limit_lower
            return num
        else:
            return num

    def xyToPolar(self,data_xyz):
        # change here for different transformation
        longitudinal_distance = data_xyz.point.x
        lateral_distance = data_xyz.point.y

        # get angle between the robot and the target position
        resultant_distance = math.sqrt(longitudinal_distance**2 + lateral_distance**2)
        angle = math.acos(longitudinal_distance/(resultant_distance))    #in radian

        if(lateral_distance<0):
            angle = -angle

        return resultant_distance,angle

if __name__ == "__main__":

    try:
        MagniMovementPublisherNode()
    except rospy.ROSInterruptException:
        pass
   
    rospy.spin()