#!/usr/bin/env python
import rospy
import roslib 
from geometry_msgs.msg import PoseStamped
import rospy
from gazebo_msgs.srv import *
import math

model_name = 'magni1'
relative_entity_name = 'magni'
keep_distance = 1.0

def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        return gms
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/goal/position', PoseStamped, queue_size=1)
    res = gms_client(model_name,relative_entity_name)
    msg = PoseStamped()
    msg.header.frame_id = "base_link"
    try:
        while True:
            g = res(model_name,relative_entity_name)
            msg.header.stamp = rospy.Time.now()
            x = g.pose.position.x
            y = g.pose.position.y
            print ("return x ",x,y)
            # if (math.sqrt(x**2+y**2)-keep_distance>0):
            #     x_goal = x/math.sqrt(x**2+y**2)*keep_distance
            #     y_goal = y/math.sqrt(x**2+y**2)*keep_distance
                # msg.pose.position.x = x-x_goal
                # msg.pose.position.y = y-y_goal
            # else :
            #     msg.pose.position.x = 0
            #     msg.pose.position.y = 0
            msg.pose.position.x = x
            msg.pose.position.y = y
            pub.publish(msg)
            rospy.sleep(0.05)
            print ("return x position ",msg.pose.position.x,msg.pose.position.y)
    except KeyboardInterrupt:
        print('interrupted!')