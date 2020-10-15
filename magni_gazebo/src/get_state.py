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
tolerance = 0.2
def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        return gms
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def goal_processor(x,y,keep_distance):
    r = math.sqrt(x*x+y*y)
    if(r>keep_distance+tolerance):
        print("larger than r: {} {} --> {}".format(x,y,r))
        x_goal = x - (x/math.sqrt(x*x+y*y)*keep_distance)
        y_goal = y - (y/math.sqrt(x*x+y*y)*keep_distance)
        return x_goal,y_goal
    else:
        print("Small than r: {} {}".format(x,y))
        return 0,0

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/target_point', PoseStamped, queue_size=1)
    res = gms_client(model_name,relative_entity_name)
    msg = PoseStamped()
    msg.header.frame_id = "base_link"
    try:
        while True:
            g = res(model_name,relative_entity_name)
            msg.header.stamp = rospy.Time.now()
            x = g.pose.position.x
            y = g.pose.position.y
            x,y=goal_processor(x,y,keep_distance)
            msg.pose.position.x = x
            msg.pose.position.y = y
            print("{} {}\n".format(msg.pose.position.x,msg.pose.position.y))

            pub.publish(msg)
            rospy.sleep(0.05)
    except KeyboardInterrupt:
        print('interrupted!')