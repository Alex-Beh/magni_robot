#!/usr/bin/env python
import rospy
import roslib 
from geometry_msgs.msg import PoseStamped, TransformStamped
import rospy
from gazebo_msgs.srv import *
import math
import rospy

# Because of transformations
import tf_conversions
import tf2_ros


model_name = 'magni1'
relative_entity_name = 'magni'
keep_distance = -0.3
tolerance = 1.2 
def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        return gms
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def goal_processor(x,y,keep_distance,tolerance):
    r = math.sqrt(x*x+y*y)
    if r <= 1.0 :
        keep_distance = 0.5
        print("Approaching vehicle! Slow down! {}".format(keep_distance))
    elif abs(y) > 0.3 :
        keep_distance = 0.2
        tolerance = 0.7 
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
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    
    t.header.frame_id = "base_link"
    t.child_frame_id = "human"
    pub1 = rospy.Publisher('/leg_pose', PoseStamped, queue_size=1)
    pub = rospy.Publisher('/global_goal', PoseStamped, queue_size=1)
    res = gms_client(model_name,relative_entity_name)
    msg = PoseStamped()
    msg.header.frame_id = "base_link"
    try:
        while True:
            g = res(model_name,relative_entity_name)
            msg.header.stamp = rospy.Time.now()
            x = g.pose.position.x
            y = g.pose.position.y
            msg.pose.position.x = x
            msg.pose.position.y = y
            pub1.publish(msg)
            # t.transform.translation.x = x
            # t.transform.translation.y = y
            # t.transform.rotation.w = 1.0
            x,y=goal_processor(x,y,keep_distance,tolerance)
            msg.pose.position.x = x
            msg.pose.position.y = y
            # t.header.stamp = rospy.Time.now()
            # br.sendTransform(t)
            print("{} {}\n".format(msg.pose.position.x,msg.pose.position.y))

            pub.publish(msg)
            rospy.sleep(0.05)
    except KeyboardInterrupt:
        print('interrupted!')