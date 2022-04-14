

import rospy
from  sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Pose
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge,CvBridgeError
import cv2
# from costmap_2d import costmap_2d
# costmap_2d.

class Data:
    def __init__(self):
        self.bridge=CvBridge()
        self.pos_subscriber=rospy.Subscriber("/odom", Odometry, self.pose_callback)
        self.camera_subscriebr=rospy.Subscriber("/camera/image",Image,self.img_callback)
        # self.goal_publish=rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        # self.goal=PoseStamped()
        # self.goal.header.stamp=rospy.Time.now()    
        # self.goal.header.frame_id="map"
        # self.goal.pose.position.x =-4.560
        # self.goal.pose.position.y =0.204
        # self.goal.pose.position.z =0.0
        # self.goal.pose.orientation.w=1.0
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # print(self.client)
        # rospy.wait_for_service(self.client)
        rospy.loginfo('Waiting for the action client to start')
        self.client.wait_for_server()
        rospy.loginfo("Action client started")
        #self.client.wait_for_server(wait)
        self.movebase_action_client()
        
        
        
    def img_callback(self,msg):
        self.bridge=CvBridge()
        try :
            self.image_data=self.bridge.imgmsg_to_cv2(msg,desired_encoding ="bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("image",self.image_data)
        cv2.waitKey(1)
        # rospy.signal_shutdown(1==1)

    def pose_callback(self,msg):
        # print('msg',msg)
        rospy.logwarn("Subscribed sucessfullly")
    
        # print(self.goal)
        rospy.sleep(2)
    def movebase_action_client(self):
        self.goal=MoveBaseGoal()
        self.goal.target_pose.header.frame_id="map"
        self.goal.target_pose.header.stamp=rospy.Time.now()
        self.goal.target_pose.pose=self.goal_generator()
        self.client.send_goal(self.goal,self.completed_action_callback,self.active_action_callback,self.feedback_action_server_callback)
    def goal_generator(self):
        self.goal_pose=Pose()
        self.goal_pose.position.x =-5.560
        self.goal_pose.position.y =0.204
        self.goal_pose.position.z =0.0
        self.goal_pose.orientation.w=1.0
        return self.goal_pose
    def feedback_action_server_callback(self,feedback):
        rospy.loginfo("Goal pose") 
        # print(feedback)
    def completed_action_callback(self,status,result):
        
        if status ==2:
            rospy.loginfo("Goal pose is interrupted and changed") 
        if status ==3:
            rospy.loginfo("Goal pose is successfully reached")
    def active_action_callback(self):
        rospy.loginfo("Goal is being processed by the acttion server")
    
if __name__ == "__main__":
    rospy.init_node("Data_Collection",disable_signals=True)
    data=Data()
    rospy.spin()
