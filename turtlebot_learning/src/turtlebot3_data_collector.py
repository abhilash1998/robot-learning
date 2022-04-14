

import rospy
from  sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Pose,Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Image,CompressedImage,Imu
import numpy as np
from cv_bridge import CvBridge,CvBridgeError
import cv2
from sensor_msgs.msg import LaserScan
# from costmap_2d import costmap_2d
# costmap_2d.

class Data:
    def __init__(self):
        self.bridge=CvBridge()
        self.pos_subscriber=rospy.Subscriber("/odom", Odometry, self.pose_callback)
        self.camera_subscriebr=rospy.Subscriber("/camera/image",Image,self.img_callback)
        self.depth_subscriber=rospy.Subscriber("/camera/image/compressedDepth",CompressedImage,self.compressed_depth_img)
        self.laser_2d_subscriber=rospy.Subscriber("/scan",LaserScan,self.lidar_2d_callback)
        self.output_vel=rospy.Subscriber("/cmd_vel",Twist,self.self.cmd_vel_callback)
        
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo('Waiting for the action client to start')
        self.client.wait_for_server()
        rospy.loginfo("Action client started")
        #self.client.wait_for_server(wait)
        self.movebase_action_client()
    def cmd_vel_callback(self,msg):
        cmd_vel_linear=np.zeros(2)
        cmd_vel_linear[0]=msg.linear.x
        cmd_vel_linear[1]=msg.linear.y
        
        cmd_vel_angular=msg.angular.z
        
    def imu_callback(msg):
        imu_data= msg  
    def compressed_depth_img(self,msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        # print(depth)
        cv2.imshow('depth',image_np)
        cv2.waitKey(1)  
    def lidar_2d_callback(self,msg):
        # print(msg)
        self.laser_data=msg.ranges
        print(np.array(self.laser_data).shape)
    def img_callback(self,msg):
        self.bridge=CvBridge()
        try :
            self.image_data=self.bridge.imgmsg_to_cv2(msg,desired_encoding ="bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("image",self.image_data)
        cv2.waitKey(1)
       
    def pose_callback(self,msg):
        # print('msg',msg)
        rospy.logwarn("Subscribed sucessfullly")
        position=np.zeros(2)
        position[0]=msg.pose.pose.position.x
        position[1]=msg.pose.pose.position.y
        
        quaternion=np.zeros(4)
        quaternion[0]=msg.pose.pose.orientation.x
        quaternion[1]=msg.pose.pose.orientation.y
        quaternion[2]=msg.pose.pose.orientation.z
        quaternion[3]=msg.pose.pose.orientation.w
        
        linear_velocity=np.zeros(2)
        linear_velocity[0]=msg.twist.twist.linear.x
        linear_velocity[1]=msg.twist.twist.linear.y
        # linear_velocity[2]=msg.twist.twist.linear.z
        # angular_speed=np.zeros(2)

        angular_speed=msg.twist.twist.angular.z
        
        # print(angular_speed)
        
        
    

    def movebase_action_client(self):
        self.goal=MoveBaseGoal()
        self.goal.target_pose.header.frame_id="map"
        self.goal.target_pose.header.stamp=rospy.Time.now()
        self.goal.target_pose.pose=self.goal_generator()
        self.client.send_goal(self.goal,self.completed_action_callback,self.active_action_callback,self.feedback_action_server_callback)
    def goal_generator(self):
        self.goal_pose=Pose()
        self.goal_pose.position.x =-4.560
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
