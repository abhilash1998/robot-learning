

from logging import shutdown
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
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
import os
import orjson as json

class Data:
    def __init__(self):
        self.bridge=CvBridge()
        self.pos_subscriber=rospy.Subscriber("/odom", Odometry, self.pose_callback)
        self.camera_subscriber=rospy.Subscriber("/camera/image",Image,self.img_callback)
        # self.depth_subscriber=rospy.Subscriber("/camera/image/compressedDepth",CompressedImage,self.compressed_depth_img)
        self.laser_2d_subscriber=rospy.Subscriber("/scan",LaserScan,self.lidar_2d_callback)
        self.output_vel=rospy.Subscriber("/cmd_vel",Twist,self.cmd_vel_callback)

        self.map_subscriber=rospy.Subscriber("/map",OccupancyGrid,self.map_callback)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo('Waiting for the action client to start')
        self.client.wait_for_server()
        rospy.loginfo("Action client started")
        self.imu_subscriber=rospy.Subscriber("imu",Imu,self.imu_callback)
        self.data={'Camera':[],'LaserScan':[],'IMU':[],'cmd_vel':[],'position':[],'orientation':[],'linear_velocity':[],'angular_velocity':[]}
        #self.client.wait_for_server(wait)
        # self.movebase_action_client()
        self.i=0
        self.data_dir="../data/"
        self.status=3
        #self.map=OccupancyGrid()
        
        
    def map_callback(self,data):
       self.map=np.asarray(data.data  )
       self.map=self.map.reshape(int(data.info.width),int(data.info.height))
    #    print((self.map.shape))
    #    self.map_max=self.map.shape
       self.map_resolution=data.info.resolution
       self.map_origin_x=data.info.origin.position.x
       self.map_origin_y=data.info.origin.position.y
       
       

       
       
       
       
       
    #    print(data.info.width,data.info.height)
    
    
    def cmd_vel_callback(self,msg):
        self.cmd_vel_linear=np.zeros(2)
        self.cmd_vel_linear[0]=msg.linear.x
        # cmd_vel_linear[1]=msg.linear.y
        self.cmd_vel_linear[1]=msg.angular.z
        # self.data['cmd_vel']=self.cmd_vel_linear
    
        

    def imu_callback(self,msg):
        self.imu_data=np.zeros(6)
        self.imu_data[0]= msg.angular_velocity.x
        self.imu_data[1]= msg.angular_velocity.y
        self.imu_data[2]= msg.angular_velocity.z
        
        self.imu_data[3]= msg.linear_acceleration.x
        self.imu_data[4]= msg.linear_acceleration.y
        self.imu_data[5]= msg.linear_acceleration.z
        # print('yes')
        # self.data["IMU"]=self.imu_data
        
    # def compressed_depth_img(self,msg):
        # np_arr = np.fromstring(msg.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        
        # print(depth)
        # cv2.imshow('depth',image_np)
        # cv2.waitKey(1)  
    def lidar_2d_callback(self,msg):
        # print(msg)
        self.laser_data=msg.ranges
        
        # print(np.array(self.laser_data).shape)
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
        # rospy.logwarn("Subscribed sucessfullly")
        position=np.zeros(2)
        position[0]=msg.pose.pose.position.x
        position[1]=msg.pose.pose.position.y
        self.position=position
        quaternion=np.zeros(4)
        quaternion[0]=msg.pose.pose.orientation.x
        quaternion[1]=msg.pose.pose.orientation.y
        quaternion[2]=msg.pose.pose.orientation.z
        quaternion[3]=msg.pose.pose.orientation.w
        self.orientation=euler_from_quaternion(quaternion)
        
        # print(self.orientation)
        # self.orientation=quaternion
        
        linear_velocity=np.zeros(2)
        linear_velocity[0]=msg.twist.twist.linear.x
        # linear_velocity[1]=msg.twist.twist.linear.y
        self.linear_velocity=linear_velocity
        
        
        
        
        
        angular_speed=msg.twist.twist.angular.z
        # print(type(angular_speed))
        self.angular_speed=angular_speed


    def movebase_action_client(self):
        if self.status==3:
            self.goal=MoveBaseGoal()
            self.goal.target_pose.header.frame_id="map"
            self.goal.target_pose.header.stamp=rospy.Time.now()
            # print("xxytxyxgy")
            self.goal.target_pose.pose=self.goal_generator()
            self.clearing_data()  
        # print("goal",self.goal.target_pose.pose)
        
            self.client.send_goal(self.goal,self.completed_action_callback,self.active_action_callback,self.feedback_action_server_callback)
            self.status=0
    def goal_generator(self):
        self.goal_pose=Pose()


        x=np.random.randint(0,self.map.shape[0])
        y=np.random.randint(0,self.map.shape[1])
        
        if self.map[y][x]>50 or self.map[x][y] ==-1:
            return self.goal_generator()
        else:
              
            self.goal_pose.position.x=self.map_origin_x+y*self.map_resolution
            self.goal_pose.position.y=self.map_origin_y+x*self.map_resolution
            print(self.map[x][y],self.map[int((self.goal_pose.position.x -self.map_origin_x)/self.map_resolution)][int((self.goal_pose.position.z -self.map_origin_y)/self.map_resolution)])
            # -10.8,1.2
            # print(self.map[int((-10.8 -self.map_origin_x)/self.map_resolution)][int((1.2 -self.map_origin_y)/self.map_resolution)])
            
            # self.goal_pose.position.x =-4.560
            # self.goal_pose.position.y =0.204
            self.goal_pose.position.z =0.0
            self.goal_pose.orientation.w=1.0
            return self.goal_pose
    def feedback_action_server_callback(self,feedback):
        # rospy.loginfo(self.i) 
        if self.cmd_vel_linear[1]!=0.0:
            self.data['position'].append(self.position.tolist())
            self.data['LaserScan'].append(self.laser_data)
            self.data['orientation'].append(self.orientation)
            self.data['Camera'].append(self.image_data.tolist())
            self.data['cmd_vel'].append(self.cmd_vel_linear.tolist())
            self.data['IMU'].append(self.imu_data.tolist())
            self.data['linear_velocity'].append(self.linear_velocity.tolist())
            self.data['angular_velocity'].append(self.angular_speed)
        
        # self.i=self.i+1
        # rospy.signal_shutdown()
    def clearing_data(self):
        self.data={'Camera':[],'LaserScan':[],'IMU':[],'cmd_vel':[],'position':[],'orientation':[],'linear_velocity':[],'angular_velocity':[]}  
    
    def saving_trajectory_json(self):
        file_name = self.data_dir+"trajectory_"+str(self.i)+'.json'
        while os.path.exists(file_name):
            self.i += 1
            file_name = self.data_dir+"trajectory_"+str(self.i)+'.json'
        # print(type(self.data))
        if len(self.data['cmd_vel'])>0:
            print('saving trajectory')
            with open(file_name,'wb') as fout:
                fout.write(json.dumps(self.data))
            
    def completed_action_callback(self,status,result):
        rospy.loginfo('status 3')
        self.saving_trajectory_json()
        self.status=3
        if status ==2:
            rospy.loginfo("Goal pose is interrupted and changed") 
        if status ==3:
            rospy.loginfo("Goal pose is successfully reached")
            
    def active_action_callback(self):
        rospy.loginfo("Goal is being processed by the acttion server")
    
if __name__ == "__main__":
    rospy.init_node("Data_Collection",disable_signals=True)
    rate=rospy.Rate(20)
    data=Data()
    while not rospy.is_shutdown():
        data.movebase_action_client()
        rate.sleep()
    rospy.spin()
