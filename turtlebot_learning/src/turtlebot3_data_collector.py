
import rospy
from  sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Data:
    def __init__(self):
        self.pos_subscriber=rospy.Subscriber("/odom", Odometry, self.pose_callback)
        self.goal_publish=rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.goal=PoseStamped()
        self.goal.header.stamp=rospy.Time.now()    
        self.goal.header.frame_id="map"
        self.goal.pose.position.x =-4.560
        self.goal.pose.position.y =0.204
        self.goal.pose.position.z =0.0
        self.goal.pose.orientation.w=1.0
        
    def pose_callback(self,msg):
        # print('msg',msg)
        rospy.logwarn("Subscribed sucessfullly")
        self.goal_publish.publish(self.goal) 
        print(self.goal)
        rospy.sleep(2)
        
    
if __name__ == "__main__":
    rospy.init_node("Data_Collection")
    data=Data()
    rospy.spin()
