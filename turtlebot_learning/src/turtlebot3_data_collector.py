
import rospy
from  sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msg.msg import PoseStamped

class Data:
    def __init__(self):
        self.pos_subscriber=rospy.Subscriber("/odom", Odometry, self.pose_callback)
        self.goal_publish=rospy.Publisher("/move_base_simple/goal", type, queue_size=10)
        self.goal=PoseStamped()
        self.goal.header.seq.stamp=rospy.Time.now()    
        self.pose.position.x  
    def pose_callback(self,msg):
        print('msg',msg)
        rospy.logwarn("Subscribed sucessfullly")
    
if __name__ == "__main__":
    rospy.init_node("Data_Collection")
    data=Data()
    rospy.spin()
