import rospy
from apriltag_ros.msg import AprilTagDetectionArray

class TagRead:
    def __init__(self,robot_namespace):
        print("#####################################INITIALIZE SCRIPT#####################################")
        rospy.init_node("istener_tag", anonymous=True)
        rospy.Subscriber(str(robot_namespace)+"/tag_detections", AprilTagDetectionArray, self.tag_callback)
        rospy.spin()
        
    
    def tag_callback(self,tag_msg):
        print("##############ACHEI A TAG##############")
        print(tag_msg.detections[0].id[0])

if __name__ == '__main__':
    tg_read = TagRead("/husky1")