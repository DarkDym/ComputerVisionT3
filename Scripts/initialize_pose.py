import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

I_POSE_H1 = [0.00, 0.00, 0.00]
I_POSE_H2 = [14.00, 7.00, 0.00]

class InitPose:
    def __init__(self):
             
        husky1_ini = rospy.Publisher("/husky1/initialpose", PoseWithCovarianceStamped, queue_size=1)
        husky2_ini = rospy.Publisher("/husky2/initialpose", PoseWithCovarianceStamped, queue_size=1)
        rospy.init_node("talker_initpose", anonymous=True)
        rate = rospy.Rate(10)
        h1_ipose = self.initialize_pose1(I_POSE_H1)
        h2_ipose = self.initialize_pose2(I_POSE_H2)
        husky1_ini.publish(h1_ipose)
        husky2_ini.publish(h2_ipose)
    def initialize_pose1(self, i_pose):
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.stamp = rospy.get_rostime()
        init_pose.header.frame_id = "map"
        init_pose.pose.pose.position.x = i_pose[0]
        init_pose.pose.pose.position.y = i_pose[1]
        init_pose.pose.pose.position.z = 0
        init_pose.pose.pose.orientation.x = 0
        init_pose.pose.pose.orientation.y = 0
        init_pose.pose.pose.orientation.z = 0
        init_pose.pose.pose.orientation.w = i_pose[2]
        init_pose.pose.covariance = [0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        return init_pose

    def initialize_pose2(self, i_pose):
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.stamp = rospy.get_rostime()
        init_pose.header.frame_id = "map"
        init_pose.pose.pose.position.x = i_pose[0]
        init_pose.pose.pose.position.y = i_pose[1]
        init_pose.pose.pose.position.z = 0
        init_pose.pose.pose.orientation.x = 0
        init_pose.pose.pose.orientation.y = 0
        init_pose.pose.pose.orientation.z = 0
        init_pose.pose.pose.orientation.w = i_pose[2]
        init_pose.pose.covariance = [0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        return init_pose

if __name__ == '__main__':
    InitPose()