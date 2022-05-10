import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

I_POSE_H1 = [7.90, 0.00, 1.00]
I_POSE_H2 = [8.89, 5.41, 1.00]

class InitPose:
    def __init__(self):
             
        husky1_ini = rospy.Publisher("/husky1/initialpose", PoseWithCovarianceStamped, queue_size=1)
        husky2_ini = rospy.Publisher("/husky2/initialpose", PoseWithCovarianceStamped, queue_size=1)
        rospy.init_node("talker_initpose")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            init_pose = PoseWithCovarianceStamped()
            init_pose.header.stamp = rospy.get_rostime()
            init_pose.header.frame_id = "map"
            init_pose.pose.pose.position.x = I_POSE_H1[0]
            init_pose.pose.pose.position.y = I_POSE_H1[1]
            init_pose.pose.pose.position.z = 0
            init_pose.pose.pose.orientation.x = 0
            init_pose.pose.pose.orientation.y = 0
            init_pose.pose.pose.orientation.z = 0
            init_pose.pose.pose.orientation.w = I_POSE_H1[2]
            init_pose.pose.covariance = [0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
            husky1_ini.publish(init_pose)
            init_pose.header.stamp = rospy.get_rostime()
            init_pose.header.frame_id = "map"
            init_pose.pose.pose.position.x = I_POSE_H2[0]
            init_pose.pose.pose.position.y = I_POSE_H2[1]
            init_pose.pose.pose.position.z = 0
            init_pose.pose.pose.orientation.x = 0
            init_pose.pose.pose.orientation.y = 0
            init_pose.pose.pose.orientation.z = 0
            init_pose.pose.pose.orientation.w = I_POSE_H2[2]
            init_pose.pose.covariance = [0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
            husky2_ini.publish(init_pose)
            rate.sleep()

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
    try:
        InitPose()
    except rospy.ROSInterruptException:
        print("ROS SYSTEM INTERRUPTED")
