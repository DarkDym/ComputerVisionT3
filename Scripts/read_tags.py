import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseGoal,MoveBaseResult,MoveBaseAction
from cv_bridge import CvBridge
import cv2
from scipy.spatial import distance
import actionlib
from std_msgs.msg import Int8
import imutils
import numpy as np

PATROL = [[13.00,-0.58],[16.41,-0.48],[15.64,4.72],[8.51,5.53],[5.10,0.00]]
ROBOT_NAMESPACE = "/husky1"
CONT_PATROL = 0

class TagRead:
    def __init__(self,robot_namespace,client):
        print("#####################################INITIALIZE SCRIPT#####################################")
        cont = 0
        rospy.init_node("listener_tag", anonymous=True)
        rospy.Subscriber(str(robot_namespace)+"/tag_detections_image", Image, self.image_tag_callback)
        rospy.Subscriber(str(robot_namespace)+"/tag_detections", AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber("/patrol_cont", Int8, self.patrol_cont_callback)
        pc_pub = rospy.Publisher("/patrol_cont", Int8, queue_size=10)
        self.client = client
        self.alredy_detect = False
        self.cv_image = np.zeros((480,640,3), dtype=np.uint8)
        self.first_frame = None
        self.stopped = False
        while not rospy.is_shutdown():
            if not self.stopped:
                result = self.goal_sending(ROBOT_NAMESPACE,PATROL[cont])
                if result:
                    rospy.loginfo("GOAL EXECUTION DONE")
                    print("RESULT: " +str(result))
                    print("CONT: "+str(cont))
                    if (cont >= len(PATROL)-1):
                        cont = 0
                        pc_pub.publish(cont)
                    else:
                        cont += 1
                        pc_pub.publish(cont)
        rospy.spin()
        
    
    def image_tag_callback(self,image_tag_msg):
        self.cv_image = bridge.imgmsg_to_cv2(image_tag_msg, desired_encoding='bgr8')
        cv2.imshow("TAG_DETECTION HUSKY1", self.cv_image)
        cv2.waitKey(1)
    
    def patrol_cont_callback(self,patrol_cont_msg):
        CONT_PATROL = patrol_cont_msg

    def tag_callback(self,tag_msg):
        
        if (len(tag_msg.detections) > 0):
            print("TAG ENCONTRADA")
            self.goal_cancel()
            self.get_frameRef()
            self.tracking()
        else:
            self.alredy_detect = False
            self.first_frame = None
            self.stopped = False
    #TESTAR MODIFICAR A FORMA COMO O MOVIMENTO DO HUSKY1 É REALIZADA
    def goal_cancel(self):
        self.client.cancel_all_goals()
        self.stopped = True

    def get_frameRef(self):
        self.frame_ref = self.cv_image

    #TESTAR MELHORAR A FORMA COMO O PRIMEIRO FRAME É OBTIDO, PARA DIMINUIR O RUÍDO NA IMAGEM
    def tracking(self):
        frame_ref_gray = cv2.cvtColor(self.frame_ref, cv2.COLOR_BGR2GRAY)
        print_frame = self.frame_ref
        # frame_ref_gray = cv2.GaussianBlur(frame_ref_gray,(21,21),0)

        if self.first_frame is None:
            self.first_frame = frame_ref_gray
        frame_delta = cv2.absdiff(self.first_frame,frame_ref_gray)
        #TESTAR DIMINUIR UM POUCO O VALOR DO THRESHOLD 
        threshold = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
        #TESTAR AMANHÃ, COLOCAR O KERNEL NO DILATE E TESTAR VER SE MELHORA O RESULTADO 
        kernel = np.ones((5,5))
        threshold = cv2.dilate(threshold, None, iterations=2)
        contours = cv2.findContours(threshold.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        for c in contours:
            if cv2.contourArea(c) < 500:
                continue
            (x,y,w,h) = cv2.boundingRect(c)
            cv2.rectangle(print_frame, (x,y), (x+w, y+h), (0,255,0),2)
        # cv2.imshow("DELTA", frame_delta)
        # cv2.waitKey(1)
        # cv2.imshow("FIRST_FRAME", self.first_frame)
        # cv2.waitKey(1)
        # cv2.imshow("REF_FRAME", frame_ref_gray)
        # cv2.waitKey(1)
        cv2.imshow("PRINT_FRAME", print_frame)
        cv2.waitKey(1)
                
    def goal_sending(self,robot_namespace,goal_point):
        print("#######SENDING GOAL")
        
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_point[0]
        goal.target_pose.pose.position.y = goal_point[1]
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()
    


if __name__ == '__main__':
    bridge = CvBridge()
    
    try:
        tg_read = TagRead(ROBOT_NAMESPACE,actionlib.SimpleActionClient(str(ROBOT_NAMESPACE)+"/move_base",MoveBaseAction))
    except rospy.ROSInterruptException:
        print("ROS SYSTEM INTERRUPTED")