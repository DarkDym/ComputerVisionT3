from time import sleep
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseGoal,MoveBaseResult,MoveBaseAction
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
from scipy.spatial import distance
import actionlib
from std_msgs.msg import Int8
import imutils
import numpy as np
import message_filters

PATROL = [[13.00,-0.58],[16.41,-0.48],[15.64,4.72],[8.51,5.53],[5.10,0.00]]
ROBOT_NAMESPACE = "/husky1"
CONT_PATROL = 0
PATROL_RESUME = 0

class TagRead:
    def __init__(self,robot_namespace,client):
        print("#####################################INITIALIZE SCRIPT#####################################")
        cont = CONT_PATROL
        rospy.init_node("listener_tag", anonymous=True)        
        rospy.Subscriber(str(robot_namespace)+"/tag_detections_image", Image, self.image_tag_callback)
        rospy.Subscriber(str(robot_namespace)+"/tag_detections", AprilTagDetectionArray, self.tag_callback)
        #rospy.Subscriber(str(robot_namespace)+"/cmd_vel", Twist, self.cmdvel_callback)
        rospy.Subscriber(str(robot_namespace)+"/odometry/filtered", Odometry, self.odometry_callback)
        rospy.Subscriber("/patrol_cont", Int8, self.patrol_cont_callback)
        pc_pub = rospy.Publisher("/patrol_cont", Int8, queue_size=10)
        self.client = client
        self.alredy_detect = False
        self.cv_image = np.zeros((480,640,3), dtype=np.uint8)
        self.current_frame = None
        self.first_frame = None
        self.stopped = False
        self.initialize = False

        self.cmdvel_x = 0.00
        self.cmdvel_y = 0.00

        rospy.spin()

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
    
    def image_tag_callback(self,image_tag_msg):
        self.cv_image = bridge.imgmsg_to_cv2(image_tag_msg, desired_encoding='bgr8')
        # if self.cv_image is not None:
            # cv2.imshow("TAG_DETECTION HUSKY1", self.cv_image)
            # cv2.waitKey(1)
        # else:
            # print("----------WAITING FOR CAMERA FRAME---------------")
    
    def patrol_cont_callback(self,patrol_cont_msg):
        CONT_PATROL = patrol_cont_msg
    
    
    #def cmdvel_callback(self,cmdvel_msgs):
    #    self.cmdvel_x = cmdvel_msgs.linear.x
    #    self.cmdvel_y = cmdvel_msgs.linear.y

    def odometry_callback(self,odometry_msgs):
        self.cmdvel_x = odometry_msgs.twist.twist.linear.x
        self.cmdvel_y = odometry_msgs.twist.twist.linear.y
        #print("&&&&&&&&&&&TWIST_X: " +str(self.cmdvel_x) + "TWIST_Y: " +str(self.cmdvel_y)) 

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
            CONT_PATROL = PATROL_RESUME
    #TESTAR MODIFICAR A FORMA COMO O MOVIMENTO DO HUSKY1 É REALIZADA
    def goal_cancel(self):
        self.client.cancel_all_goals()
        self.stopped = True
        PATROL_RESUME = CONT_PATROL


    def get_frameRef(self):
        if self.cv_image is not None:
            if self.cv_image.any() > 0:
                self.current_frame = self.cv_image
        else:
            print("############CV_IMAGE IS EMPTY################")

    #TESTAR MELHORAR A FORMA COMO O PRIMEIRO FRAME É OBTIDO, PARA DIMINUIR O RUÍDO NA IMAGEM
    def tracking(self):
        if self.current_frame is not None:
            if (self.cmdvel_x <= 0.0001 or self.cmdvel_x >= -0.0001) and (self.cmdvel_y <= 0.0001 or self.cmdvel_y >= -0.0001 or self.cmdvel_y == 0.00):
                current_frame_gray = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2GRAY)
                print_frame = self.current_frame
                # current_frame_gray = cv2.GaussianBlur(current_frame_gray,(21,21),0)

                if not self.alredy_detect:
                    print("RECEBIDO")
                    self.first_frame = current_frame_gray
                    self.alredy_detect = True
                frame_delta = cv2.absdiff(self.first_frame,current_frame_gray)
                #TESTAR DIMINUIR UM POUCO O VALOR DO THRESHOLD 
                threshold = cv2.threshold(frame_delta, 20, 255, cv2.THRESH_BINARY)[1]
                #TESTAR AMANHÃ, COLOCAR O KERNEL NO DILATE E TESTAR VER SE MELHORA O RESULTADO 
                kernel = np.ones((5,5))
                threshold = cv2.dilate(threshold, kernel, iterations=2)
                contours = cv2.findContours(threshold.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours = imutils.grab_contours(contours)
                for c in contours:
                    if cv2.contourArea(c) < 500:
                        continue
                    (x,y,w,h) = cv2.boundingRect(c)
                    cv2.rectangle(print_frame, (x,y), (x+w, y+h), (0,255,0),2)
                cv2.imshow("DELTA", frame_delta)
                cv2.waitKey(1)
                # cv2.imshow("FIRST_FRAME", self.first_frame)
                # cv2.waitKey(1)
                # cv2.imshow("REF_FRAME", current_frame_gray)
                # cv2.waitKey(1)
                cv2.imshow("PRINT_FRAME", print_frame)
                cv2.waitKey(1)
                # cv2.imshow("TAG_DETECTION HUSKY1", self.cv_image)
                # cv2.waitKey(1)
            # else:
            #     print(cv2.getWindowProperty("PRINT_FRAME", cv2.WND_PROP_VISIBLE))
            #     if cv2.getWindowProperty("PRINT_FRAME", cv2.WND_PROP_VISIBLE) > 0:
            #         print("WINDOW CLOSED!")
            #         cv2.destroyWindow("PRINT_FRAME")
        else:
            print("@@@@@@@@@@@@@@@@current_frame IS EMPTY@@@@@@@@@@@@@@@@@@@@@@@@")
                
    def goal_sending(self,robot_namespace,goal_point):
        if not self.alredy_detect:
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
        else:
            print("TAG DETECTED, WAITING IT MOVES.")
    


if __name__ == '__main__':
    bridge = CvBridge()
    
    try:
        tg_read = TagRead(ROBOT_NAMESPACE,actionlib.SimpleActionClient(str(ROBOT_NAMESPACE)+"/move_base",MoveBaseAction))
    except rospy.ROSInterruptException:
        print("ROS SYSTEM INTERRUPTED")
        cv2.destroyAllWindows()