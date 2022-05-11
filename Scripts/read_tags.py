from cmath import sqrt
from time import sleep

import initialize_pose

#BIBLIOTECAS PARA ROS(SIMULAÇÃO DO ROBÔ)
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseGoal,MoveBaseResult,MoveBaseAction
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from std_msgs.msg import Int8
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from visualization_msgs.msg import MarkerArray, Marker

#BIBLIOTECAS PARA ANÁLISE DAS IMAGES
import cv2
from scipy.spatial import distance
import actionlib
import imutils
import numpy as np
import message_filters

PATROL = [[13.00,-0.58],[16.41,-0.48],[15.64,4.72],[8.51,5.53],[5.10,0.00]]
MOVEMENT_HUSKY2 = [[13.73,5.63],[10.09,5.73]]
ROBOT_NAMESPACE2 = "/husky2"
ROBOT_NAMESPACE = "/husky1"
CONT_PATROL = 0
PATROL_RESUME = 0

class TagRead:
    def __init__(self,robot_namespace,client):
        print("#####################################INITIALIZE SCRIPT#####################################")
        cont = CONT_PATROL
        cont2 = 0
        rospy.init_node("listener_tag", anonymous=True)     

        cv2.namedWindow("PRINT_FRAME", cv2.WINDOW_AUTOSIZE)

        rospy.Subscriber(str(robot_namespace)+"/tag_detections_image", Image, self.image_tag_callback)
        rospy.Subscriber(str(robot_namespace)+"/tag_detections", AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber(str(robot_namespace)+"/odometry/filtered", Odometry, self.odometry_callback)   
        rospy.Subscriber("/husky2/amcl_pose", PoseWithCovarianceStamped, self.amcl_husky2_callback)   

        pc_pub = rospy.Publisher("/patrol_cont", Int8, queue_size=10)
        self.path_kalman = rospy.Publisher("/kalman_path", MarkerArray, queue_size=10)
        self.cmd_vel = rospy.Publisher(str(robot_namespace)+"/cmd_vel", Twist,queue_size=10)

        self.client = client
        self.alredy_detect = False
        self.cv_image = np.zeros((480,640,3), dtype=np.uint8)
        self.current_frame = None
        self.first_frame = None
        self.stopped = False
        self.initialize = False

        self.cont_kalman = 0
        self.f_ant = []
        self.f_cur = []

        self.cmdvel_x = 0.00
        self.cmdvel_y = 0.00

        self.h2_x = 0.00
        self.h2_y = 0.00
        self.h2_qz = 0.00
        self.h2_qw = 0.00
        self.pred_ant = (0,0)
        self.measured = np.array((2, 1), np.float32)
        self.predicted = np.zeros((2, 1), np.float32)
        self.kalman = cv2.KalmanFilter(4, 2, 0)
        self.kalman_filter_predict()
        self.r = 10.0

        # husky2_client = actionlib.SimpleActionClient(str(ROBOT_NAMESPACE2)+"/move_base",MoveBaseAction)

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
            # result = self.goal_sending_object(ROBOT_NAMESPACE2,MOVEMENT_HUSKY2[cont2],husky2_client)
            # if result:
            #     if (cont2 >= len(MOVEMENT_HUSKY2)-1):
            #         cont2 = 0
            #     else:
            #         cont2 += 1
        # rospy.spin()
    # def init_system(self,robot_namespace,client):
    
    def image_tag_callback(self,image_tag_msg):
        self.cv_image = bridge.imgmsg_to_cv2(image_tag_msg, desired_encoding='bgr8')
        # if self.cv_image is not None:
            # cv2.imshow("TAG_DETECTION HUSKY1", self.cv_image)
            # cv2.waitKey(1)
        # else:
            # print("----------WAITING FOR CAMERA FRAME---------------")
    
    def patrol_cont_callback(self,patrol_cont_msg):
        CONT_PATROL = patrol_cont_msg

    def odometry_callback(self,odometry_msgs):
        self.cmdvel_x = odometry_msgs.twist.twist.linear.x
        self.cmdvel_y = odometry_msgs.twist.twist.linear.y
    
    def amcl_husky2_callback(self, amcl_msgs):
        self.h2_x = amcl_msgs.pose.pose.position.x
        self.h2_y = amcl_msgs.pose.pose.position.y
        self.h2_qz = amcl_msgs.pose.pose.orientation.z
        self.h2_qw = amcl_msgs.pose.pose.orientation.w

    def tag_callback(self,tag_msg):
        
        if (len(tag_msg.detections) > 0):
            # print("TAG ENCONTRADA")
            self.goal_cancel()
            self.stop_cmdvel()
            self.get_frameRef()
            self.tracking()         
        else:
            self.alredy_detect = False
            self.first_frame = None
            self.stopped = False
            CONT_PATROL = PATROL_RESUME
            # cv2.destroyAllWindows()

    def goal_cancel(self):
        self.client.cancel_all_goals()
        self.stopped = True
        PATROL_RESUME = CONT_PATROL

    def stop_cmdvel(self):
        cmd = Twist()
        cmd.linear.x = 0.00
        cmd.linear.y = 0.00
        cmd.angular.z = 0.00
        self.cmd_vel.publish(cmd)

    def get_frameRef(self):
        if self.cv_image is not None:
            if self.cv_image.any() > 0:
                self.current_frame = self.cv_image
        else:
            print("############CV_IMAGE IS EMPTY################")

    def kalman_filter_predict(self):    

        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0]], np.float32)

        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                    [0, 1, 0, 1],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], np.float32)

        self.kalman.processNoiseCov = np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]], np.float32) * 0.003

        self.kalman.measurementNoiseCov = np.array([[1, 0],
                                                        [0, 1]], np.float32) * 0.1 
    
    def estimate_kalman(self,coordX,coordY):                                         
        self.measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kalman.correct(self.measured)
        self.predicted = self.kalman.predict()
        # print("PREDICTED: " + str(self.predicted))     
        return self.predicted

    def tracking(self):
        if self.current_frame is not None:
            if (self.cmdvel_x <= 0.0001 and self.cmdvel_x >= -0.0001) and (self.cmdvel_y <= 0.0001 and self.cmdvel_y >= -0.0001):
                current_frame_gray = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2GRAY)
                print_frame = self.current_frame

                if not self.alredy_detect:
                    print("RECEBIDO")
                    self.first_frame = current_frame_gray
                    self.alredy_detect = True
                frame_delta = cv2.absdiff(self.first_frame,current_frame_gray)
                #TESTAR DIMINUIR UM POUCO O VALOR DO THRESHOLD 
                threshold = cv2.threshold(frame_delta, 23, 255, cv2.THRESH_BINARY)[1]
                threshold = cv2.dilate(threshold, None, iterations=2)
                contours = cv2.findContours(threshold.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours = imutils.grab_contours(contours)
                for c in contours:
                    if cv2.contourArea(c) < 500:
                        continue
                    (x,y,w,h) = cv2.boundingRect(c)
                    cv2.rectangle(print_frame, (x,y), (x+w, y+h), (0,255,0),2)
                    
                    center_x = x+(w/2)
                    center_y = y+(h/2)
                    
                    # pred = self.estimate_kalman(center_x,center_y)
                    pred = self.estimate_kalman(x,y)

                    # print("PRED__X: " + str(pred[0][0]) + " PRED__Y: " + str(pred[1][0]))
                    # print("PRED_ANT_X: " + str(self.pred_ant[0]) + " PRED_ANT_Y: " + str(self.pred_ant[1]))
                    # if (((x+(w/2))-pred[0][0]) != self.pred_ant[0]) or (((y+(h/2))-pred[1][0]) != self.pred_ant[1]):
                        # print("PRINTANDO O PATH")
                    self.pred_ant = self.path_publish((pred[0][0]/center_x,pred[1][0]/center_y))
                    # self.pred_ant = self.path_publish((pred[0][0]/x,pred[1][0]/y))
                        
                    # print("BOUNDING BOX: X: " + str(x) + " | Y: " + str(y) + " | W: " + str(w) + " | H: " + str(h))
                    # if len(c) > 1:
                    #     self.alredy_detect = False
                    
                    # cv2.circle(print_frame, (int(pred[0]), int(pred[1])), 10, [0, 0, 255], 2, 8)
                    cv2.rectangle(print_frame, (int(pred[0]), int(pred[1])), (int(pred[0])+w, int(pred[1])+h), (255,0,0),2)
                cv2.imshow("PRINT_FRAME", print_frame)
                cv2.waitKey(1)
            # else:
            #     print(cv2.getWindowProperty("PRINT_FRAME", cv2.WND_PROP_VISIBLE))
            #     if cv2.getWindowProperty("PRINT_FRAME", cv2.WND_PROP_VISIBLE) > 0:
            #         print("WINDOW CLOSED!")
            #         cv2.destroyWindow("PRINT_FRAME")
        else:
            print("@@@@@@@@@@@@@@@@current_frame IS EMPTY@@@@@@@@@@@@@@@@@@@@@@@@")

    def path_publish(self, predict_norm):
        path = MarkerArray()
        mark = Marker()
        mark.header.stamp = rospy.get_rostime()
        mark.header.frame_id = "map"
        print("H2_X_PLUS: " + str(self.h2_x + predict_norm[0]) + " H2_Y_PLUS: " + str(self.h2_y + predict_norm[1]))
        mark.pose.position.x = self.h2_x + predict_norm[0]
        mark.pose.position.y = self.h2_y + predict_norm[1]
        mark.pose.orientation.z = self.h2_qz
        mark.pose.orientation.w = self.h2_qw
        mark.color.r = self.r
        mark.color.g = 0.0
        mark.color.b = 0.0
        mark.color.a = 1.0
        mark.scale.x = 0.4
        mark.scale.y = 0.1
        mark.scale.z = 0.1
        path.markers.append(mark)
        self.path_kalman.publish(path)
        return predict_norm

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
            print("TAG DETECTED, WAITING ROBOT TO MOVE.")
    def goal_sending_object(self,robot_namespace,goal_point,client):
        print("#######SENDING GOAL")
            
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_point[0]
        goal.target_pose.pose.position.y = goal_point[1]
        goal.target_pose.pose.orientation.w = 1.0
        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()
    


if __name__ == '__main__':
    bridge = CvBridge()    
    try:
        tg_read = TagRead(ROBOT_NAMESPACE,actionlib.SimpleActionClient(str(ROBOT_NAMESPACE)+"/move_base",MoveBaseAction))
    except rospy.ROSInterruptException:
        print("ROS SYSTEM INTERRUPTED")
        cv2.destroyAllWindows()