from cmath import sqrt
from time import sleep

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

#BIBLIOTECAS PARA ANÁLISE DAS IMAGES
import cv2
from scipy.spatial import distance
import actionlib
import imutils
import numpy as np
import message_filters

PATROL = [[13.00-7,-0.58],[16.41-7,-0.48],[15.64-7,4.72],[8.51-7,5.53],[5.10-7,0.00]]
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
        rospy.Subscriber("/husky2/amcl_pose", PoseWithCovarianceStamped, self.amcl_husky2_callback)   

        # rospy.Subscriber("/patrol_cont", Int8, self.patrol_cont_callback)
        pc_pub = rospy.Publisher("/patrol_cont", Int8, queue_size=10)
        self.path_kalman = rospy.Publisher("/kalman_path", Path, queue_size=10)

        self.client = client
        self.alredy_detect = False
        self.cv_image = np.zeros((480,640,3), dtype=np.uint8)
        self.current_frame = None
        self.first_frame = None
        self.stopped = False
        self.initialize = False

        self.contador = 0

        self.cont_kalman = 0
        self.f_ant = []
        self.f_cur = []

        self.cmdvel_x = 0.00
        self.cmdvel_y = 0.00

        self.h2_x = 0.00
        self.h2_y = 0.00

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
            cv2.destroyAllWindows()
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

    def compute_direction(self,x,y,x_i,y_i):
        z = sqrt(pow(x,2)+pow(y,2))
        z_i = sqrt(pow(x_i,2)+pow(y_i,2))
        if z_i > z:
            return "apr"
        elif z_i < z:
            return "dis"

    def kalman_filter_predict(self):
        kalman = cv2.KalmanFilter(4,2)
        kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        return kalman
    
    def estimate_kalman(self,coordX,coordY):
        kalman = cv2.KalmanFilter(4, 2, 0)

        measured = np.array((2, 1), np.float32)
        predicted = np.zeros((4, 1), np.float32)

        kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0]], np.float32)

        kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                    [0, 1, 0, 1],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], np.float32)

        kalman.processNoiseCov = np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]], np.float32) * 0.0001

        kalman.measurementNoiseCov = np.array([[1, 0],
                                                        [0, 1]], np.float32) * 0.1                                   

        
        
        predicted = kalman.predict()
        
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        # print("ANTES: " + str(measured))
        kalman.correct(measured)
        # print("DEPOIS: " + str(measured))
        # print("X_PREDITO: " + str(predicted[0]) + " | Y_PREDITO: " + str(predicted[1]))
        return measured


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
                threshold = cv2.dilate(threshold, None, iterations=2)
                contours = cv2.findContours(threshold.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours = imutils.grab_contours(contours)
                for c in contours:
                    if cv2.contourArea(c) < 500:
                        continue
                    (x,y,w,h) = cv2.boundingRect(c)
                    cv2.rectangle(print_frame, (x,y), (x+w, y+h), (0,255,0),2)
                    # kalman = self.kalman_filter_predict()
                    pred = self.estimate_kalman(x+w/2,y+h/2)
                    self.path_publish(((x+w/2)-pred[0],(y+h/2)-pred[1]))
                    # print("BOUNDING BOX: X: " + str(x) + " | Y: " + str(y) + " | W: " + str(w) + " | H: " + str(h))
                    # if len(c) > 1:
                    #     self.alredy_detect = False
                    
                    cv2.circle(print_frame, (int(pred[0]), int(pred[1])), 20, [0, 0, 255], 2, 8)
                # cv2.imshow("DELTA", frame_delta)
                cv2.imwrite("delta"+str(self.contador)+".png", frame_delta)
                # cv2.waitKey(1)
                # cv2.imshow("THRESHOLD", threshold)
                cv2.imwrite("threshold"+str(self.contador)+".png", threshold)
                # cv2.waitKey(1)
                # cv2.imshow("FIRST_FRAME", self.first_frame)
                # cv2.waitKey(1)
                # cv2.imshow("REF_FRAME", current_frame_gray)
                # cv2.waitKey(1)
                # cv2.imshow("PRINT_FRAME", print_frame)
                cv2.imwrite("print_frame"+str(self.contador)+".png", print_frame)

                self.contador = self.contador + 1
                
                # cv2.waitKey(1)
                # cv2.imshow("TAG_DETECTION HUSKY1", self.cv_image)
                # cv2.waitKey(1)
            # else:
            #     print(cv2.getWindowProperty("PRINT_FRAME", cv2.WND_PROP_VISIBLE))
            #     if cv2.getWindowProperty("PRINT_FRAME", cv2.WND_PROP_VISIBLE) > 0:
            #         print("WINDOW CLOSED!")
            #         cv2.destroyWindow("PRINT_FRAME")
        else:
            print("@@@@@@@@@@@@@@@@current_frame IS EMPTY@@@@@@@@@@@@@@@@@@@@@@@@")

    def path_publish(self, predict_norm):
        path = Path()
        pose = PoseStamped()
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = "map"
        print("H2_X: " + str(self.h2_x) + " H2_Y: " + str(self.h2_y))
        pose.pose.position.x = self.h2_x + predict_norm[0]
        pose.pose.position.y = self.h2_y + predict_norm[1]
        path.poses.append(pose)
        # pose.position.x = self.h2_x + predict_norm[0]
        # path.poses.pose.position.y = self.h2_y + predict_norm[1]
        self.path_kalman.publish(path)

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
    


if __name__ == '__main__':
    bridge = CvBridge()
    
    try:
        tg_read = TagRead(ROBOT_NAMESPACE,actionlib.SimpleActionClient(str(ROBOT_NAMESPACE)+"/move_base",MoveBaseAction))
    except rospy.ROSInterruptException:
        print("ROS SYSTEM INTERRUPTED")
        cv2.destroyAllWindows()