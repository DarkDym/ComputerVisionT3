import rospy
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction
import actionlib


MOVEMENT_HUSKY2 = [[13.73,5.63],[10.09,5.73]]
ROBOT_NAMESPACE2 = "/husky2"

class husky2:
    def __init__(self):
        print("SENDING")
        rospy.init_node("move_husky2")

    def goal_sending_object(self,goal_point,client):
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
    move_husky = husky2()
    cont2 = 0
    husky2_client = actionlib.SimpleActionClient(str(ROBOT_NAMESPACE2)+"/move_base",MoveBaseAction)
    while not rospy.is_shutdown():
        result = move_husky.goal_sending_object(MOVEMENT_HUSKY2[cont2],husky2_client)
        if result:
            if (cont2 >= len(MOVEMENT_HUSKY2)-1):
                cont2 = 0
            else:
                cont2 += 1