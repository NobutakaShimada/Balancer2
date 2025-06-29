
#!/usr/bin/env python3

import rospy
import actionlib

from beuato_monitor_test.msg import BeuatoBalancerAction, BeuatoBalancerGoal

def receive_callback(feedback):
    print(feedback)

def beuato_client():
    client = actionlib.SimpleActionClient('my_beuato', BeuatoBalancerAction)
    client.wait_for_server()
    goal = BeuatoBalancerGoal()
    goal.sampling_number = 10
    goal.is_capture_mode = True
    
    duration = rospy.Duration(10)
    
    client.send_goal(goal, feedback_cb=receive_callback)
    client.wait_for_result(timeout=duration)
    
    if goal.is_capture_mode == True:
        client.cancel_goal()
    
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('beuato_client')
    rospy.loginfo('simple action client started')
    result = beuato_client()
    
    if result != None:
        print('result ', result.sampling_number)

