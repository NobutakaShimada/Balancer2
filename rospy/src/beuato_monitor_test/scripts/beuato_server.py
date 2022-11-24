
#!usr/env/bin python3

import os
import rospy
import actionlib

from beuato_monitor_test.msg import BeuatoBalancerAction, BeuatoBalancerResult, BeuatoBalancerFeedback

class BeuatoBalancerServer(object):
    _feedback = BeuatoBalancerFeedback()
    _result = BeuatoBalancerResult()
    
    _beuato_dir = "/dev/BeuatoCtrl0"
    _is_connect_device = False
    
    
    def __init__(self, name):
        if os.path.exists(self._beuato_dir) == False:
            rospy.logerr("Not Found Device : {0}".format(self._beuato_dir))
            return
        
        self._is_connect_device = True

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, BeuatoBalancerAction, execute_cb=self.execute_callback, auto_start=False)
        self._as.start()
        
    def execute_callback(self, goal):
        suspend = False
        rate = rospy.Rate(2)
        counter = 0
        for counter in range(0, 1000):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s Preempted' % self._action_name)
                success = True
                break

            with open(self._beuato_dir, 'w') as fout:            
                fout.write('r 68 2 ')
                
            sensor_value = 0
            with open(self._beuato_dir, 'r') as fin:                    
                temp = fin.readline().split(" ")
                temp_arr = int(temp[3], 16).to_bytes(1, 'big') + int(temp[2], 16).to_bytes(1, 'big')
                
                sensor_value = int.from_bytes(temp_arr, 'big', signed=True)
                rospy.loginfo("{0}".format(temp_arr))
                       
            self._feedback.ad_gyro.append(sensor_value)
            self._as.publish_feedback(self._feedback)
            rate.sleep()

        self._result.sampling_number = counter
        self._as.set_succeeded(self._result)

    def is_connected(self):
        return self._is_connect_device

if __name__ == '__main__':
    rospy.init_node('my_beuato')
    server = BeuatoBalancerServer('my_beuato')
    
    if server.is_connected():
        rospy.spin()
