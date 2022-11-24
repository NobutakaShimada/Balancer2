
#!usr/env/bin python3

import os
import sys
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

        sampling_size = sys.maxsize
        if goal.is_capture_mode == False:
            sampling_size = goal.sampling_number
        
        counter = 0
        for i in range(0, sampling_size):
            counter = counter + 1
            
            if self._as.is_preempt_requested():
                rospy.loginfo('%s Preempted' % self._action_name)
                suspend = True
                self._result.sampling_number = counter
                self._as.set_preempted(self._result)
                break

            with open(self._beuato_dir, 'w') as fout:            
                fout.write('r 68 2 ')
                
            sensor_value = 0
            with open(self._beuato_dir, 'r') as fin:                    
                temp = fin.readline().split(" ")
                
                if len(temp) < 4:
                    continue
                
                temp_arr = int(temp[3], 16).to_bytes(1, 'big') + int(temp[2], 16).to_bytes(1, 'big')
                
                sensor_value = int.from_bytes(temp_arr, 'big', signed=True)
                rospy.loginfo("{0}".format(temp_arr))
                       
            self._feedback.ad_gyro = sensor_value
            self._as.publish_feedback(self._feedback)
            rate.sleep()

        if suspend == False:
            self._result.sampling_number = counter
            self._as.set_succeeded(self._result)

    def is_connected(self):
        return self._is_connect_device

if __name__ == '__main__':
    rospy.init_node('my_beuato')
    server = BeuatoBalancerServer('my_beuato')
    
    if server.is_connected():
        rospy.spin()
