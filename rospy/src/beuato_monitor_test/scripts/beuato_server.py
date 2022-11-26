
#!usr/env/bin python3

import os
import sys
import rospy
import actionlib
import stat
import time

from common_driver.beuato_errorcode import BeuatoErrorCode

from beuato_monitor_test.msg import BeuatoBalancerAction, BeuatoBalancerResult, BeuatoBalancerFeedback

class BeuatoBalancerServer(object):
    _feedback = BeuatoBalancerFeedback()
    _result = BeuatoBalancerResult()
    
    _beuato_dir = "/dev/BeuatoCtrl0"
    _is_connect_device = False
    
    
    def __init__(self, name):
        if self.is_device_found() == False:
            rospy.logerr("Not Found Device : {0}".format(self._beuato_dir))
        elif self.is_device_accessible() == False:
            rospy.logerr("Device Accesibility Error : {0}".format(self._beuato_dir))
                
        self._is_connect_device = True

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, BeuatoBalancerAction, execute_cb=self.execute_callback, auto_start=False)
        self._as.start()
        
    def is_device_found(self):
        return os.path.exists(self._beuato_dir)
    
    def is_device_accessible(self):       
        st = os.stat(self._beuato_dir)
        return bool(st.st_mode & (stat.S_IRUSR | stat.S_IWUSR))
    
    def suspend_loop(self, counter):
        self._suspend = True
        self._result.sampling_number = counter
        self._result.is_suspend_for_error = False
        self._as.set_preempted(self._result)
    
    def abort_loop(self, counter, error_state):
        self._suspend = True
        self._result.sampling_number = counter
        self._result.is_suspend_for_error = True
        self._result.errorcode = error_state.value
        self._as.set_succeeded(self._result)
    
    def execute_callback(self, goal):
        self._suspend = False
        rate = rospy.Rate(2)

        sampling_size = sys.maxsize
        if goal.is_capture_mode == False:
            sampling_size = goal.sampling_number
        
        counter = 0
        time_start = time.perf_counter()
        for i in range(0, sampling_size):           
            if self._as.is_preempt_requested():
                rospy.loginfo('%s Preempted' % self._action_name)
                self.suspend_loop(counter)
            elif self.is_device_found() == False :
                rospy.logerr("Not Found Device : {0}".format(self._beuato_dir))
                self.abort_loop(counter, BeuatoErrorCode.DEVICE_NOT_FOUND)
            elif self.is_device_accessible() == False:
                rospy.logerr("Device Accesibility Error : {0}".format(self._beuato_dir))
                self.abort_loop(counter, BeuatoErrorCode.DEVICE_ACCESIBILITY_ERROR)
            
            if self._suspend == True:
                break

            with open(self._beuato_dir, 'w') as fout:            
                fout.write('r 68 2 ')
            
            counter = counter + 1
            sensor_value = 0

            with open(self._beuato_dir, 'r') as fin:                    
                temp = fin.readline().split(" ")
                
                if len(temp) < 4:
                    continue
                
                temp_arr = int(temp[3], 16).to_bytes(1, 'big') + int(temp[2], 16).to_bytes(1, 'big')
                
                sensor_value = int.from_bytes(temp_arr, 'big', signed=True)
                rospy.loginfo("{0}".format(temp_arr))
            
            time_sampling = time.perf_counter()
            self._feedback.time = time_sampling - time_start
            self._feedback.ad_gyro = sensor_value
            self._as.publish_feedback(self._feedback)
            rate.sleep()

        if self._suspend == False:
            self._result.sampling_number = counter
            self._as.set_succeeded(self._result)

    def is_connected(self):
        return self._is_connect_device

if __name__ == '__main__':
    rospy.init_node('my_beuato')
    server = BeuatoBalancerServer('my_beuato')
    
    if server.is_connected():
        rospy.spin()
