
#!/usr/bin/env python3

import rospy
import actionlib
import threading
import os
import rosparam

from flask import Flask, request, render_template, redirect, abort

from std_msgs.msg import UInt32

from beuato_monitor_test.msg import BeuatoBalancerAction, BeuatoBalancerGoal
from common_driver.beuato_errorcode import BeuatoErrorCode

action_client = actionlib.SimpleActionClient('my_beuato', BeuatoBalancerAction)

class DataBoard:
    _max_count = 0    
    _errorcode = BeuatoErrorCode.OK
    
    def __init__(self, *args, **kwargs):
        self._data = []
        self._error_reason = None

        if 'max_count' in kwargs:
            self._max_count = kwargs['max_count']
        
    def add(self, value):
        rospy.loginfo('add value:{0}'.format(value))
        self._data.append(value)
        
        if self._max_count == 0:
            return
        
        if self._data.count >= self._max_count:
            num = self._data.count - self._max_count
            
            for i in range(0, num):
                self._data.pop(0)
    
    def give_errorcode(self, errorcode):
        self._errorcode = BeuatoErrorCode(errorcode)
    
    def get_errorcode(self):
        return self._errorcode.value
    
    def refer(self):
        return self._data
    
    def clear(self):
        self._errorcode = BeuatoErrorCode.OK
        return self._data.clear()



root_path = rosparam.get_param("/beuato_client_controller/flask_root_path")
if root_path != "":
    app = Flask(__name__, root_path=root_path)
else:
    app = Flask(__name__)    
    

data_board = DataBoard()

def init():
    print(app.root_path)
    rospy.init_node('beuato_client_controller', disable_signals=True)
    action_client.wait_for_server()

# https://answers.ros.org/question/234418/easiest-way-to-implement-http-server-that-can-send-ros-messages/
threading.Thread(target=lambda: init()).start()

def receive_callback(feedback):
    data_board.add(feedback.ad_gyro)

def complete_callback(status, result):
    rospy.loginfo('complete callback')
    if(result.is_suspend_for_error):
        data_board.give_errorcode(result.errorcode)

@app.route('/')
def index():
    return redirect('/dashboard')


@app.route('/dashboard')
def index_dashboard():
    return render_template('/dashboard/index.html')


@app.route('/capture_start')
def capture_start():   
    data_board.clear()
    goal = BeuatoBalancerGoal()
    goal.is_capture_mode = True
    action_client.send_goal(goal, feedback_cb=receive_callback, done_cb=complete_callback)
    
    return redirect('/dashboard')
    
@app.route('/capture_stop')
def capture_stop():
    action_client.cancel_goal()
    
    return redirect('/dashboard')

@app.route('/sampling', methods=["GET"])
def run_sampling():
    data_board.clear()
    number = request.args.get('number')
    if number == None:
        abort(400)
    
    goal = BeuatoBalancerGoal()
    goal.sampling_number = int(number)
    action_client.send_goal(goal, feedback_cb=receive_callback, done_cb=complete_callback)
    
    return redirect('/dashboard')


@app.route('/api/recent')
def get_recent_data_api():
    transmit_data = { 
        'ad_gyro' : data_board.refer(),
        'sampling_number' : len(data_board.refer())
    }
    
    errorcode = data_board.get_errorcode()
    transmit_data['is_error'] = errorcode != BeuatoErrorCode.OK.value
    transmit_data['errorcode'] = errorcode

    return transmit_data


if __name__ == '__main__':    
    app.run(debug=True)

