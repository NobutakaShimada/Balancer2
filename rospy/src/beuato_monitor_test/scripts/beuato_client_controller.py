
#!/usr/bin/env python3

import rospy
import actionlib
import threading

from flask import Flask, request, abort

from std_msgs.msg import UInt32

from beuato_monitor_test.msg import BeuatoBalancerAction, BeuatoBalancerGoal


action_client = actionlib.SimpleActionClient('my_beuato', BeuatoBalancerAction)

def init():
    rospy.init_node('beuato_client_controller', disable_signals=True)
    action_client.wait_for_server()

def ros_callback(msg):
    print(msg)

# https://answers.ros.org/question/234418/easiest-way-to-implement-http-server-that-can-send-ros-messages/
threading.Thread(target=lambda: init()).start()


app = Flask(__name__)

def receive_callback(feedback):
    print(feedback)

@app.route('/')
def hello_world():
    return 'Hello, World!'

@app.route('/capture_start')
def capture_start():   
    goal = BeuatoBalancerGoal()
    goal.is_capture_mode = True
    action_client.send_goal(goal, feedback_cb=receive_callback)
    
    return 'Capture Start'
    
@app.route('/capture_stop')
def capture_stop():
    action_client.cancel_goal()
    
    return 'Capture Stop'

@app.route('/sampling', methods=["GET"])
def run_sampling():
    number = request.args.get('number')
    if number == None:
        abort(400)
    
    goal = BeuatoBalancerGoal()
    goal.sampling_number = int(number)
    action_client.send_goal(goal, feedback_cb=receive_callback)
    
    return 'Sampling...'
    

def receive_callback(feedback):
    print(feedback)

if __name__ == '__main__':    
    app.run(debug=True)

