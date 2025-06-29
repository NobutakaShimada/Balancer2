import rospy
import threading
from flask import Flask, render_template
import rosparam

root_path = rosparam.get_param("/flask_server/flask_root_path")
if root_path != "":
    app = Flask(__name__, root_path=root_path)
else:
    app = Flask(__name__)    

print(app.root_path)            # ここをチェックする
    
threading.Thread(target=lambda: rospy.init_node('ros_flask_test', disable_signals=True)).start()

@app.route('/')
def root_page():
    return "Hello World"

@app.route('/some_page')
def some_page():
    return render_template("some_page.html")

if __name__ == '__main__':    
    app.run(debug=True, port=11000)

