
# https://roboticsbackend.com/ros-import-python-module-from-another-package/

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['common_driver'],
    package_dir={'': 'scripts'},
    requires=['std_msgs', 'rospy']
)
setup(**d)