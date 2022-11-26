from enum import Enum

#https://roboticsbackend.com/ros-import-python-module-from-another-package/
#https://gist.github.com/zkytony/fb7616b745d77025f6abe919c41f8ea4

class BeuatoErrorCode(Enum):
    OK = 0
    DEVICE_NOT_FOUND = -100
    DEVICE_ACCESIBILITY_ERROR = -101
        

