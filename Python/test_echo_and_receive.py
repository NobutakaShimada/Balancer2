
import time

if __name__ == '__main__':

    for i in range(0,5) :
        with open('/dev/BeuatoCtrl0', mode='w') as f:
            f.write("r 68 2 ")

        time.sleep(1)
        
        with open('/dev/BeuatoCtrl0', mode='r') as f:
            print(f.readline())
    

    

