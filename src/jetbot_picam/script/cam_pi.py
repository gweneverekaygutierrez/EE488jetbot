#!/usr/bin/python

# Old: !/usr/bin/env python
import cv2
import time
import os
from PIL import Image, ImageEnhance
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CAM:
    def gstreamer_pipeline (self, capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=10, flip_method=2) :   
        return ('nvarguscamerasrc ! ' 
        'video/x-raw(memory:NVMM), '
        'width=(int)%d, height=(int)%d, '
        'format=(string)NV12, framerate=(fraction)%d/1 ! '
        'nvvidconv flip-method=%d ! '
        'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))


    def make_folder_img(self):
        saving_folder = "/home/jetbot/img_test"
        local_folder = time.localtime()
        local_folder = "%04d-%02d-%02d-%02d:%02d:%02d" % (local_folder.tm_year, local_folder.tm_mon, local_folder.tm_mday, local_folder.tm_hour, local_folder.tm_min, local_folder.tm_sec)
        self.location = saving_folder+local_folder
        os.makedirs(os.path.join(saving_folder+local_folder))
    
    def SAVE_IMG(self, now, img):
        name = self.location + '/' + str(now) + '.jpg'
        cv2.imwrite( name, img)
        

    def run(self):
        # Ros init
        rospy.init_node('cam_pi', anonymous=True)

        print self.gstreamer_pipeline(flip_method=2)
        cap = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)


        if self.save_img is 1 :
            self.make_folder_img()


        # Define Publisher & Subscriber
        image_pub1 = rospy.Publisher("cam_pi", Image, queue_size=1)
        bridge = CvBridge()
        self.prev_time=0

        while not rospy.is_shutdown():
            ret_val, img = cap.read();
            
            if ret_val is True:
                now = time.time()
                if self.save_img is 1:
                    self.SAVE_IMG(now,img)

                img = cv2.flip(img, 0)

                image_pub1.publish(bridge.cv2_to_imgmsg(img, "bgr8"))  
                #print(str(1.0/(now-self.prev_time)) + "fps")
                self.prev_time = now
            else:
                continue
        self.cap.release()
        cv2.destroyAllWindows()
        

################ MAIN ###################

cam = CAM()
cam.save_img = 0; # 1 : save img, 0 : not save img
cam.run()





