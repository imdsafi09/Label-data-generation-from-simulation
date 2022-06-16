#!/usr/bin/env python

# ROS python API
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped,Twist,Vector3
# import all mavros messages and services
import os
import cv2
from airsim_ros_pkgs.msg import VelCmd
import airsim
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
# Main class: Converts joystick commands to position setpoints
class Controller:
    # initialization method
    def __init__(self):
        self._cv_bridge = CvBridge()


        self.im  = Image()
        self.client = airsim.CarClient()
        self.client.confirmConnection()




    def getScreenRGB(self):

        responses = self.client.simGetImages([airsim.ImageRequest("2", airsim.ImageType.Scene, False, False)])
        responses1 = self.client.simGetImages([airsim.ImageRequest(0, airsim.ImageType.Segmentation, False, False)])
        response = responses[0]
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
        if ((responses[0].width != 0 or responses[0].height != 0)):
            img_rgba = img1d.reshape(response.height, response.width, 3)
            rgb = cv2.cvtColor(img_rgba, cv2.COLOR_BGR2RGB)
        else:
            print("Something bad happened! Restting AirSim!")
            self.AirSim_reset()
            rgb = np.ones(480, 640, 3)
        response = responses1[0]
        img2d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
        if ((responses1[0].width != 0 or responses1[0].height != 0)):
            img_rgba = img2d.reshape(response.height, response.width, 3)

            seg = cv2.cvtColor(img_rgba, cv2.COLOR_BGR2RGB)
        else:
            print("Something bad happened! Restting AirSim!")
            self.AirSim_reset()
            seg = np.ones(480, 640, 3)

        return rgb,seg


# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    cnt = Controller()
    rate = rospy.Rate(1.0)



    input_image = []
    input_seg = []

    path_rgb = '/home/imad/Desktop/Segmentation/RGB'
    path_seg = '/home/imad/Desktop/Segmentation/SEG'         # Destination/path to which all the images will be saved
    num  = 0


    while not rospy.is_shutdown():
        img,seg = cnt.getScreenRGB()
        #img = np.array(img)
        image_array = []
        seg_array = []
        num  =  num + 1
        imageName = 'Image_No'+ str(num) + '.png'
        image_array.append(img)
        input_image = image_array

        seg_array.append(seg)
        input_seg = seg_array




        cv2.imwrite(os.path.join(path_rgb, imageName), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(path_seg, imageName), cv2.cvtColor(seg, cv2.COLOR_RGB2BGR))
        print('im saving')                                    # Trying to save the image in the exact same directory.



        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
