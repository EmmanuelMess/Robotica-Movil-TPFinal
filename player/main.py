#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import glob
import os

def imu(imuData, imuTimes, startTimeCurrent):
    publisher = rospy.Publisher('/imu0', Imu, queue_size=10)
    
    for line, time in zip(imuData, imuTimes):      
        relativeCurrentTime = rospy.Time.now() - startTimeCurrent
        deltaTime = time - relativeCurrentTime
    
        rospy.loginfo("Load imu " + str(time.to_sec()))
        rospy.sleep(deltaTime)
        
        msg = Imu()
        msg.angular_velocity.x = line[1]
        msg.angular_velocity.y = line[2]
        msg.angular_velocity.z = line[3]
        msg.linear_acceleration.x = line[4]
        msg.linear_acceleration.y = line[5]
        msg.linear_acceleration.z = line[6]
        publisher.publish(msg)
        
        rospy.loginfo("Show imu " + str(line))
    
    
def image(files, imageTimes, startTimeCurrent):  
    bridge = CvBridge()
    publisher = rospy.Publisher('/cam0/image_raw0', Image, queue_size=10)
    
    firstFile = files[0]
    
    for file, time in zip(files, imageTimes):
        relativeCurrentTime = rospy.Time.now() - startTimeCurrent
        deltaTime = time - relativeCurrentTime
        
        rospy.loginfo("Load image " + str(time.to_sec()))
        rospy.sleep(deltaTime)
        
        image = cv2.imread(file)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        publisher.publish(bridge.cv2_to_imgmsg(image, 'mono8'))
        
        rospy.loginfo("Show image " + file)


def main():
    rospy.init_node('image_publisher')
    
    imuData = [map(lambda l: float(l), line.split(",")) for line in open("/root/covins_ws/dataset/mav0/imu0/data.csv", "r")]
    imuTimes = [rospy.Time.from_sec(float(line[0]) * 1e-9) for line in imuData]
    
    files = glob.glob("/root/covins_ws/dataset/mav0/cam0/data/*.png")
    files.sort() 
    
    imageTimes = []
    for path in files:
        name = os.path.splitext(os.path.basename(path))[0]
        time = rospy.Time.from_sec(float(name) * 1e-9)
        imageTimes.append(time)
    
    startTime = imageTimes[0]
    imuTimes = list(map(lambda time: time - startTime, imuTimes))
    imageTimes = list(map(lambda time: time - startTime, imageTimes))
    
    startTimeCurrent = rospy.Time.now()
    
    rospy.Timer(rospy.Duration(0.01), lambda _: imu(imuData, imuTimes, startTimeCurrent), oneshot=True)
    rospy.Timer(rospy.Duration(0.01), lambda _: image(files, imageTimes, startTimeCurrent), oneshot=True)
    
    rospy.spin()
    

if __name__ == '__main__':
    main()
