#! /usr/bin/python
import rospy
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge, CvBridgeError
import cv2
import glob
import os
import argparse

def imu(inputNumber, imuData, imuTimes, startTimeCurrent):
    publisher = rospy.Publisher('/imu' + str(inputNumber), Imu, queue_size=10)
    
    for line, time in zip(imuData, imuTimes):  
        now = rospy.Time.now()    
        relativeCurrentTime = now - startTimeCurrent
        deltaTime = time - relativeCurrentTime
    
        rospy.loginfo("Load imu " + str(time.to_sec()))
        rospy.sleep(deltaTime)
        
        message = Imu()
        message.header.stamp = startTimeCurrent + time
        message.header.frame_id = "imu0"
        message.angular_velocity.x = line[1]
        message.angular_velocity.y = line[2]
        message.angular_velocity.z = line[3]
        message.linear_acceleration.x = line[4]
        message.linear_acceleration.y = line[5]
        message.linear_acceleration.z = line[6]
        publisher.publish(message)
        
        rospy.loginfo("Show imu " + str(line))
    
    
def image(inputNumber, files, imageTimes, startTimeCurrent):  
    bridge = CvBridge()
    publisher1 = rospy.Publisher('/cam0/image_raw' + str(inputNumber), Image, queue_size=10)
    
    firstFile = files[0]
    
    for file, time in zip(files, imageTimes):
        now = rospy.Time.now()    
        relativeCurrentTime = now - startTimeCurrent
        deltaTime = time - relativeCurrentTime
        
        rospy.loginfo("Load image " + str(time.to_sec()))
        rospy.sleep(deltaTime)
        
        image = cv2.imread(file)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        message = bridge.cv2_to_imgmsg(image, 'mono8')
        message.header.stamp = startTimeCurrent + time
        message.header.frame_id = "cam0"
        
        publisher1.publish(message)
        
        rospy.loginfo("Show image " + file)


def main():
    parser = argparse.ArgumentParser("RosPlayer")
    parser.add_argument("input_number", help="The input number for this", type=int)
    args = parser.parse_args()

    rospy.init_node('ros_player_'+str(args.input_number))
    
    csvFile = open("/root/covins_ws/dataset/imu.csv", "r")
    imuData = [map(lambda l: float(l), line.split(",")) for line in csvFile if line[0] != "#" ]
    imuTimes = [rospy.Time.from_sec(float(line[0]) * 1e-9) for line in imuData]
    
    files = glob.glob("/root/covins_ws/dataset/images/*.png")
    files.sort() 
    
    imageTimes = []
    for path in files:
        name = os.path.splitext(os.path.basename(path))[0]
        time = rospy.Time.from_sec(float(name) * 1e-9)
        imageTimes.append(time)
    
    startTime = min(imageTimes[0], imuTimes[0])
    imuTimes = list(map(lambda time: time - startTime, imuTimes))
    imageTimes = list(map(lambda time: time - startTime, imageTimes))
    
    startTimeCurrent = rospy.Time.now()
    
    rospy.Timer(rospy.Duration(0.01), lambda _: imu(args.input_number, imuData, imuTimes, startTimeCurrent), oneshot=True)
    rospy.Timer(rospy.Duration(0.01), lambda _: image(args.input_number, files, imageTimes, startTimeCurrent), oneshot=True)
    
    rospy.spin()
    

if __name__ == '__main__':
    main()
