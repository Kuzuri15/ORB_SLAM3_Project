#!/usr/bin/env python2
import rospy
import rosbag
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

current_frame = 0
published_messages = 0


image_list = []
bag = rosbag.Bag('/home/vivekw964/vivek_ws/src/flex_orbslam/scripts/MH_01_easy.bag')
for topic, msg, t in bag.read_messages(topics =['/cam0/image_raw', '/imu', '/position']):
    image_list.append(msg)

def velcallback(data_vel):

    print ('Received velocity', data_vel.data)
    global velocity, count
    pub = rospy.Publisher('/camera/images', Image, queue_size = 1)
    global image_list, current_frame, published_messages
    rate = rospy.Rate(2)

    velocity = data_vel.data
 
    while current_frame < len(image_list):
        # print("under while loop")
        pub.publish(image_list[current_frame])
        print("Frame being published: ",current_frame)
        published_messages += 1
        print('Published messages till now: ', published_messages)
        current_frame += 1
        current_frame = current_frame + velocity
        rospy.sleep(0.1) #should be 0.05 as waiting time = 1/20fps
        break


def mainf():
    print("Waiting for velocity...\n")
    rospy.init_node('emulator', anonymous=True)
    rospy.Subscriber("robot_velocity", Int32, velcallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        mainf()
    except rospy.ROSInterruptException:
        pass
