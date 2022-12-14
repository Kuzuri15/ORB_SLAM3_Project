#!/usr/bin/env python2
import rospy
import rosbag
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

vel_pub = rospy.Publisher("robot_velocity", Int32, queue_size=1)
img_pub = rospy.Publisher('/camera/image_raw', Image, queue_size = 1)


def imagecallback(img):
    # print("i am inside img callback\n")
    global img_pub
    img_msg = img
    rate = rospy.Rate(2)
    img_pub.publish(img_msg)
    print("image published")
    rospy.sleep(0.1)


def tlosscallback(flag):
    global vel_pub
    t_flag = flag.data
    rate = rospy.Rate(2)
    print("flag value in buffer:", t_flag)

    if t_flag == False:
        velocity = 3
        vel_pub.publish(velocity)
        rospy.sleep(0.1)
    elif t_flag == True:
        velocity = 0
        vel_pub.publish(velocity)
        rospy.sleep(0.1)


def main_func():
    print("Starting...")
    global vel_pub
    rospy.init_node('buffer', anonymous=True)
    rospy.Subscriber("/camera/images", Image, imagecallback)
    rospy.Subscriber("/track_loss", Bool, tlosscallback)
    # rospy.sleep(0.1)
    velo = 3
    count = 0
    while not rospy.is_shutdown():
        # print("reached in while")
        if count <= 3:
            # print(count)
            vel_pub.publish(velo)
            count += 1
            rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        main_func()

    except rospy.ROSInterruptException:
        pass

