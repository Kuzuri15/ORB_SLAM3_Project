#!/usr/bin/env python2
import rospy
import rosbag
import time
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

vel_pub = rospy.Publisher("robot_velocity", Int32, queue_size=1)
img_pub = rospy.Publisher('/camera/image_raw', Image, queue_size = 1)
count = 0
count_2 = 0
count_img = 0
velocity = 0
image_list2 = []

def imagecallback(img):
    global img_pub, count_img, count, velocity
    start_time = time.time()

    # img_msg = img
    image_list2.append(img)
    img_msg = image_list2.pop()
    if image_list2 or img_msg == img:
        if velocity == 0:
            rate = rospy.Rate(20)
            img_pub.publish(img_msg)
            rate.sleep()
        elif velocity != 0:
            rate = rospy.Rate(10)
            img_pub.publish(img_msg)
            rate.sleep()

    count_img += 1
    print("Images published from buffer: ", count_img)
    print 'Processsing time per loop:', time.time() - start_time



def tlosscallback(flag):
    global vel_pub, count, count_2, velocity
    t_flag = flag.data
    rate = rospy.Rate(10)
    print("Flag value in buffer:", t_flag)

    if t_flag == False:  #tracking is going good
        count = 0
        velocity = 3
        vel_pub.publish(velocity)
        rate.sleep()
            
    elif t_flag == True:  #if tracking is lost
        count += 1
        if count < 20:
            velocity = 1
            vel_pub.publish(velocity)
            rate.sleep()
        elif count >=20:  #failing to track for long time or local mapping mode is activated
            velocity = 0
            vel_pub.publish(velocity)
            rate.sleep()


def main_func():
    print("Starting...")
    global vel_pub
    rospy.init_node('buffer', anonymous=True)
    rospy.Subscriber("/camera/images", Image, imagecallback, queue_size = 15)
    rospy.Subscriber("/track_loss", Bool, tlosscallback)
    velo = 1
    ct = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if ct <= 9:
            # print(count)
            vel_pub.publish(velo)
            ct += 1


if __name__ == '__main__':
    try:
        main_func()

    except rospy.ROSInterruptException:
        pass

