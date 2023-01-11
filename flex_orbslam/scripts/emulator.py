#!/usr/bin/env python2
import rospy
import rosbag
from std_msgs.msg import String, Int32, Float32, Bool
from sensor_msgs.msg import Image
# from buffer import MAX_VELOCITY, LEAST_VELOCITY

current_frame = 0
velocity = 0
published_messages = 0
prev_img = None
IMG_PUB = rospy.Publisher('/camera/images', Image, queue_size=1)
IMG_PUB_RATE = rospy.Duration(0.2)
img_pub_timer = None

SHUTDOWN_PUB = rospy.Publisher('/shutdown', Bool, queue_size=1)

# Reading bagfile from the path where bagfile is stored
bag = rosbag.Bag('/home/vivekw964/vivek_ws/src/flex_orbslam/scripts/MH_01_easy.bag')
BAG_READ = bag.read_messages(topics =['/cam0/image_raw', '/imu', '/position'])
print("Reading bag file")
IMAGES_SIZE = sum(1 for _ in BAG_READ)  #getting length size of bagfile (number of image messages present in it)
print("Number of image messages in ROS bag: ", IMAGES_SIZE)
#After reading bagfile, pointer is set at the start of bagfile to pint at first image in it
BAG_READ = bag.read_messages(topics =['/cam0/image_raw', '/imu', '/position'])
bag_pos = 0

def velCallback(callback_value):

    # rospy.loginfo("Velocity received\n")
    print ('Received velocity: ', callback_value.data)
    global velocity, frames_skipped
    
    velocity = int(round(callback_value.data))
    print('Integer velocity: ', velocity)

    #before publishing the image, updating the number  of frames to be skipped before publishing


def shutdownROS():
    global img_pub_timer
    print("Shutting down ROS")
    img_pub_timer.stop()


def publish_image(event=None):

    global prev_img, bag_pos, IMG_PUB, current_frame, published_messages, frames_skipped, BAG_READ, IMAGES_SIZE, SHUTDOWN_PUB, velocity
    
    #boundary condition to be checked before publishing the image
    current_frame = current_frame + velocity
    if current_frame < IMAGES_SIZE:
        image = prev_img
        # moving bag position to current frame ID for publishing
        while bag_pos <= current_frame:
            image = next(BAG_READ)
            bag_pos += 1
        prev_img = image

        IMG_PUB.publish(image[1])
        print("Frame being published: ", current_frame)
        published_messages += 1
        print('Published messages till now: ', published_messages)
        print('--------------------------------')
    else:
        #shutting down if all images from bag file are published
        SHUTDOWN_PUB.publish(True)
        rospy.signal_shutdown("Stopping Publishing")

def mainf():
    global IMG_PUB_RATE, img_pub_timer
    print("Waiting for velocity...\n")
    rospy.init_node('emulator', anonymous=True)
    rospy.Subscriber("robot_velocity", Float32, velCallback)
    img_pub_timer = rospy.Timer(IMG_PUB_RATE, publish_image)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        mainf()
    except rospy.ROSInterruptException:
        rospy.logwarn("There is something wrong with the program!")
