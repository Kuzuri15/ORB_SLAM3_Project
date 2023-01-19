#!/usr/bin/env python2
import rospy
import rosbag
from std_msgs.msg import String, Int32, Float32, Bool
from sensor_msgs.msg import Image
import json
from datetime import datetime

current_frame = 0
velocity = 0.0
published_messages = 0
prev_img = None
IMG_PUB = rospy.Publisher('/camera/images', Image, queue_size=1)
IMG_PUB_RATE = rospy.Duration(0.029) #35
img_pub_timer = None
image_list = []
publishedImagesHistory = []
SHUTDOWN_PUB = rospy.Publisher('/shutdown', Bool, queue_size=1)

print("Reading bag file")
# Reading bagfile from the path where bagfile is stored
bag = rosbag.Bag('/home/vivekw964/vivek_ws/src/flex_orbslam/scripts/MH_01_easy.bag')

BAG_READ = bag.read_messages(topics =['/cam0/image_raw'])
for topic, msg, time in BAG_READ:
    image_list.append(msg)
print("Number of image messages in ROS bag: ", len(image_list))
#After reading bagfile, pointer is set at the start of bagfile to point at first image in it
bag_pos = 0.0

def velCallback(callback_value):

    print ('Received velocity: ', callback_value.data)
    global velocity
    
    velocity = (callback_value.data)


def publish_image(event=None):

    global bag_pos, IMG_PUB, current_frame, published_messages, BAG_READ, SHUTDOWN_PUB, velocity, image_list
    
    #boundary condition to be checked before publishing the image
    bag_pos = bag_pos + velocity
    current_frame = int(bag_pos)
    if current_frame < len(image_list):

        current_time = rospy.Time.now()
        pub_frame_time = datetime.fromtimestamp(current_time.secs).strftime("%m/%d/%Y, %H:%M:%S")+"."+str(current_time.nsecs)  

        image_list[current_frame].header.stamp = current_time

        IMG_PUB.publish(image_list[current_frame])

        publishedImagesHistory.append({
            "frameID": int(image_list[current_frame].header.seq),
            "timestamp": pub_frame_time
        })
        print("Frame being published: ", current_frame)
        published_messages += 1
        print('Published messages till now: ', published_messages)
        print('--------------------------------')
    else:
        rospy.signal_shutdown("Stopping Publishing")


def save_values():
    global publishedImagesHistory
    publishedImagesHistory = sorted(publishedImagesHistory, key = lambda x:(datetime.strptime(x["timestamp"].split(".")[0], "%m/%d/%Y, %H:%M:%S"), int(x["timestamp"].split(".")[1])))
    with open('/home/vivekw964/vivek_ws/src/flex_orbslam/logs/imageHistoryEmulator.json', 'wb') as myfile:
        myfile.seek(0)
        json.dump(publishedImagesHistory, myfile, indent=4, sort_keys = True)

def mainf():
    global IMG_PUB_RATE, img_pub_timer
    print("Waiting for velocity...\n")
    rospy.init_node('emulator', anonymous=True)
    rospy.Subscriber("robot_velocity", Float32, velCallback)
    img_pub_timer = rospy.Timer(IMG_PUB_RATE, publish_image)
    rospy.spin()
    save_values()
    

if __name__ == '__main__':
    try:
        mainf()
    except rospy.ROSInterruptException:
        rospy.logwarn("There is something wrong with the program!")
