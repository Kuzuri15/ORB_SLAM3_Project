#!/usr/bin/env python2
import rospy
import rosbag
import time
import csv
import json
from std_msgs.msg import String, Float32, Int32, Bool
from datetime import datetime
from sensor_msgs.msg import Image
from collections import deque
from threading import Thread
from threading import Lock


BUFFER_LOCK = Lock()
BUFFER_SIZE = 1000
BUFFER_IMAGES = deque()

tracking_metric = 0
track_flag = True
img_flag = True
TRACKING_THRESHOLD = 50
count = 0
count2 = 0

DEFAULT_VEL = 2.0
BACKOFF_COEFF = 0.5
SPEEDUP_COEFF = 1.2  
SPEEDDOWN_COEFF = 0.8  
SLOWING_COEFF = 2 * (DEFAULT_VEL/BUFFER_SIZE) #0.004
BUFFER_ALLOWANCE = 20
imageCount = 0

pub_frame_time = ""
step = 0.0 
last_successful_frame = 0
next_frame = 0
prev_frame = 0
prev_tracking_metric = None

#publishers initiated for velocity and image
VEL_PUB = rospy.Publisher("robot_velocity", Float32, queue_size=1)
IMG_PUB = rospy.Publisher('/camera/image_raw', Image, queue_size = 1)
velocity = 0.0
bufferImageData = []

def imageCallback(img):
    global BUFFER_IMAGES, BUFFER_SIZE, imageCount, SLOWING_COEFF, DEFAULT_VEL, img_flag, BUFFER_LOCK
    
    BUFFER_LOCK.acquire()
    print("-----Image received from emulator:", img.header.seq)
    BUFFER_IMAGES.append(img)
    imageCount += 1
    if imageCount%100 == 0:
        print("100 frames passed, Images in buffer now: ", imageCount, len(BUFFER_IMAGES))
    
    #to publish first image so that tracking metric can be received and then publishing images from trackmetric callback.
    if img_flag:
        img_flag = False
        publish_image()
    BUFFER_LOCK.release()
    
    

def backTraversal():
    global next_frame, last_successful_frame
    print("---------going back in time")
    next_frame = 0
    last_successful_frame = 0


def trackMetricCallback(callback_val):
    global velocity, step, tracking_metric, last_successful_frame, pub_frame_time, prev_tracking_metric, BUFFER_IMAGES, track_flag, next_frame, prev_frame
    BUFFER_LOCK.acquire()
    prev_frame = next_frame
    print("-----ORBSLAM processed frame:", prev_frame)
    prev_tracking_metric = tracking_metric
    tracking_metric = callback_val.data

    current_time = rospy.Time.now()
    metric_recieved_time = datetime.fromtimestamp(current_time.secs).strftime("%m/%d/%Y, %H:%M:%S")+"."+str(current_time.nsecs)

    calculateStep()
    
    frame = int(BUFFER_IMAGES[next_frame].header.seq)
    length = len(BUFFER_IMAGES)
    publish_velocity()
    bufferImageData.append({
        "frameID": frame,
        "timestamp_before_image_processed": pub_frame_time,
        "timestamp_after_image_processed": metric_recieved_time,
        "stepSize" : step,
        "bufferLength": length,
        "tracking_metric": tracking_metric,
        "velocity" : velocity
    })
    publish_image()
    BUFFER_LOCK.release()
    
    
    
def calculateStep():
    global prev_tracking_metric, TRACKING_THRESHOLD, BUFFER_IMAGES, last_successful_frame, STEP_CONTROL, SPEEDDOWN_COEFF, SPEEDUP_COEFF, BACKOFF_COEFF, BUFFER_LOCK, step, tracking_metric, next_frame, count2, count, prev_frame

    if tracking_metric == -1:
        count += 1
        backTraversal()
        print("State value: ", tracking_metric)

    else:
        if tracking_metric >= TRACKING_THRESHOLD: #if tracking is going good as per metric
            #deleting before last sucessful step (published image)
            for i in range(last_successful_frame):
                BUFFER_IMAGES.popleft()
            prev_frame -= last_successful_frame
            last_successful_frame =  prev_frame

            #if tracking going better than previous published image, use speed up coeffieient to increase step size
            if tracking_metric >= prev_tracking_metric: 
                step = max(1.0, step*SPEEDUP_COEFF)
                print("+++Acceptable/better tracking than prev:", tracking_metric)
            #if tracking is going bad than the previous published image, use speed down coeffieient to decrease step size
            elif tracking_metric < prev_tracking_metric:
                step = max(1.0, step*SPEEDDOWN_COEFF)
                print("+++Acceptable/bad tracking than prev:", tracking_metric)
        else:
            step = max(1.0, step*BACKOFF_COEFF) #if tracking is going bad, we backoff the step to go slowly
            print("---Bad tracking:", tracking_metric)
        print("Updated step:", round(step,2))

        next_frame = prev_frame + int(round(step))
        #if next frame gets out of bound of buffer length, control the step by backing it off and publish last image from the buffer
        if next_frame >= len(BUFFER_IMAGES):
            next_frame = len(BUFFER_IMAGES)-1
            step = max(1.0, step*BACKOFF_COEFF)
            print("***Next frame is out of bounds, updating step to:", step)
        


def calculateVelocity():
    global velocity, SLOWING_COEFF, BUFFER_IMAGES, DEFAULT_VEL
    #updating the velocity as per the state of buffer
    velocity = max(0.6, DEFAULT_VEL - SLOWING_COEFF*(len(BUFFER_IMAGES) - BUFFER_ALLOWANCE)) #can clamp velocity at 1

def publish_image(event=None):
    global IMG_PUB, BUFFER_IMAGES, step, published_images, BUFFER_LOCK, pub_frame_time, next_frame
    if BUFFER_IMAGES and next_frame < len(BUFFER_IMAGES):
        
        current_time = rospy.Time.now()
        pub_frame_time = datetime.fromtimestamp(current_time.secs).strftime("%m/%d/%Y, %H:%M:%S")+"."+str(current_time.nsecs)
        IMG_PUB.publish(BUFFER_IMAGES[next_frame])
        print("Images in buffer: ", len(BUFFER_IMAGES))
        print("Frame published from buffer: ", next_frame)

    

def publish_velocity():
    global VEL_PUB, velocity
    calculateVelocity()
    # print("Velocity sent: ", velocity)
    VEL_PUB.publish(velocity)


#create json file to store values of tracking metric, buffer length, velocity and step size
def save_values():
    global bufferImageData
    bufferImageData = sorted(bufferImageData, key = lambda x:(datetime.strptime(x["timestamp_before_image_processed"].split(".")[0], "%m/%d/%Y, %H:%M:%S"), int(x["timestamp_before_image_processed"].split(".")[1])))
    with open('/home/vivekw964/vivek_ws/src/flex_orbslam/logs/BufferData.json', 'wb') as myfile:
        myfile.seek(0)
        json.dump(bufferImageData, myfile, indent=4)

def main_func():
    print("Starting...")
    global VEL_PUB, VEL_PUB_RATE, IMG_PUB_RATE, count, count2
    rospy.init_node('buffer', anonymous=True)
    rospy.Subscriber("/camera/images", Image, imageCallback)
    rospy.Subscriber("/tracking_metric", Int32, trackMetricCallback)
    
    rospy.spin()
    save_values()


if __name__ == '__main__':
    try:
        main_func()

    except rospy.ROSInterruptException:
        rospy.logwarn("There is something wrong with the program!")

