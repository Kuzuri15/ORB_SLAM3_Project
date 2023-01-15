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

DEFAULT_VEL = 2.0
BACKOFF_COEFF = 1.1
SPEEDUP_COEFF = 1.4    #1.2 previous
SPEEDDOWN_COEFF = 1.25   #temp value, experiment then change
SLOWING_COEFF = 2 * (DEFAULT_VEL/BUFFER_SIZE) #0.008
BUFFER_ALLOWANCE = 20
STEP_CONTROL = 0.6

pub_frame_time = ""
step = 0 #using as a buffer pointer
last_successful_step = 0
prev_tracking_metric = None

#publishers initiated for velocity and image
VEL_PUB = rospy.Publisher("robot_velocity", Float32, queue_size=1)
IMG_PUB = rospy.Publisher('/camera/image_raw', Image, queue_size = 1)

velocity = 0.0
bufferImageData = []

def imageCallback(img):
    global BUFFER_IMAGES, BUFFER_SIZE, imageCount, SLOWING_COEFF, DEFAULT_VEL, img_flag, BUFFER_LOCK
    # print("---------------img callback")
    # while BUFFER_LOCK.locked():
    #     continue
    BUFFER_LOCK.acquire()
    BUFFER_IMAGES.append(img)
    if len(BUFFER_IMAGES) > (BUFFER_SIZE-240)//2:  #if buffer goes beyond buffer size, resetting slowing coefficient to half the value before
        BUFFER_SIZE = 2*BUFFER_SIZE
        SLOWING_COEFF = 2 * (DEFAULT_VEL/BUFFER_SIZE)

    # imageCount += 1
    if len(BUFFER_IMAGES)%100 == 0:
        print("----------Images in buffer: ", len(BUFFER_IMAGES))
    BUFFER_LOCK.release()
    #to publish first image so that tracking metric can be received and then publishing images from trackmetric callback.
    if img_flag:
        img_flag = False
        publish_image()
    
    

def backTraversal():
    global step, last_successful_step
    print("---------going back in time")
    step = 0
    last_successful_step = 0


def trackMetricCallback(callback_val):
    global velocity, step, tracking_metric, last_successful_step, TRACKING_THRESHOLD, pub_frame_time
    global prev_tracking_metric, BUFFER_IMAGES, BACK_TRAVERSAL, track_flag
    # print("---------------metric callback")
   
    tracking_metric = callback_val.data
    # state = callback_val.data.mState
    print("Metric value: ", tracking_metric)
    current_time = rospy.Time.now()
    metric_recieved_time = datetime.fromtimestamp(current_time.secs).strftime("%m/%d/%Y, %H:%M:%S")+"."+str(current_time.nsecs)

    bufferImageData.append({
        "frameID": int(BUFFER_IMAGES[int(round(step))].header.seq),
        "timestamp_before_image_processed": pub_frame_time,
        "timestamp_after_image_processed": metric_recieved_time,
        "stepSize" : step,
        "bufferLength": len(BUFFER_IMAGES),
        "tracking_metric": tracking_metric,
        "velocity" : velocity
    })
    
    #if no tracking metric for first image
    # if not track_flag:
    step = calculateStep(tracking_metric, step)
    # else:
    #     step = 1
    #     track_flag = False
    prev_tracking_metric = tracking_metric
    publish_velocity()
    publish_image()
    
    
    
def calculateStep(tracking_metric, prev_step):
    global prev_tracking_metric, TRACKING_THRESHOLD, BUFFER_IMAGES, last_successful_step, STEP_CONTROL
    global SPEEDDOWN_COEFF, SPEEDUP_COEFF, BACKOFF_COEFF, BUFFER_LOCK
    # while BUFFER_LOCK.locked():
    #     continue
    BUFFER_LOCK.acquire()
    if tracking_metric == -1:
        backTraversal()
        print("State value: ", tracking_metric)
    
    #deleting before last sucessful step (published image)
    if tracking_metric >= TRACKING_THRESHOLD:
        print("---deleting")
        # print("step and lss:", int(round(step)), last_successful_step, len(BUFFER_IMAGES))
        for i in range(last_successful_step):
            BUFFER_IMAGES.popleft()
        prev_step -= last_successful_step
        last_successful_step =  int(round(prev_step))


    if tracking_metric >= TRACKING_THRESHOLD: #if tracking is going good as per metric
        #if tracking going better than previous published image, use speed up coeffieient to increase step size
        if tracking_metric >= prev_tracking_metric: 
            next_step = min(len(BUFFER_IMAGES)-1.0, max(prev_step+1.0, prev_step*SPEEDUP_COEFF))
            # print("better tracking than prev:", prev_step, next_step)
        #if tracking is going bad than the previous published image, use speed down coeffieient to decrease step size
        elif tracking_metric < prev_tracking_metric:
            # prevstep + 1 beacuse atleast 1 index should be increased from last step index
            next_step = min(len(BUFFER_IMAGES)-1.0, max(prev_step+1.0, prev_step*SPEEDDOWN_COEFF)) 
            # print("bad tracking than prev:", prev_step, next_step)
    else:
        # if last_successful_step == int(round(prev_step)):
        next_step = min(len(BUFFER_IMAGES)-1.0, max(prev_step+1.0, prev_step*BACKOFF_COEFF))  #if tracking is going bad, we backoff the step to go slowly
        # print("bad tracking:", prev_step, next_step)
    #when step exceeds kength of buffer, speed down to not exceed
    while int(round(next_step)) >= len(BUFFER_IMAGES):
    #int round() because if next_step becomes equal to buffer length after rounding in publish image function, will generate index out of range error.
        next_step = min(len(BUFFER_IMAGES)-1.0, next_step*STEP_CONTROL) 
    print("Step index: ", next_step)
    BUFFER_LOCK.release()
    return next_step

def calculateVelocity(BUFFER_IMAGES, DEFAULT_VEL):
    global velocity, SLOWING_COEFF
    # print("Slowing Coefficient:", SLOWING_COEFF)
    #updating the velocity as per the state of buffer
    velocity = max(1, DEFAULT_VEL - SLOWING_COEFF*(len(BUFFER_IMAGES) - BUFFER_ALLOWANCE)) #can clamp velocity at 1
    
    return velocity

def publish_image():
    global IMG_PUB, BUFFER_IMAGES, step, published_images, BUFFER_LOCK, pub_frame_time
    # print("buffer images, step", len(BUFFER_IMAGES), step)
    # while BUFFER_LOCK.locked():
    #     continue
    if BUFFER_IMAGES:
        BUFFER_LOCK.acquire()
        img_to_publish = int(round(step))
        print("publishing step:", img_to_publish)
        current_time = rospy.Time.now()
        pub_frame_time = datetime.fromtimestamp(current_time.secs).strftime("%m/%d/%Y, %H:%M:%S")+"."+str(current_time.nsecs)
        # pub_frame_time = datetime.fromtimestamp(BUFFER_IMAGES[img_to_publish].header.stamp.secs).strftime("%m/%d/%Y, %H:%M:%S")+"."+str(BUFFER_IMAGES[img_to_publish].header.stamp.nsecs) 
        IMG_PUB.publish(BUFFER_IMAGES[img_to_publish])
        print("Images in buffer: ", len(BUFFER_IMAGES))
        print("Frame published from buffer: ", img_to_publish)
        BUFFER_LOCK.release()
    

def publish_velocity():
    global VEL_PUB, velocity, DEFAULT_VEL, BUFFER_IMAGES
    try:
        velocity = calculateVelocity(BUFFER_IMAGES, DEFAULT_VEL)
        # print("Velocity sent: ", velocity)
        VEL_PUB.publish(velocity)
    except Exception:
        print("emultor stopped")


def shutdownCallback(callback_val):
    if callback_val:
        rospy.signal_shutdown("Stopping Publishing")


#create CSV files to store values of tracking metric, velocity and step size
def save_values():
    global bufferImageData
    bufferImageData = sorted(bufferImageData, key = lambda x:(datetime.strptime(x["timestamp_before_image_processed"].split(".")[0], "%m/%d/%Y, %H:%M:%S"), int(x["timestamp_before_image_processed"].split(".")[1])))
    with open('/home/vivekw964/vivek_ws/src/flex_orbslam/logs/BufferData.json', 'wb') as myfile:
        myfile.seek(0)
        json.dump(bufferImageData, myfile, indent=4)

def main_func():
    print("Starting...")
    global VEL_PUB, VEL_PUB_RATE, IMG_PUB_RATE
    rospy.init_node('buffer', anonymous=True)
    rospy.Subscriber("/tracking_metric", Int32, trackMetricCallback)
    rospy.Subscriber("/camera/images", Image, imageCallback, queue_size = 1)
    rospy.Subscriber("/shutdown", Bool, shutdownCallback)
    rospy.spin()
    save_values()


if __name__ == '__main__':
    try:
        main_func()

    except rospy.ROSInterruptException:
        rospy.logwarn("There is something wrong with the program!")

