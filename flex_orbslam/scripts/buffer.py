#!/usr/bin/env python2
import rospy
import rosbag
import time
import csv
import json
from std_msgs.msg import String, Float32, Int32, Bool
# from ORB_SLAM3.msg import TrackData
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

DEFAULT_VEL = 4.0
BACKOFF_COEFF = 1.1
SPEEDUP_COEFF = 1.4    #1.2 previous
SPEEDDOWN_COEFF = 1.25   #temp value, experiment then change
SLOWING_COEFF = 2 * (DEFAULT_VEL/BUFFER_SIZE) #0.008
BUFFER_ALLOWANCE = 5


step = 0 #using as a buffer pointer
last_successful_step = 0
published_images = 0
prev_tracking_metric = None

#publishers initiated for velocity and image
VEL_PUB = rospy.Publisher("robot_velocity", Float32, queue_size=1)
IMG_PUB = rospy.Publisher('/camera/image_raw', Image, queue_size = 1)
# VEL_PUB_RATE = rospy.Duration(0.2) # 10 Hz
IMG_PUB_RATE = rospy.Duration(0.1) # 10 Hz


velocity = 0.0
velocity_history = []
metric_history = []
stepsize_history = []
bufferLength_history = []
publishedImagesHistory = []



def imageCallback(img):
    global BUFFER_IMAGES, BUFFER_SIZE, imageCount, SLOWING_COEFF, DEFAULT_VEL
    # print("-----------------------")
    # print(img.header.stamp, img.header.seq, img.header.frame_id)
    # if not BUFFER_LOCK.locked():
    while BUFFER_LOCK.locked():
        continue
    BUFFER_LOCK.acquire()
    BUFFER_IMAGES.append(img)
    if len(BUFFER_IMAGES) > (BUFFER_SIZE-240)//2:  #if buffer goes beyond buffer size, resetting slowing coefficient to half the value before
        BUFFER_SIZE = 2*BUFFER_SIZE
        SLOWING_COEFF = 2 * (DEFAULT_VEL/BUFFER_SIZE)

    # imageCount += 1
    if len(BUFFER_IMAGES)%100 == 0:
        print("----------Images in buffer: ", len(BUFFER_IMAGES))
    BUFFER_LOCK.release()
    

def backTraversal():
    global step, last_successful_step
    print("---------going back in time")
    step = 0
    last_successful_step = 0


def trackMetricCallback(callback_val):
    global track_lost, velocity, step, tracking_metric, TRACK_LOST_THRESHOLD, last_successful_step
    global prev_tracking_metric, BUFFER_IMAGES, BACK_TRAVERSAL, track_flag
    while BUFFER_LOCK.locked():
        continue
    BUFFER_LOCK.acquire()
    tracking_metric = callback_val.data
    # state = callback_val.data.mState
    print("Metric value: ", tracking_metric)
    metric_history.append({
        "tracking_metric": tracking_metric,
        "timestamp": datetime.now().strftime("%m/%d/%Y, %H:%M:%S")
    })


    if tracking_metric == -1:
        backTraversal()
        print("State value: ", tracking_metric)
    
    #deleting before last sucessful step (published image)
    if tracking_metric >= 50:
        print("---deleting")
        # print("step and lss:", int(round(step)), last_successful_step, len(BUFFER_IMAGES))
        for i in range(last_successful_step):
            BUFFER_IMAGES.popleft()
        step -= last_successful_step
        last_successful_step =  int(round(step))

    #if no tracking metric for first image
    if not track_flag:
        step = calculateStep(tracking_metric, step)
    else:
        step = 1
        track_flag = False
    print("Step size: ", step)

    prev_tracking_metric = tracking_metric

    if int(round(step)) >= len(BUFFER_IMAGES): #boundary condition
        step = len(BUFFER_IMAGES) - 1
    if int(round(step)) < 0:  #boundary condition
        step = 0
        
    BUFFER_LOCK.release()
    publish_velocity()


def calculateStep(tracking_metric, prev_step):
    global prev_tracking_metric, BUFFER_SIZE, MIN_STEP, MAX_STEP
    global SPEEDDOWN_COEFF, SPEEDUP_COEFF, BACKOFF_COEFF
    
    if tracking_metric >= 50: #if tracking is going good as per metric
        #if tracking going better than previous published image, use speed up coeffieient to increase step size
        if tracking_metric >= prev_tracking_metric: 
            next_step = min(len(BUFFER_IMAGES)-1, max(prev_step+1, prev_step*SPEEDUP_COEFF))
            print("better tracking than prev:", prev_step, next_step)
        #if tracking is going bad than the previous published image, use speed down coeffieient to decrease step size
        elif tracking_metric < prev_tracking_metric:
            # prevstep + 1 beacuse atleast 1 index should be increased from last step index
            next_step = min(len(BUFFER_IMAGES)-1, max(prev_step+1, prev_step*SPEEDDOWN_COEFF)) 
            print("bad tracking than prev:", prev_step, next_step)
    else:
        # if last_successful_step == int(round(prev_step)):
        next_step = min(len(BUFFER_IMAGES)-1, max(prev_step+1, prev_step*BACKOFF_COEFF))  #if tracking is going bad, we backoff the step to go slowly
        print("bad tracking:", prev_step, next_step)
    return next_step

def calculateVelocity(BUFFER_IMAGES, DEFAULT_VEL):
    global velocity, SLOWING_COEFF
    print("Slowing Coefficient:", SLOWING_COEFF)
    #updating the velocity as per the state of buffer
    velocity = max(0, min(DEFAULT_VEL, DEFAULT_VEL - SLOWING_COEFF*(len(BUFFER_IMAGES) - BUFFER_ALLOWANCE)))
    
    return velocity

def publish_image(event=None):
    global IMG_PUB, BUFFER_IMAGES, step, bufferLength_history, publishedImagesHistory
    # print("buffer images, step", len(BUFFER_IMAGES), step)
    while BUFFER_LOCK.locked():
        continue
    if BUFFER_IMAGES:
        BUFFER_LOCK.acquire()
        # print("publishing step:", int(round(step)))
        IMG_PUB.publish(BUFFER_IMAGES[int(round(step))])
        publishedImagesHistory.append({
            "frameID": str(BUFFER_IMAGES[int(round(step))].header.seq),
            "timestamp":  str(BUFFER_IMAGES[int(round(step))].header.stamp.secs)+"."+str(BUFFER_IMAGES[int(round(step))].header.stamp.nsecs)
        })
        print("Images in buffer: ", len(BUFFER_IMAGES))
        print("Frame published from buffer: ", int(round(step)))
        bufferLength_history.append(len(BUFFER_IMAGES))
        stepsize_history.append(step)
        BUFFER_LOCK.release()
    

def publish_velocity():
    global VEL_PUB, velocity, DEFAULT_VEL, velocity_history, BUFFER_IMAGES
    velocity = calculateVelocity(BUFFER_IMAGES, DEFAULT_VEL)
    print("Velocity sent: ", velocity)
    velocity_history.append(velocity)
    VEL_PUB.publish(velocity)

def shutdownCallback(callback_val):
    if callback_val:
        rospy.signal_shutdown("Stopping Publishing")


#create CSV files to store values of tracking metric, velocity and step size
def save_values():
    global metric_history, velocity_history, stepsize_history, bufferLength_history, publishedImagesHistory
    with open('velocity_values.csv', 'wb') as myfile:
        write = csv.writer(myfile)
        for w in velocity_history:
            write.writerow([w])
    with open('stepsize_values.csv', 'wb') as myfile:
        write = csv.writer(myfile)
        for w in stepsize_history:
            write.writerow([w])
    with open('bufferlength_values.csv', 'wb') as myfile:
        write = csv.writer(myfile)
        for w in bufferLength_history:
            write.writerow([w])
    with open('metric_values.json', 'wb') as myfile:
        myfile.seek(0)
        json.dump(metric_history, myfile, indent=4)
    with open('imageHistory.json', 'wb') as myfile:
        myfile.seek(0)
        json.dump(publishedImagesHistory, myfile, indent=4)

def main_func():
    print("Starting...")
    global VEL_PUB, velocity_history, VEL_PUB_RATE, IMG_PUB_RATE
    rospy.init_node('buffer', anonymous=True)
    rospy.Subscriber("/camera/images", Image, imageCallback, queue_size = 1)
    rospy.Subscriber("/tracking_metric", Int32, trackMetricCallback)
    rospy.Subscriber("/shutdown", Bool, shutdownCallback)

    # Publishing image to ORBslam
    img_pub_timer = rospy.Timer(IMG_PUB_RATE, publish_image)
    # # vel_pub_timer = rospy.Timer(VEL_PUB_RATE, publish_velocity)
    rospy.spin()

    save_values()


if __name__ == '__main__':
    try:
        main_func()

    except rospy.ROSInterruptException:
        rospy.logwarn("There is something wrong with the program!")

