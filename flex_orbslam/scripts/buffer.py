#!/usr/bin/env python2
import rospy
import rosbag
import time
import csv
from std_msgs.msg import String, Float32, Int32, Bool
from sensor_msgs.msg import Image
from collections import deque
from threading import Thread
from threading import Lock

BUFFER_LOCK = Lock()

BUFFER_SIZE = 1000
# BUFFER_IMAGES = []
imageCount = 0
# buffer_pointer = 0
last_frame_pointer = 0

MUTEX = False
BUFFER_IMAGES = deque()

tracking_metric = 0
track_flag = True
last_frame = 0
next_frame = 0

DEFAULT_VEL = 4.0
BACKOFF_COEFF = 0.5     #temp value, experiment then change
SPEEDUP_COEFF = 1.1    #1.2 previous
SPEEDDOWN_COEFF = 0.9   #temp value, experiment then change
SLOWING_COEFF = 2 * (DEFAULT_VEL/BUFFER_SIZE) #0.008
# SLOWING_COEFF = 0.025


step = 1
MAX_STEP = 5  #to avoid information loss which will lead to tracking loss
MIN_STEP = 1
BACK_TRAVERSAL = 0.3 # During velocity 0 how much  % from current buffer pointer to go Back in Time

GBIT = False  #  boolean to show if went back in time

published_images = 0
prev_pub_img = None
prev_tracking_metric = None
same_image = False #when flag is true, need to go back in time to help the robot relocalize

#publishers initiated for velocity and image
VEL_PUB = rospy.Publisher("robot_velocity", Float32, queue_size=1)
IMG_PUB = rospy.Publisher('/camera/image_raw', Image, queue_size = 1)
# VEL_PUB_RATE = rospy.Duration(0.2) # 10 Hz
IMG_PUB_RATE = rospy.Duration(0.2) # 10 Hz

TRACK_LOST_THRESHOLD = 5 #maximum continuous track loss after which we need to set velocity = 0 
track_lost = 0

velocity = 0.0
velocity_history = []
metric_history = []
stepsize_history = []
bufferLength_history = []


def imageCallback(img):
    global prev_pub_img, BUFFER_IMAGES, same_image, BUFFER_SIZE, imageCount

    if not BUFFER_LOCK.locked():
        BUFFER_LOCK.acquire()
        BUFFER_IMAGES.append(img)
        publish_velocity()
        BUFFER_LOCK.release()
    imageCount += 1
    if imageCount%100 == 0:
        print("----------Images in buffer: ", len(BUFFER_IMAGES))


def trackMetricCallback(callback_val):
    global track_lost, velocity, step, tracking_metric, GBIT, TRACK_LOST_THRESHOLD
    global prev_tracking_metric, last_frame_pointer, BUFFER_IMAGES, BACK_TRAVERSAL, track_flag
    tracking_metric = callback_val.data
    print("Metric value: ", tracking_metric)
    metric_history.append(tracking_metric)

    #after continuous track loss of more than 5 published frames, we go back in time to relocalize
    if tracking_metric <= 30:
        track_lost+=1
        print("track_lost", track_lost)
        if track_lost >= TRACK_LOST_THRESHOLD: #boundary condition check
            track_lost = 0
            GBIT = True
    else:
        # track_lost-=1
        GBIT = False

    if tracking_metric >= 50 and last_frame_pointer>400:
  
        if not BUFFER_LOCK.locked():
            BUFFER_LOCK.acquire()
            for i in range(last_frame_pointer-20):
                BUFFER_IMAGES.popleft()
            last_frame_pointer -= (last_frame_pointer-20)
            BUFFER_LOCK.release()


    elif GBIT:  #when we go back in time, buffer pointer should go back by 30% (back traversal)
        print("-------------going back: ")
        last_frame_pointer -= int(BACK_TRAVERSAL * last_frame_pointer)
        step = 1  #resetting the step

    #if no tracking metric for first image
    if not track_flag:
        next_step = calculateStep(tracking_metric, step)
        last_frame_pointer = min(len(BUFFER_IMAGES)-1, last_frame_pointer + int(round(next_step)))  #BUFFER_SIZE -> BUFFER_IMAGES length
    else:
        next_step = 1
        track_flag = False
    step = next_step
    stepsize_history.append(step)
    print("Step size: ", step)

    prev_tracking_metric = tracking_metric
    last_frame_pointer = last_frame_pointer + 1 #as last frame has been published, buffer pointer should come on next frame

    if last_frame_pointer >= len(BUFFER_IMAGES): #boundary condition
        last_frame_pointer = len(BUFFER_IMAGES) - 1
    if last_frame_pointer < 0:  #boundary condition
        last_frame_pointer = 0
    if last_frame_pointer >= len(BUFFER_IMAGES): last_frame_pointer = len(BUFFER_IMAGES)-1


def calculateStep(tracking_metric, prev_step):
    global prev_tracking_metric, last_frame_pointer, BUFFER_SIZE, MIN_STEP, MAX_STEP
    global SPEEDDOWN_COEFF, SPEEDUP_COEFF, BACKOFF_COEFF
    
    if tracking_metric >= 50: #if tracking is going good as per metric
        #if tracking going better than previous published image, use speed up coeffieient to increase step size
        if tracking_metric >= prev_tracking_metric: next_step = min(MAX_STEP,max(MIN_STEP, prev_step*SPEEDUP_COEFF))
        #if tracking is going bad than the previous published image, use speed down coeffieient to decrease step size
        elif tracking_metric < prev_tracking_metric: next_step = min(MAX_STEP,max(MIN_STEP, prev_step*SPEEDDOWN_COEFF))
    else: next_step = min(MAX_STEP, max(MIN_STEP, prev_step*BACKOFF_COEFF))  #if tracking is going bad, we backoff the step
    return next_step

def calculateVelocity(BUFFER_IMAGES, DEFAULT_VEL):
    global velocity, SLOWING_COEFF
    print("Slowing Coefficient:", SLOWING_COEFF)
    #updating the velocity as per the state of buffer
    velocity = max(0, (DEFAULT_VEL - SLOWING_COEFF*len(BUFFER_IMAGES)))
    
    return velocity

def publish_image(event=None):
    global IMG_PUB, BUFFER_IMAGES, last_frame_pointer, last_frame_pointer, bufferLength_history
    if BUFFER_IMAGES:

        IMG_PUB.publish(BUFFER_IMAGES[last_frame_pointer])

        print("Images in buffer: ", len(BUFFER_IMAGES))
        print("Frame published from buffer: ", last_frame_pointer)
        bufferLength_history.append(len(BUFFER_IMAGES))
    

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
    global metric_history, velocity_history, stepsize_history, bufferLength_history
    with open('velocity_values.csv', 'wb') as myfile:
        write = csv.writer(myfile)
        for w in velocity_history:
            write.writerow([w])
    with open('metric_values.csv', 'wb') as myfile:
        write = csv.writer(myfile)
        for w in metric_history:
            write.writerow([w])
    with open('stepsize_values.csv', 'wb') as myfile:
        write = csv.writer(myfile)
        for w in stepsize_history:
            write.writerow([w])
    with open('bufferlength_values.csv', 'wb') as myfile:
        write = csv.writer(myfile)
        for w in bufferLength_history:
            write.writerow([w])

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
    # rate = rospy.Rate(5)
    # while not rospy.is_shutdown():
    #     # img_pub_timer = rospy.Timer(IMG_PUB_RATE, publish_image)
    #     publish_velocity()
    #     rate.sleep()

    save_values()


if __name__ == '__main__':
    try:
        main_func()

    except rospy.ROSInterruptException:
        rospy.logwarn("There is something wrong with the program!")

