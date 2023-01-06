#!/usr/bin/env python2
import rospy
import rosbag
import time
import csv
from std_msgs.msg import String, Float32, Int32, Bool
from sensor_msgs.msg import Image

BUFFER_SIZE = 200
BUFFER_IMAGES = []
buffer_pointer = 0


tracking_metric = 0
track_flag = True
last_frame = 0
next_frame = 0

DEFAULT_VEL = 3.0
BACKOFF_COEFF = 0.5     #temp value, experiment then change
SPEEDUP_COEFF = 1.3     #temp value, experiment then change
SPEEDDOWN_COEFF = 0.9   #temp value, experiment then change
SLOWING_COEFF = 2 * 0.005

step = 1
MAX_STEP = 5  #to avoid information loss which will lead to tracking loss
MIN_STEP = 1
BACK_TRAVERSAL = 0.3 # During velocity 0 how much  % from current buffer pointer to go Back in Time

GBIT = False  #  boolean to show if went back in time

published_images = 0
prev_pub_img = None
prev_tracking_metric = None
same_image = False #when flag is true, need to go back in time to help the algorithm relocalize

#publishers initiated for velocity and image
VEL_PUB = rospy.Publisher("robot_velocity", Float32, queue_size=1)
IMG_PUB = rospy.Publisher('/camera/image_raw', Image, queue_size = 1)
VEL_PUB_RATE = rospy.Duration(0.6) # 10 Hz
IMG_PUB_RATE = rospy.Duration(0.8) # 10 Hz

TRACK_LOST_THRESHOLD = 5 #maximum continuous track loss after which we need to set velocity = 0 
track_lost = 0

velocity = 0.0
velocity_history = []

def imageCallback(img):
    global prev_pub_img, BUFFER_IMAGES, same_img, BUFFER_SIZE, buffer_pointer
    start_time = time.time()

    #if same image received from emulator as the previous image published to ORBslam then we can say
    #that velocity is 0 and need to go back in time
    if prev_pub_img and prev_pub_img.data != img.data or not prev_pub_img:
        same_img = False
    else:
        same_img = True

    #if velocity not 0 that means different image received and it can be appended to the buffer
    if not same_img:
        if BUFFER_IMAGES and len(BUFFER_IMAGES) >= BUFFER_SIZE:
            BUFFER_IMAGES.pop(0)
            buffer_pointer -= 1
        BUFFER_IMAGES.append(img)

    print("Images in buffer: ", len(BUFFER_IMAGES))
    # print 'Processing time per loop:', time.time() - start_time



def trackMetricCallback(callback_val):
    global track_lost, velocity, MAX_VELOCITY, LEAST_VELOCITY, step, tracking_metric, GBIT, TRACK_LOST_THRESHOLD
    tracking_metric = callback_val.data
    print("Metric value: ", tracking_metric)

    #after continuous track loss of more than 5 published frames, we go back in time to relocalize
    if tracking_metric <= 30:
        track_lost+=1
        if track_lost >= TRACK_LOST_THRESHOLD: #boundary condition check
            track_lost = 0
            GBIT = True
    else:
        track_lost-=1
        GBIT = False

def calculateStep(tracking_metric, prev_step):
    global prev_tracking_metric, last_frame, BUFFER_SIZE, MIN_STEP, MAX_STEP
    global SPEEDDOWN_COEFF, SPEEDUP_COEFF, BACKOFF_COEFF
    
    if tracking_metric >= 50: #if tracking is going good as per metric
        #if tracking going better than previous published image
        if tracking_metric >= prev_tracking_metric: next_step = min(MAX_STEP,max(MIN_STEP, prev_step*SPEEDUP_COEFF))
        #if tracking is going bad than the previous published image
        elif tracking_metric < prev_tracking_metric: next_step = min(MAX_STEP,max(MIN_STEP, prev_step*SPEEDDOWN_COEFF))
    else: next_step = min(MAX_STEP, max(MIN_STEP, prev_step*BACKOFF_COEFF))  #if tracking is going bad, we backoff the step
    return next_step


def publish_image(event=None):
    global IMG_PUB, BUFFER_IMAGES, buffer_pointer, published_images, same_image, prev_pub_img, step, last_frame, GBIT
    global tracking_metric, track_flag, prev_tracking_metric, BACK_TRAVERSAL

    #before publishing, check if buffer is empty or not
    if BUFFER_IMAGES:
        print("Publishing image")
        #when tracking is good, flushing the buffer but keeping atleast 20 frames in buffer if
        #required to go back in time/buffer
        if tracking_metric >= 50 and last_frame>20:
            print("Buffer pointer n last frame: ", buffer_pointer, last_frame)
            del BUFFER_IMAGES[:last_frame-20]
            buffer_pointer -= (last_frame-20)

        elif GBIT:  #when we go back in time, buffer pointer should go back by 30% (back traversal)
            print("-------------going back: ")
            buffer_pointer -= int(BACK_TRAVERSAL * buffer_pointer)
            step = 1  #resetting the step
        
        last_frame = buffer_pointer
        print("last frame at start: ", last_frame)
        if not track_flag:
            next_step = calculateStep(tracking_metric, step)
            last_frame = min(BUFFER_SIZE-1, last_frame + int(round(next_step)))
        else:
            next_step = 1
            track_flag = False

        step = next_step
        print("Step size: ", step)

        if buffer_pointer >= len(BUFFER_IMAGES): #boundary condition
            buffer_pointer = len(BUFFER_IMAGES) - 1
        if buffer_pointer < 0:  #boundary condition
            buffer_pointer = 0
        if last_frame >= len(BUFFER_IMAGES): last_frame = len(BUFFER_IMAGES)-1

        IMG_PUB.publish(BUFFER_IMAGES[last_frame])
        # rospy.loginfo("Buffer\n")

        prev_pub_img = BUFFER_IMAGES[last_frame]
        prev_tracking_metric = tracking_metric
        
        published_images += 1
        print("Images published from buffer: ", last_frame)
        buffer_pointer = last_frame + 1

        #when buffer pointer reaches end of the buffer and velocity is non zero, we keep on publishing the last image
        if buffer_pointer >= len(BUFFER_IMAGES) and not same_image:
            buffer_pointer = len(BUFFER_IMAGES) - 1
        if buffer_pointer >= len(BUFFER_IMAGES): #boundary condition
            buffer_pointer = len(BUFFER_IMAGES) - 1
        if buffer_pointer < 0:  #boundary condition
            buffer_pointer = 0

     


def publish_velocity(event=None):
    global VEL_PUB, velocity, BUFFER_IMAGES, DEFAULT_VEL, BUFFER_SIZE, SLOWING_COEFF, velocity_history
    print("slowing coeff:", SLOWING_COEFF)
    #updating the velocity as per the continuous track lost and state of buffer
    velocity = max(0, (DEFAULT_VEL - SLOWING_COEFF*len(BUFFER_IMAGES)))
    print("Velocity sent: ", velocity)
    velocity_history.append(velocity)
    VEL_PUB.publish(velocity)
    

def shutdownCallback(callback_val):
    if callback_val:
        rospy.signal_shutdown("Stopping Publishing")


def main_func():
    print("Starting...")
    global VEL_PUB, velocity_history, VEL_PUB_RATE, IMG_PUB_RATE
    rospy.init_node('buffer', anonymous=True)
    rospy.Subscriber("/camera/images", Image, imageCallback, queue_size = 1)
    rospy.Subscriber("/tracking_metric", Int32, trackMetricCallback)
    rospy.Subscriber("/shutdown", Bool, shutdownCallback)

    # Publishing image to ORBslam
    img_pub_timer = rospy.Timer(IMG_PUB_RATE, publish_image)
    # Publishing velocity to emulator
    vel_pub_timer = rospy.Timer(VEL_PUB_RATE, publish_velocity)
    rospy.spin()

    with open('velocity_values_MH04.csv', 'wb') as myfile:
        write = csv.writer(myfile)
        for w in velocity_history:
            write.writerow([w])



if __name__ == '__main__':
    try:
        main_func()

    except rospy.ROSInterruptException:
        pass

