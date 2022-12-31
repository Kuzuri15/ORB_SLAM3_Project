#!/usr/bin/env python2
import rospy
import rosbag
import time
import csv
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import Image

BUFFER_SIZE = 200
BUFFER_IMAGES = []
buffer_pointer = 0
step = 0
STEP_THRESHOLD = 20  #to avoid information loss which will lead to tracking loss
BACK_TRAVERSAL = 0.3 # During velocity 0 how much  % from current buffer pointer to go Back in Time
published_images = 0
wbit = False  #  boolean to show if went back in time

prev_pub_img = None
vel_0_same_img = False #when flag is true, need to go back in time to help the algorithm relocalize

#publishers initiated for velocity and image
VEL_PUB = rospy.Publisher("robot_velocity", Float32, queue_size=1)
IMG_PUB = rospy.Publisher('/camera/image_raw', Image, queue_size = 1)
# VEL_PUB_RATE = rospy.Duration(1.1) # 10 Hz
# IMG_PUB_RATE = rospy.Duration(0.8) # 10 Hz


VEL_PUB_RATE = rospy.Duration(0.8) # 10 Hz
IMG_PUB_RATE = rospy.Duration(0.6) # 10 Hz


MAX_VELOCITY = 4.0
LEAST_VELOCITY = 0.0
TRACK_LOST_THRESHOLD = 20 #maximum continuous track loss after which we need to set velocity = 0 
track_lost = 0
velocity = 0.0
velocity_history = []

def imageCallback(img):
    global prev_pub_img, BUFFER_IMAGES, vel_0_same_img, BUFFER_SIZE, buffer_pointer, wbit
    start_time = time.time()

    #if same image received from emulator as the previous image published to ORBslam then we can say
    #that velocity is 0 and need to go back in time
    if prev_pub_img and prev_pub_img.data != img.data or not prev_pub_img:
        vel_0_same_img = False
        step = 0
    else:
        vel_0_same_img = True
        # wbit = False

    #if velocity not 0 that means different image receoved and it can be appended to the buffer
    if not vel_0_same_img:
        if BUFFER_IMAGES and len(BUFFER_IMAGES) >= BUFFER_SIZE:
            BUFFER_IMAGES.pop(0)
            buffer_pointer -= 1
        BUFFER_IMAGES.append(img)
    if len(BUFFER_IMAGES) > BUFFER_SIZE:
        buffer_pointer = len(BUFFER_IMAGES) - 1

    print("Images in buffer: ", len(BUFFER_IMAGES))
    print 'Processing time per loop:', time.time() - start_time



def trackLostCallback(callback_val):
    global track_lost, velocity, MAX_VELOCITY, LEAST_VELOCITY, TRACK_LOST_THRESHOLD, step
    tracking_metric = callback_val.data
    print("Flag value in buffer:", tracking_metric)

    if tracking_metric == False:  #tracking is going good
        track_lost -= 2           #decrease the amount of deacceleartion when tracking is going good
        if vel_0_same_img:        #when velocity is 0 and tracking is going good, increasing the step parameter
            step += 2             
        if step > STEP_THRESHOLD: #boundary condition check
            step = STEP_THRESHOLD
        if track_lost < 0:        #boundary condition check
            track_lost = 0          
    elif tracking_metric == True:  #if tracking is lost
        if vel_0_same_img:         #when velocity is 0 and tracking is going bad, decreasing the step parameter
            step -= 1
        if step < 0:               #boundary condition check
            step = 0
        track_lost += 1
        if track_lost >= TRACK_LOST_THRESHOLD: #boundary condition check
            track_lost = TRACK_LOST_THRESHOLD
    #updating the velocity as per the continuous track lost
    velocity = (1 - (float(track_lost) / TRACK_LOST_THRESHOLD)) * MAX_VELOCITY
    print(velocity, TRACK_LOST_THRESHOLD, MAX_VELOCITY, track_lost)
    velocity_history.append(velocity)


def publish_image(event=None):
    global IMG_PUB, BUFFER_IMAGES, buffer_pointer, published_images, vel_0_same_img, prev_pub_img, wbit, step
    # buffer_pointer = len(BUFFER_IMAGES) - 1
    # print("Publishing image")
    # print(buffer_pointer, len(BUFFER_IMAGES))
    # IMG_PUB.publish(BUFFER_IMAGES[buffer_pointer])
    # published_images += 1
    # print("Images published from buffer: ", published_images)
    #before publishing, check if buffer is empty or not
    if BUFFER_IMAGES:
        print("Publishing image")
        print(buffer_pointer, len(BUFFER_IMAGES), step, wbit, vel_0_same_img)

        # if not wbit and vel_0_same_img:
            # wbit = True
        if vel_0_same_img:  #going back in time when velocity is 0
            buffer_pointer -= int(BACK_TRAVERSAL * buffer_pointer)
        #condition to use step parameter after we go back in time until buffer pointer reaches new images in the buffer
        # if not vel_0_same_img:
        #     buffer_pointer -= step
        if vel_0_same_img:
            buffer_pointer += step
        if buffer_pointer >= len(BUFFER_IMAGES): #boundary condition
            buffer_pointer = len(BUFFER_IMAGES) - 1
        if buffer_pointer < 0:  #boundary condition
            buffer_pointer = 0
        IMG_PUB.publish(BUFFER_IMAGES[buffer_pointer])
        prev_pub_img = BUFFER_IMAGES[buffer_pointer]
        published_images += 1
        print("Images published from buffer: ", published_images)
        buffer_pointer += 1
        # if vel_0_same_img:
        #     buffer_pointer = 0
        #when buffer pointer reaches end of the buffer and velocity is non zero, we keep on publishing the last image
        if buffer_pointer >= len(BUFFER_IMAGES) and not vel_0_same_img:
            buffer_pointer = len(BUFFER_IMAGES) - 1
        if buffer_pointer >= len(BUFFER_IMAGES): #boundary condition
            buffer_pointer = len(BUFFER_IMAGES) - 1
        if buffer_pointer < 0:  #boundary condition
            buffer_pointer = 0

def publish_velocity(event=None):
    global VEL_PUB, velocity
    VEL_PUB.publish(velocity)

def shutdownCallback(callback_val):
    if callback_val:
        rospy.signal_shutdown("Stopping Publishing")



def main_func():
    print("Starting...")
    global VEL_PUB, velocity_history, VEL_PUB_RATE, IMG_PUB_RATE
    rospy.init_node('buffer', anonymous=True)
    # rate = rospy.Rate(10)
    rospy.Subscriber("/camera/images", Image, imageCallback, queue_size = 1)
    rospy.Subscriber("/track_loss", Bool, trackLostCallback)
    rospy.Subscriber("/shutdown", Bool, shutdownCallback)

    # Publishing image to ORBslam
    img_pub_timer = rospy.Timer(IMG_PUB_RATE, publish_image)
    # Publishing velocity to emulator
    vel_pub_timer = rospy.Timer(VEL_PUB_RATE, publish_velocity)
    rospy.spin()
    
    #to get velocity values in CSV file to plot the graph
    with open('velocity_values_MH04.csv', 'wb') as myfile:
        write = csv.writer(myfile)
        for w in velocity_history:
            write.writerow([w])


if __name__ == '__main__':
    try:
        main_func()

    except rospy.ROSInterruptException:
        pass

