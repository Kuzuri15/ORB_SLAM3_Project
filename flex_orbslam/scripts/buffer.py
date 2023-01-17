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
SPEEDUP_COEFF = 1.1   #1.2 previous
SPEEDDOWN_COEFF = 1.05   #temp value, experiment then change
SLOWING_COEFF = 2 * (DEFAULT_VEL/BUFFER_SIZE) #0.008
BUFFER_ALLOWANCE = 20
imageCount = 0

pub_frame_time = ""
step = 0.0 #using as a buffer pointer
last_successful_frame = 0
next_frame = 0
prev_tracking_metric = None

#publishers initiated for velocity and image
VEL_PUB = rospy.Publisher("robot_velocity", Float32, queue_size=1)
IMG_PUB = rospy.Publisher('/camera/image_raw', Image, queue_size = 1)
# IMG_PUB_RATE = rospy.Duration(0.05)
velocity = 0.0
bufferImageData = []

def imageCallback(img):
    global BUFFER_IMAGES, BUFFER_SIZE, imageCount, SLOWING_COEFF, DEFAULT_VEL, img_flag, BUFFER_LOCK
    
    # while BUFFER_LOCK.locked():
    #     continue
    BUFFER_LOCK.acquire()
    print("---------------img callback")
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
  
    # time.sleep(10)
    print("---------going back in time")
    next_frame = 0
    last_successful_frame = 0


def trackMetricCallback(callback_val):
    global velocity, step, tracking_metric, last_successful_frame, pub_frame_time, prev_tracking_metric, BUFFER_IMAGES, track_flag, next_frame
    BUFFER_LOCK.acquire()
    # print("---------------metric callback")
    tracking_metric = callback_val.data
    prev_tracking_metric = tracking_metric
    # state = callback_val.data.mState
    print("Metric value: ", tracking_metric)
    current_time = rospy.Time.now()
    metric_recieved_time = datetime.fromtimestamp(current_time.secs).strftime("%m/%d/%Y, %H:%M:%S")+"."+str(current_time.nsecs)
    # while BUFFER_LOCK.locked():
    #     continue
    #if no tracking metric for first image
    
    calculateStep()
    # if tracking_metric == -1:
    #     if track_flag:
    #         next_frame = min(len(BUFFER_IMAGES)-1, next_frame+1)
    # else:
    #     track_flag = False
    # #     next_frame = 0
    # #     track_flag = False
    
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
    global prev_tracking_metric, TRACKING_THRESHOLD, BUFFER_IMAGES, last_successful_frame, STEP_CONTROL, SPEEDDOWN_COEFF, SPEEDUP_COEFF, BACKOFF_COEFF, BUFFER_LOCK, step, tracking_metric, next_frame, count2, count
    # while BUFFER_LOCK.locked():
    #     continue
    # BUFFER_LOCK.acquire()
    current_frame = next_frame
    if tracking_metric == -1:
        count += 1
        backTraversal()
        print("State value: ", tracking_metric)

    else:
        print("--------step", current_frame, round(step,2), last_successful_frame)
        if tracking_metric >= TRACKING_THRESHOLD: #if tracking is going good as per metric
            #deleting before last sucessful step (published image)
            print("---deleting")
            # print("step and lss:", int(round(step)), last_successful_frame, len(BUFFER_IMAGES))
            for i in range(last_successful_frame):
                BUFFER_IMAGES.popleft()
            current_frame -= last_successful_frame
            last_successful_frame =  current_frame

            #if tracking going better than previous published image, use speed up coeffieient to increase step size
            if tracking_metric >= prev_tracking_metric: 
                step = max(1.0, step*SPEEDUP_COEFF)
                print("better tracking than prev:", round(step,2))
            #if tracking is going bad than the previous published image, use speed down coeffieient to decrease step size
            elif tracking_metric < prev_tracking_metric:
                # prevstep + 1 beacuse atleast 1 index should be increased from last step index
                step = max(1.0, step*SPEEDDOWN_COEFF)
                print("bad tracking than prev:", round(step,2))
        else:
            # if last_successful_frame == int(round(step)):
            step = max(1.0, step*BACKOFF_COEFF) #if tracking is going bad, we backoff the step to go slowly
            print("bad tracking:", round(step,2))
        #when step exceeds kength of buffer, speed down to not exceed
        next_frame = current_frame
        current_frame = current_frame + int(round(step))
        # print("cur n  next frm:", current_frame, next_frame)
    #if the last published frame already points to the end of the buffer, then it doesn't make sense to update the next_frame with the step as minimum values of step 
    #is 1 which will cause an infinite while loop. So, decided to publish the last frame in the buffer again (occured when no new image was published from emulator)
    if next_frame < len(BUFFER_IMAGES)-1:
        #need this while condition since publish_image is called in trackmetrcicallback, so need to make sure that the next_frame is in bounds in buffer length
        #so untill next_frame exceeds the buffer length, we need to backoff the step. If next_frame exceeds the buffer length and if we don't bound it here then 
        #publish_image function will get skipped and the trackmetrcicallback won't have any value which in turn won't call publish_image
        while current_frame >= len(BUFFER_IMAGES):
            print("buff len, cur frame", len(BUFFER_IMAGES), current_frame, step, next_frame)
        #here, backingoff the step parameter and not the next frame as if we backoff the next_frame and not the step parameter, step keeps on increasing if tracking is good (reached 150 in 25 frames)
            step = max(1.0, step*BACKOFF_COEFF)
            current_frame = next_frame + int(round(step))
    else: 
        current_frame = next_frame
        count2+=1

    # if current_frame >= len(BUFFER_IMAGES):
    # #int round() because if step becomes equal to buffer length after rounding in publish image function, will generate index out of range error.
    #     current_frame = int(round(min(len(BUFFER_IMAGES)-1, current_frame*BACKOFF_COEFF)))
    next_frame = current_frame
    print("next_frame, step_size ", next_frame, round(step,2), last_successful_frame)
    # BUFFER_LOCK.release()


def calculateVelocity():
    global velocity, SLOWING_COEFF, BUFFER_IMAGES, DEFAULT_VEL
    # print("Slowing Coefficient:", SLOWING_COEFF)
    #updating the velocity as per the state of buffer
    velocity = max(1, DEFAULT_VEL - SLOWING_COEFF*(len(BUFFER_IMAGES) - BUFFER_ALLOWANCE)) #can clamp velocity at 1

def publish_image(event=None):
    global IMG_PUB, BUFFER_IMAGES, step, published_images, BUFFER_LOCK, pub_frame_time, next_frame
    # print("buffer images, step", len(BUFFER_IMAGES), step)
    # while BUFFER_LOCK.locked():
    #     continue
    # BUFFER_LOCK.acquire()
    if BUFFER_IMAGES and next_frame < len(BUFFER_IMAGES):
        
        current_time = rospy.Time.now()
        pub_frame_time = datetime.fromtimestamp(current_time.secs).strftime("%m/%d/%Y, %H:%M:%S")+"."+str(current_time.nsecs)
        IMG_PUB.publish(BUFFER_IMAGES[next_frame])
        print("Images in buffer: ", len(BUFFER_IMAGES))
        print("Frame published from buffer: ", next_frame)
    # BUFFER_LOCK.release()
    

def publish_velocity():
    global VEL_PUB, velocity
    calculateVelocity()
    print("Velocity sent: ", velocity)
    VEL_PUB.publish(velocity)


#create CSV files to store values of tracking metric, velocity and step size
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
    # rospy.Subscriber("/shutdown", Bool, shutdownCallback)
    # Publishing image to ORBslam
    # img_pub_timer = rospy.Timer(IMG_PUB_RATE, publish_image)
    rospy.spin()
    print("went back in time:", count)
    print("repeated images:", count2)
    save_values()


if __name__ == '__main__':
    try:
        main_func()

    except rospy.ROSInterruptException:
        rospy.logwarn("There is something wrong with the program!")

