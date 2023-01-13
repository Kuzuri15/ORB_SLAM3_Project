/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <std_msgs/Int32.h>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include<Tracking.h>
#include"../../../include/System.h"

using namespace std;
ros::Publisher metric_pub;     //initialized publisher to publish metric value
std_msgs::Int32 track_metric;  //to store metric value which is published to Buffer node from here
std_msgs::Int32 state;
vector<float> time_track;      //keeps track of processing time for each frame
int val_count = 0;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{   
    time_track.resize(5000); //setting vector size
    ofstream myFile;
    myFile.open("Time_Process_ROS.csv"); //creating file to save time statistics

    ros::init(argc, argv, "Mono");
    ros::start();
    ros::Rate rate(10);
    
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);
    
    ros::NodeHandle nodeHandler;

    //created publisher to publish metric value to buffer node on topic /tracking_metric
    metric_pub = nodeHandler.advertise<std_msgs::Int32>("/tracking_metric", 1);
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb); 

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    //adding processing time per frame to the file
    for (float val : time_track){
        myFile << val << "," << endl;
    }

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{   
    
    //initiating clock
    auto start_time = std::chrono::steady_clock::now();

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // ROS_INFO("processing time: ");
    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
    //getting metric value from ORBSLAM system after image is passed to the system for processing
    track_metric.data = mpSLAM->GetmpTracker()->GetMatchesInliers();
    cout << "Matches metric: " << track_metric.data << endl;
    //getting mState
    state.data = mpSLAM->GetmpTracker()->mState;
    cout << "mstate value: " << state.data << endl;
    
    if (state.data == 4){
        state.data = -1;
        metric_pub.publish(state); //publishing metric value to buffer node
    }
    else{
        metric_pub.publish(track_metric);
    }
    //ending clock
    auto end_time = std::chrono::steady_clock::now();
    //calculating difference between start and end time
    double process_time = std::chrono::duration_cast<std::chrono::duration<double> >(end_time - start_time).count();
    std::cout << "Processing time per frame: " << process_time << "s\n";
    time_track[val_count]= process_time;  //appending processing time to time_track vector
    val_count++;  //incrementing pointer inside time_track vector to store new value
}
