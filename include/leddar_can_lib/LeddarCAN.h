#ifndef LeddarCAN_H
#define LeddarCAN_H

//ROS Includes
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <sensor_msgs/LaserScan.h>
//#include <std_msgs/String.h>


class LeddarCAN
{
    public:
        LeddarCAN();
        ~LeddarCAN();
        
        //returns parameter rate_ in hz
        // used to define ros::Rate
        int get_rate();
        
        // sends 740#01 if not max_stream_
        void requestSingleScan();
        
    private:
        void leddarCallback(const can_msgs::Frame& frame);
        //void leddarCallback(const can_msgs::Frame::ConstPtr& frame);
        
        // sends 740#02
        void startMaxStream(); 
        
        // sends 740#03
        void stopStream();


        ros::NodeHandle nh_;
        ros::NodeHandle nh_p;
        ros::Publisher request_pub_;
        ros::Publisher scan_pub_;
        ros::Subscriber can_sub_;
        
        uint8_t num_detections; //number of detections given in 0x751
        uint8_t det_count; //counter to check we have the expected scan data
        
        sensor_msgs::LaserScan scan;
        can_msgs::Frame can_request;
        
        //parameters
        bool max_stream_; //Default false, set to true to tell leddar to stream data continuously
        int rate_; //Default 5 Hz, how often we send a request for a single leddar scan
        double min_amp_slope_, min_amp_offset_;
};

#endif
