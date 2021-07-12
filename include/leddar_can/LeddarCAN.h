#ifndef LeddarCAN_H
#define LeddarCAN_H

//ROS Includes
#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
//#include <std_msgs/String.h>


class LeddarCAN: public rclcpp::Node
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
        void leddarCallback(const can_msgs::msg::Frame::SharedPtr frame);
        //void leddarCallback(const can_msgs::Frame::ConstPtr& frame);
        
        // sends 740#02
        void startMaxStream(); 
        
        // sends 740#03
        void stopStream();

        rclcpp::TimerBase::SharedPtr m_timer;

        std::shared_ptr<rclcpp::Subscription<can_msgs::msg::Frame> > can_sub_;
        std::shared_ptr<rclcpp::Publisher<can_msgs::msg::Frame> > request_pub_;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan> > scan_pub_;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan> > scan2_pub_;
        
        uint8_t num_detections; //number of detections given in 0x751
        uint8_t det_count; //counter to check we have the expected scan data
        
        sensor_msgs::msg::LaserScan scan, scan2;
        can_msgs::msg::Frame can_request;
        
        //parameters
        bool max_stream_; //Default false, set to true to tell leddar to stream data continuously
        int rate_; //Default 5 Hz, how often we send a request for a single leddar scan
        double min_amp_slope_, min_amp_offset_;
};

#endif
