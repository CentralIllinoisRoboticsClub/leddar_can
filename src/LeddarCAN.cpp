#include "LeddarCAN.h"

/**********************************************************************
* Leddar CAN communication to ROS
* Requires:
* sudo apt install ros-kinetic-ros-opencan
* rosrun socketcan_bridge socketcan_bridge_node
*   The socket_can bridge turns CAN into /received_messages topic
*   The socket_can bridge turns /sent_messages into CAN 
* 
* Publish 740#01 at a specified frequency or just 740#02 once to request Leddar scan data
* Subscribe to /received_messages and respond to Leddar CAN ids 751 and 750
* 
* The leddar.launch file launches this node as well as a static transform between base_link and laser
**********************************************************************/

//Constructor
LeddarCAN::LeddarCAN()
{
    //Topics you want to publish
    request_pub_ = nh_.advertise<can_msgs::Frame>("sent_messages", 50); //send CAN
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10); //send laser scan

    //Topic you want to subscribe
    // leddarCallback will run every time ros sees the topic /received_messages
    can_sub_ = nh_.subscribe("received_messages", 50, &LeddarCAN::leddarCallback, this); //receive CAN
    
    nh_.param("max_stream_flag", max_stream_, false); //set in leddar.launch
    nh_.param("rate_hz", rate_, 5); //set in leddar.launch
    nh_.param("min_amp_slope", min_amp_slope_, 1.5); //set in leddar.launch
    nh_.param("min_amp_offset", min_amp_offset_, 15.0); //set in leddar.launch
    
    num_detections = 0; //number of detections given in 0x751
    det_count = 0; //counter to check we have the expected scan data
    
    //Constant properties of published data
    can_request.id = 0x00000740;
    can_request.is_extended = true;
    can_request.dlc = 1;
    
    scan.header.frame_id = "laser";
    scan.angle_min = -0.3925; //-22.5*3.14/180.0
    scan.angle_max = 0.392; //22.5*3.14/180.0
    scan.angle_increment = 0.0523; //3.0*3.14/180.0
    scan.range_min = 0.1;
    scan.range_max = 50.0;
    
    stopStream(); //Make sure leddar is not continuously streaming initially
}

//Default destructor.
LeddarCAN::~LeddarCAN()
{
    stopStream();
}

int LeddarCAN::get_rate()
{
    return rate_;
}

void LeddarCAN::requestSingleScan()
{
    if(!max_stream_)
    {
        can_request.header.stamp = ros::Time::now();
        can_request.data[0] = 1;
        request_pub_.publish(can_request);
    }
}

void LeddarCAN::leddarCallback(const can_msgs::Frame& frame)
{    
    //can id
    uint32_t id = frame.id;
    
    // look for 0x751 to start a scan reading
    if(id == 0x00000751)
    {
        if(det_count > 0)
        {
            scan_pub_.publish(scan);
        }
        num_detections = frame.data[0];
        scan.header.stamp = frame.header.stamp;
        scan.ranges.clear();
        scan.ranges.resize(16,0);
        scan.intensities.clear();
        scan.intensities.resize(16,0);
        det_count = 0;
    }
    else if(id == 0x00000750 && num_detections > 0)
    {
        // Two laser data sets per CAN msg
        // (DIST_BYTE_L, DIST_BYTE_H, Amp_Byte_L, LaserNum4BITS-Amp4BITS) x 2
        for(int k=0; k<2; ++k)
        {
            ++det_count;
            if(det_count <= num_detections)
            {
                //byte 3 or 7 shifted 4 bits
                uint8_t laser_num = frame.data[3+k*4] >> 4;
                
                //Dist bytes 1,0 or 5,4
                float range_meters = (float)(frame.data[1+k*4]<<8 | frame.data[0+k*4])/100.0; // cm to meters
                //Intensity is ampH(byte 3 or 7 right 4 bits), byte 2 or 6 (12 bits)
                uint8_t ampH = frame.data[3+k*4] & 0x0F; //keep just the right 4 bits
                float amplitude = (float)(ampH<<8 | frame.data[2+k*4])/4.0;
                
                float min_amplitude = (float)min_amp_offset_ - (float)min_amp_slope_*range_meters;
                /*if(min_amplitude < 0)
                {
                    min_amplitude = 0; //not needed because amplitude >= 0
                }*/
                
                if(amplitude > min_amplitude)
                {
                    scan.ranges[laser_num] = range_meters;
                    scan.intensities[laser_num] = amplitude;
                }             

            }
        }
        if(det_count == num_detections)
        {
            scan_pub_.publish(scan);
            det_count = 0;
        }
    }
}

void LeddarCAN::startMaxStream()
{
    can_request.header.stamp = ros::Time::now();
    can_request.data[0] = 2;
    request_pub_.publish(can_request);
}

void LeddarCAN::stopStream()
{
    can_request.header.stamp = ros::Time::now();
    can_request.data[0] = 3;
    request_pub_.publish(can_request);
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "leddar_scan");

    LeddarCAN leddar_can;
    ROS_INFO("Starting Leddar CAN");
    int loop_time = leddar_can.get_rate();
    ros::Rate rate(loop_time);

    while(ros::ok())
    {
        ros::spinOnce();
        leddar_can.requestSingleScan();
        rate.sleep();
    }

    return 0;
}
