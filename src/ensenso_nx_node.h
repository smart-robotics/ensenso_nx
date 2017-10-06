#ifndef ensenso_nx_node_H
#define ensenso_nx_node_H

//std
#include <iostream>

//this package
#include "ensenso_nx.h"

//std ros dependencies
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h> //PCL-ROS interoperability
#include <pcl_conversions/pcl_conversions.h> //conversions from/to PCL/ROS
//#include <std_msgs/Empty.h> //snapshot request
#include <std_srvs/Trigger.h>

#include <rgbd/RGBDImage.h>
#include <sr/rgbd/serialization.h>

//this package dependencies
#include <ensenso_nx/ensenso_nx_paramsConfig.h> //ROS dynamic configure

//enum run mode
enum RunMode {SERVER=0,PUBLISHER};

/** \brief Ensenso NX ROS wrapper for point cloud capture
 *
 * Ensenso NX ROS wrapper for point cloud capture
 *
 * Two running modes:
 *    * SERVER: Snapshot upon request
 *    * PUBLISHER: Continuous point cloud publication
 *
 * In both cases the point cloud is published thorugh the same topic
 *
 **/
class EnsensoNxNode
{
    protected:
        //Device object with HW API
        EnsensoNx::Device *camera_;

        //ros node handle
        ros::NodeHandle nh_;

        //capture server
        ros::ServiceServer cloud_server_;

        //Publisher. Point Clouds are published through this topic
        ros::Publisher cloud_publisher_, rgbd_publisher_;

        //published point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud_;

        //RGBD image
        sr::rgbd::Image image_;
        rgbd::RGBDImage rgbd_msg;


    //node configuration parameters
        RunMode run_mode_;//run mode: The node acts as a server, or a continuous pcl publisher
        double rate_; //loop rate
        std::string frame_name_; //name of the frame of references with respect cloud are published
        bool rgbd;

        //device parameters TODO: think if it is necessary, ... it seems not!
        //EnsensoNx::DeviceParams device_params_;

        //capture parameters
        EnsensoNx::CaptureParams capture_params_;

    public:
        //constructor
        EnsensoNxNode();

        //destructor
        ~EnsensoNxNode();

        //returns run_mode_
        RunMode runMode() const;

        //returns rate_ value
        double rate() const;
        bool do_publish;
        //Call to device snapshot acquisition and publish the point cloud
        bool publish();
        bool publishRGBD();
        bool publishCloud();

        void preCapture();


protected:
        //Service callback implementing the point cloud capture
        bool publishServiceCallback(std_srvs::Trigger::Request &_request,
                                    std_srvs::Trigger::Response &_reply);

};
#endif
