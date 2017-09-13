#include "ensenso_nx_node.h"

EnsensoNxNode::EnsensoNxNode():
    nh_() //node handle without additional namespace
{
    int param_int;
	std::string param_str;
	std::string ns_str;

    //init the point cloud publisher
    cloud_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("ensenso_cloud", 1);

    //init server
    cloud_server_ = nh_.advertiseService("ensenso_server", &EnsensoNxNode::pointCloudServiceCallback, this);

    //Allocate the ensenso device with the provided serial number
	ros::param::get("serial_number", param_str);
    camera_ = new EnsensoNx::Device(param_str);

    //configure node according yaml params
    ros::param::get("run_mode", param_int); this->run_mode_ = (RunMode)param_int;
    ros::param::get("rate", this->rate_);
	  ns_str = ros::this_node::getNamespace();
	  ns_str.erase (ns_str.begin()); //it starts with "//", so one slash has to be removed. Maybe it returns concatenation of ns from node and nodehandle
    ros::param::get("frame_name", param_str);
	  this->frame_name_ = param_str;
	  std::cout << this->frame_name_ << std::endl;
    ros::param::get("auto_exposure", this->capture_params_.auto_exposure_);
    ros::param::get("exposure_time", param_int); this->capture_params_.exposure_time_ = (unsigned int)param_int;
    ros::param::get("dense_cloud", param_int); this->capture_params_.dense_cloud_ = (bool)param_int;
    if ( run_mode_ == PUBLISHER )
    {
        camera_->configureCapture(this->capture_params_);
    }

    //print configs
    std::cout << "ROS EnsensoNxNode Settings: " << std::endl;
    std::cout << "\trun mode: \t" << run_mode_ << std::endl;
    std::cout << "\tframe name: \t" << frame_name_ << std::endl;
    if ( run_mode_ == PUBLISHER ) //in SERVER, rate is not applicable, and other capture params are set at the request message
    {
        std::cout << "\trate [hz]: \t" << rate_  << std::endl;
        std::cout << "\tauto_exposure [hz]: \t" << capture_params_.auto_exposure_ << std::endl;
        if ( !capture_params_.auto_exposure_ )
        {
            std::cout << "\texposure [ms]: \t" << capture_params_.exposure_time_ << std::endl;
        }
        std::cout << "\tdense_cloud: [t/f] \t" << capture_params_.dense_cloud_ << std::endl;
    }
}

EnsensoNxNode::~EnsensoNxNode()
{
    //destroy pointer to ensenso device
    delete camera_;
}

RunMode EnsensoNxNode::runMode() const
{
    return run_mode_;
}

double EnsensoNxNode::rate() const
{
    return rate_;
}

bool EnsensoNxNode::publish()
{
    //Get a single capture from camera and publish the point cloud
    if ( camera_->capture(cloud_) == 1 )
    {
        //get time
        ros::Time ts = ros::Time::now();

        //publish the cloud
        cloud_.header.stamp = (pcl::uint64_t)(ts.toSec()*1e6); //TODO: should be set by the EnsensoNx::Device class
        cloud_.header.frame_id = frame_name_;
        cloud_publisher_.publish(cloud_);
        return true;
    }

    std::cout << "EnsensoNxNode::publish(): Error with point cloud capture" << std::endl;
    return false;
}

bool EnsensoNxNode::pointCloudServiceCallback(std_srvs::Trigger::Request  & _request,
                                              std_srvs::Trigger::Response & _reply)
{

    capture_params_.auto_exposure_ = true;
    capture_params_.dense_cloud_ = true;
    camera_->configureCapture(capture_params_);

    if (this->publish()){
        _reply.message = "Successfully published.";
        _reply.success = true;
    }
    else {
        _reply.message = "Failed to publish cloud.";
        _reply.success = false;
    }

    //return
    return true;
}
