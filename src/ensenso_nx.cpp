#include "ensenso_nx.h"
#include "ros/ros.h"

void ensensoExceptionHandling (const NxLibException &ex,
                               std::string func_nam)
{
    ROS_ERROR ("%s: NxLib error %s (%d) occurred while accessing item %s.\n", func_nam.c_str (), ex.getErrorText ().c_str (), ex.getErrorCode (),
               ex.getItemPath ().c_str ());
    if (ex.getErrorCode () == NxLibExecutionFailed)
    {
        NxLibCommand cmd ("");
        ROS_WARN ("\n%s\n", cmd.result ().asJson (true, 4, false).c_str ());
    }
}

pcl::uint64_t getPCLStamp (const double ensenso_stamp)
{
#if defined _WIN32 || defined _WIN64
    return (ensenso_stamp * 1000000.0);
#else
    return ( (ensenso_stamp - 11644473600.0) * 1000000.0);
#endif
}

namespace EnsensoNx
{

    Device::Device(const std::string & _serial_num)
    {
        std::cout << "EnsensoNx::Device: Opening camera ..." << std::endl;

        //init nx library
        nxLibInitialize(true);

        // Create an object referencing the camera's tree item, for easier access:
        camera_ = nx_lib_root_[itmCameras][itmBySerialNo][_serial_num];
        if (!camera_.exists() || (camera_[itmType] != valStereo))
        {
            std::cout << "EnsensoNx::Device: Camera not found. Please connect a single stereo camera to your computer" << std::endl;
            return;
        }

        //get serial number of the connected camera
        device_params_.serial_num_ = camera_[itmSerialNumber].asString();

        //open camera
        NxLibCommand open(cmdOpen); // When calling the 'execute' method in this object, it will synchronously execute the command 'cmdOpen'
        open.parameters()[itmCameras] = device_params_.serial_num_; // Set parameters for the open command
        open.execute();
        std::cout << "EnsensoNx::Device: Camera open. SN: " << device_params_.serial_num_ << std::endl;

        fx = camera_[itmCalibration][itmDynamic][itmStereo][itmLeft][itmCamera][0][0].asDouble();
        fy = camera_[itmCalibration][itmDynamic][itmStereo][itmLeft][itmCamera][1][1].asDouble();
        cx =  camera_[itmCalibration][itmDynamic][itmStereo][itmLeft][itmCamera][2][0].asDouble();
        cy = camera_[itmCalibration][itmDynamic][itmStereo][itmLeft][itmCamera][2][1].asDouble();

        std::cout << "EnsensoNx::Device: Camera open. SN: " << device_params_.serial_num_ << std::endl;

        std::cout << "fx, fy: " << fx << ", " << fy << std::endl;
        std::cout << "cx, cy: " << cx << ", " << cy << std::endl;


    }

    Device::~Device()
    {
        //close the camera
        std::cout << "EnsensoNx::Device: Closing camera ..." << std::endl;
        NxLibCommand (cmdClose).execute();
        std::cout << "EnsensoNx::Device: Camera closed." << std::endl;

        //finalizes nx library
        nxLibFinalize();
    }



    void Device::configureCapture(const CaptureParams & _params)
    {
        //update class member
        capture_params_.auto_exposure_ = _params.auto_exposure_;
        capture_params_.exposure_time_ = _params.exposure_time_;
        capture_params_.dense_cloud_ = _params.dense_cloud_;

        //call protected member to set the configuration to the camera
        this->configureCapture();
    }

// void Device::configureExposure(unsigned int _exposure)
// {
//     if (_exposure == 0) //autoexposure case
//     {
//         capture_params_.auto_exposure_ = true;
//     }
//     else //manual exposure case
//     {
//         capture_params_.auto_exposure_ = false;
//         capture_params_.exposure_time_ = _exposure;
//     }
//
//     //call protected member to set the configuration to the camera
//     this->configureCapture();
// }
    void Device::preCapture() {
        // Capture images
        try {

            NxLibCommand (cmdCapture).execute();
        }
        catch (NxLibException &ex)
        {
            ensensoExceptionHandling (ex, "PreCapture");
        }
    }

    int Device::capture(sr::rgbd::Image & _rgb_image)
    {
        try {
            // Capture images
            //NxLibCommand (cmdCapture).execute();

            // Compute Disparity Map
            NxLibCommand (cmdComputeDisparityMap).execute();

            // Compute Point Cloud
            NxLibCommand (cmdComputePointMap).execute();

            // Get image dimensions
            int width, height;
            double time_stamp;
            camera_[itmImages][itmPointMap].getBinaryDataInfo(&width, &height, nullptr,
                                                              nullptr, nullptr, &time_stamp);
            //std::cout << "ts1: " << std::fixed << time_stamp << std::endl;
            //Get 3D image raw data
            int nx_return_code;
            camera_[itmImages][itmPointMap].getBinaryData(&nx_return_code, raw_points_, &time_stamp);

            //Move raw data to rgbd image
            _rgb_image.depth = cv::Mat( (unsigned int)height, (unsigned int)width, CV_32FC1, NAN);
            _rgb_image.timestamp = getPCLStamp(time_stamp) * 1000;

            _rgb_image.P.setOpticalTranslation(0, 0);
            _rgb_image.P.setOpticalCenter(cx, cy);
            _rgb_image.P.setFocalLengths(fx, fy);
            float px, py, pz;
            for(unsigned int i = 0; i < width * height; ++i) {
                px = raw_points_[i * 3];
                if (!std::isnan(px)) {
                    px /= 1000.;
                    py = raw_points_[i * 3 + 1] / 1000;
                    pz = raw_points_[i * 3 + 2] / 1000;
                    sr::Vec2i pix = _rgb_image.P.project3Dto2D(sr::Vec3(px, -py, -pz));
                    if (pix.x >= 0 && pix.y >= 0 && pix.x < _rgb_image.depth.cols && pix.y < _rgb_image.depth.rows) {
                        _rgb_image.depth.at<float>(pix.y, pix.x) = pz;
                    }
                }

            }
           return 1;
            //return success
        }
        catch (NxLibException &ex)
        {
            ensensoExceptionHandling (ex, "grabRGBDImage");
            return 0;
        }

    }

    int Device::capture(pcl::PointCloud<pcl::PointXYZ> & _p_cloud)
    {
        try {
            int ww, hh;
            float px;

            //std::vector<float> raw_points;
            int nx_return_code;

            // Capture images
            //NxLibCommand(cmdCapture).execute();

            // Compute Disparity Map
            NxLibCommand(cmdComputeDisparityMap).execute();

            // Compute Point Cloud
            NxLibCommand(cmdComputePointMap).execute();
            double time_stamp;
            // Get image dimensions
            camera_[itmImages][itmPointMap].getBinaryDataInfo(&ww, &hh, 0, 0, 0, &time_stamp);

            //Get 3D image raw data
            camera_[itmImages][itmPointMap].getBinaryData(&nx_return_code, raw_points_, 0);

            //Move raw data to point cloud
            _p_cloud.width = (unsigned int) ww;
            _p_cloud.height = (unsigned int) hh;
            _p_cloud.header.stamp = getPCLStamp(time_stamp);

            _p_cloud.resize(_p_cloud.width * _p_cloud.height);
            unsigned int kk = 0;
            for (unsigned int ii = 0; ii < _p_cloud.height; ii++) {
                for (unsigned int jj = 0; jj < _p_cloud.width; jj++) {
                    px = raw_points_[(ii * _p_cloud.width + jj) * 3];
                    if (!std::isnan(px)) {
                        _p_cloud.points.at(kk).x = px / 1000.;
                        _p_cloud.points.at(kk).y = raw_points_[(ii * _p_cloud.width + jj) * 3 + 1] / 1000.;
                        _p_cloud.points.at(kk).z = raw_points_[(ii * _p_cloud.width + jj) * 3 + 2] / 1000.;
                        kk++;
                    } else //in case of nan, check dense_cloud_ to fill in the cloud or not
                    {
                        if (capture_params_.dense_cloud_) {
                            _p_cloud.points.at(kk).x = std::nan("");
                            _p_cloud.points.at(kk).y = std::nan("");
                            _p_cloud.points.at(kk).z = std::nan("");
                            kk++;
                        } else {
                            //nothing to do , the point is lost
                        }

                    }

                }
            }

            //resize with number valid points. If _dense_cloud, just set the flag ordered to true
            _p_cloud.resize(
                    kk);//checks if kk=ww*hh to set the cloud as ordered (width,height) or unordered (width=size,height=1)
            _p_cloud.is_dense = capture_params_.dense_cloud_;

            //debug message
//     std::cout << "Cloud capture: " << std::endl <<
//                  "\treturn code: " << nx_return_code << std::endl <<
//                  "\tnum points: " << raw_points_.size()/3 << std::endl <<
//                  "\twidth: " << ww << std::endl <<
//                  "\theight: " << hh << std::endl <<
//                  "\tvalid_points: " << kk << std::endl;

            //return success
            return 1;
        }
        catch (NxLibException &ex)
        {
            ensensoExceptionHandling (ex, "grabSingleCloud");
            return 0;
        }
    }

//PROTECTED METHODS

    void Device::configureCapture()
    {
        //sets capture configuration to the camera
        camera_[itmParameters][itmCapture][itmAutoExposure] = capture_params_.auto_exposure_;
        camera_[itmParameters][itmCapture][itmExposure    ] = (double)capture_params_.exposure_time_;//TODO check if requires cast to double.

        //print out
        //std::cout << "EnsensoNx::Device: Capture params set to:" << std::endl;
        //capture_params_.print();
    }

}//close namespace
