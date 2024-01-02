#include "flir_ros_driver/FlirDriver.hpp"

namespace flir_ros_driver
{
    FlirDriver::FlirDriver(CameraPtr&& cameraPtr){
        this->cameraPtr = new Camera(std::move(cameraPtr));
    }
    FlirDriver::~FlirDriver(){
        delete cameraPtr;
    }
    int FlirDriver::run() {
        int result = 0;
        try
        {
            cameraPtr->configureCustomImageSettings();
            cameraPtr->startAcquisition();
            ros::NodeHandle nh;
            image_transport::ImageTransport it(nh);
            image_transport::Publisher image_raw_pub = it.advertise("image_raw", 1);
            image_transport::Publisher image_color_pub = it.advertise("image_color", 1);
            sensor_msgs::Image image;
            while(ros::ok())
            {
                cameraPtr->grabImage(&image, "BGR8");
                image_raw_pub.publish(image);

                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
                image_color_pub.publish(cv_ptr->toImageMsg());
                // ROS_INFO_STREAM(image.header.stamp.sec << " " << image.header.stamp.nsec);
                
                ros::spinOnce();
            }
            cameraPtr->stopAcquisition();
        }
        catch (Spinnaker::Exception &e){
            std::cout << "Error: " << e.what() << std::endl;
            result = -1;
        }
        return result;
    }

    void infoTitle(std::string title){
        std::cout << std::endl << "*** "<< title <<" ***" << std::endl << std::endl;
    }
    gcstring getSpinnakerLibraryVersion(SystemPtr& system){
        gcstring version = gcstring("");
        try {
            const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
            std::stringstream ss;
            ss << spinnakerLibraryVersion.major << "."
            << spinnakerLibraryVersion.minor << "."
            << spinnakerLibraryVersion.type  << "."
            << spinnakerLibraryVersion.build;
            version += ss.str().c_str();
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR("%s", e.what());
        }
        return version;
    }
}