#include "flir_ros_driver/FlirDriver.hpp"

int main(int argc, char **argv)
{
    int result = 0;
    try
    {
        SystemPtr system = System::GetInstance();
        ROS_INFO_STREAM("Spinnaker Library Version: " << flir_ros_driver::getSpinnakerLibraryVersion(system));
        CameraList camList = system->GetCameras();
        unsigned int numCameras = camList.GetSize();
        if (numCameras == 0) { 
            camList.Clear();
            system->ReleaseInstance();
            ROS_ERROR("Not enough cameras!");
            return -1;
        }
        ros::init(argc, argv, "flir_image");
        for (unsigned int i = 0; i < numCameras; i++)
        {
            flir_ros_driver::FlirDriver flirDriver = flir_ros_driver::FlirDriver(camList.GetByIndex(0));
            flirDriver.run();
        }
        camList.Clear();
        system->ReleaseInstance();
    }
    catch (Spinnaker::Exception& e)
    {
        ROS_ERROR("%s", e.what());
        return -1;
    }
    return result;
}