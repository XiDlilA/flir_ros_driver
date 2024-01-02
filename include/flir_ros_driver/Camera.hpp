#include "Spinnaker.h"
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <cv_bridge/cv_bridge.h>

namespace flir_ros_driver
{
    class Camera
    {
    private:
        CameraPtr pCam;
        INodeMap *nodeMap;
        gcstring deviceSerialNumber = "";
        gcstring deviceModelName;
        gcstring deviceVendorName;
        gcstring device;

    public:
        explicit Camera(CameraPtr&&);
        ~Camera();
        
        int configureCustomImageSettings();
        int configureTrigger();
        int resetTrigger();

        gcstring Init();

        int run();

        inline bool setProperty(const std::string &, const std::string &);
        inline bool setProperty(const std::string &, const float &);
        inline bool setProperty(const std::string &, const bool &);
        inline bool setProperty(const std::string &, const int &);
        inline bool executeProperty(const std::string &);

        int startAcquisition();
        int stopAcquisition();
        int grabImage(sensor_msgs::Image*, const std::string&);
    };
}