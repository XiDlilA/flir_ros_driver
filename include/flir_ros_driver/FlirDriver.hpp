#include "flir_ros_driver/Camera.hpp"
#include <image_transport/image_transport.h>

namespace flir_ros_driver
{
    class FlirDriver
    {
    private:
        Camera* cameraPtr;

    public:
        explicit FlirDriver(CameraPtr&&);
        ~FlirDriver();

        int run();
    };

    void infoTitle(std::string);
    gcstring getSpinnakerLibraryVersion(SystemPtr &);
}