#include "flir_ros_driver/Camera.hpp"

namespace flir_ros_driver{
    enum triggerType
    {
        SOFTWARE,
        HARDWARE
    };
    void infoTitle(std::string title){
        std::cout << std::endl << "*** "<< title <<" ***" << std::endl << std::endl;
    }
    Camera::Camera(CameraPtr&& pCam) {
        this->pCam = pCam;
        Init();
    }

    Camera::~Camera() {
    }

    gcstring Camera::Init(){
        gcstring cameraInfo = gcstring("");
        try
        {
            pCam->Init();
            nodeMap = &pCam->GetNodeMap();
            INodeMap& nodeMapTLDevice = this->pCam->GetTLDeviceNodeMap();
            CStringPtr ptrDeviceVendorName = nodeMapTLDevice.GetNode("DeviceVendorName");
            CStringPtr ptrDeviceModelName = nodeMapTLDevice.GetNode("DeviceModelName");
            if (IsAvailable(ptrDeviceVendorName) && IsReadable(ptrDeviceVendorName)) {
                deviceVendorName = ptrDeviceVendorName->ToString();
            }
            if (IsAvailable(ptrDeviceModelName) && IsReadable(ptrDeviceModelName)) {
                deviceModelName = ptrDeviceModelName->ToString();
            }
            cameraInfo = deviceVendorName + " " + deviceModelName; 
            deviceModelName = "[" + deviceModelName + "]";
            deviceVendorName = "[" + deviceVendorName + "]";
            deviceSerialNumber = static_cast<Spinnaker::GenApi::CStringPtr>(nodeMap->GetNode("DeviceID"))->GetValue();
            device = deviceModelName + "(" + deviceSerialNumber + "): ";
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR_STREAM("Error: " << e.what());
        }
        return cameraInfo;
    }

    int Camera::configureCustomImageSettings()
    {
        int result = 0;
        infoTitle("CONFIGURING CUSTOM IMAGE SETTINGS");
        try {
            setProperty("PixelFormat", (const std::string&)"BayerRG8");
            setProperty("OffsetX", 0);
            setProperty("OffsetY", 0);
            setProperty("Width", 1440);
            setProperty("Height", 1080);
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR_STREAM("Error: " << e.what());
            result = -1;
        }

        return result;
    }

    int Camera::run() {
        int result = 0;
        try
        {
            result |= configureCustomImageSettings();
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR_STREAM("Error: " << e.what());
            result = -1;
        }
        return result;
    }

    inline bool Camera::setProperty(const std::string& property_name, const std::string& entry_name) {
        Spinnaker::GenApi::CEnumerationPtr enumerationPtr = nodeMap->GetNode(property_name.c_str());

        if (!Spinnaker::GenApi::IsImplemented(enumerationPtr))
        {
            ROS_ERROR_STREAM(device << "Enumeration name " << property_name << " not "
                                                                        "implemented.");
            return false;
        }

        if (Spinnaker::GenApi::IsAvailable(enumerationPtr))
        {
            if (Spinnaker::GenApi::IsWritable(enumerationPtr))
            {
                Spinnaker::GenApi::CEnumEntryPtr enumEmtryPtr = enumerationPtr->GetEntryByName(entry_name.c_str());

                if (Spinnaker::GenApi::IsAvailable(enumEmtryPtr))
                {
                    if (Spinnaker::GenApi::IsReadable(enumEmtryPtr))
                    {
                        enumerationPtr->SetIntValue(enumEmtryPtr->GetValue());

                        ROS_INFO_STREAM(device << property_name << " set to " << enumerationPtr->GetCurrentEntry()->GetSymbolic()
                                        << ".");

                        return true;
                    }
                    else
                    {
                        ROS_WARN_STREAM(device << "Entry name " << entry_name << " not writable.");
                    }
                }
                else
                {
                    ROS_WARN_STREAM(device << "Entry name " << entry_name << " not available.");

                    ROS_WARN("Available:");
                    Spinnaker::GenApi::NodeList_t entries;
                    enumerationPtr->GetEntries(entries);
                    for (auto& entry : entries)
                    {
                    auto enumEntry = dynamic_cast<Spinnaker::GenApi::IEnumEntry*>(entry);
                    if (enumEntry && Spinnaker::GenApi::IsAvailable(entry))
                        ROS_WARN_STREAM(" - " << entry->GetName() << " (display " << entry->GetDisplayName() << ", symbolic "
                                            << enumEntry->GetSymbolic() << ")");
                    }
                }
            }
            else
            {
            ROS_WARN_STREAM(device << "Enumeration " << property_name << " not writable.");
            }
        }
        else
        {
            ROS_WARN_STREAM(device << "Enumeration " << property_name << " not available.");
        }
        return false;
        }

    inline bool Camera::setProperty(const std::string& property_name, const float& value)
        {
        Spinnaker::GenApi::CFloatPtr floatPtr = nodeMap->GetNode(property_name.c_str());

        if (!Spinnaker::GenApi::IsImplemented(floatPtr))
        {
            ROS_ERROR_STREAM(device << "Feature name " << property_name << " not implemented.");
            return false;
        }
        if (Spinnaker::GenApi::IsAvailable(floatPtr))
        {
            if (Spinnaker::GenApi::IsWritable(floatPtr))
            {
            float temp_value = value;
            if (temp_value > floatPtr->GetMax())
                temp_value = floatPtr->GetMax();
            else if (temp_value < floatPtr->GetMin())
                temp_value = floatPtr->GetMin();
            floatPtr->SetValue(temp_value);
            ROS_INFO_STREAM(device
                            << property_name << " set to " << floatPtr->GetValue() << ".");
            return true;
            }
            else
            {
            ROS_WARN_STREAM(device << "Feature " << property_name << " not writable.");
            }
        }
        else
        {
            ROS_WARN_STREAM(device << "Feature " << property_name << " not available.");
        }
        return false;
        }

    inline bool Camera::setProperty(const std::string& property_name, const bool& value)
        {
        Spinnaker::GenApi::CBooleanPtr boolPtr = nodeMap->GetNode(property_name.c_str());
        if (!Spinnaker::GenApi::IsImplemented(boolPtr))
        {
            ROS_ERROR_STREAM(device << "Feature name " << property_name << " not implemented.");
            return false;
        }
        if (Spinnaker::GenApi::IsAvailable(boolPtr))
        {
            if (Spinnaker::GenApi::IsWritable(boolPtr))
            {
            boolPtr->SetValue(value);
            ROS_INFO_STREAM(device << property_name << " set to " << boolPtr->GetValue() << ".");
            return true;
            }
            else
            {
            ROS_WARN_STREAM(device << "Feature " << property_name << " not writable.");
            }
        }
        else
        {
            ROS_WARN_STREAM(device << "Feature " << property_name << " not available.");
        }
        return false;
        }

    inline bool Camera::setProperty(const std::string& property_name, const int& value)
    {
        Spinnaker::GenApi::CIntegerPtr intPtr = nodeMap->GetNode(property_name.c_str());
        if (!Spinnaker::GenApi::IsImplemented(intPtr))
        {
            ROS_ERROR_STREAM(device << "Feature name " << property_name << " not implemented.");
            return false;
        }
        if (Spinnaker::GenApi::IsAvailable(intPtr))
        {
            if (Spinnaker::GenApi::IsWritable(intPtr))
            {
            int temp_value = value;
            if (temp_value > intPtr->GetMax())
                temp_value = intPtr->GetMax();
            else if (temp_value < intPtr->GetMin())
                temp_value = intPtr->GetMin();
            intPtr->SetValue(temp_value);
            ROS_INFO_STREAM(device << property_name << " set to " << intPtr->GetValue() << ".");
            return true;
            }
            else
            {
            ROS_WARN_STREAM(device << "Feature " << property_name << " not writable.");
            }
        }
        else
        {
            ROS_WARN_STREAM(device << "Feature " << property_name << " not available.");
        }
        return false;
    }

    inline bool Camera::executeProperty(const std::string &property_name){
        Spinnaker::GenApi::CCommandPtr commandPtr = nodeMap->GetNode(property_name.c_str());

        if (Spinnaker::GenApi::IsWritable(commandPtr))
        {
            commandPtr->Execute();
            ROS_INFO_STREAM(device << property_name << " execute.");
            return true;
        }
        else {
            ROS_WARN_STREAM(device << "Command " << property_name << " not writable.");
        }
        return false;
    }

    int Camera::startAcquisition() {
        int result = 0;
        try {
            setProperty("AcquisitionMode", (const std::string&)"Continuous");
            pCam->BeginAcquisition();
            ROS_INFO_STREAM("[Acquisition]: Acquisition started...");
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR_STREAM("Error: " << e.what());
            result = -1;
        }
        return result;
    }

    int Camera::stopAcquisition() {
        int result = 0;
        try
        {
            pCam->EndAcquisition();
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR_STREAM("Error: " << e.what());
            result = -1;
        }
        return result;
    }

    int Camera::grabImage(sensor_msgs::Image* image, const std::string& frame_id){
        int result = 0;
        try
        {
            // result = result | grabNextImageByTrigger();
            Spinnaker::ImagePtr image_ptr = pCam->GetNextImage(10000);
            std::string format(image_ptr->GetPixelFormatName());
            ROS_WARN_STREAM_ONCE("[Acquisition]: Format--"<< format.c_str());
            while (image_ptr->IsIncomplete())
            {
                ROS_WARN_STREAM_ONCE(device << "Camera is incomplete. Trying again.");
                image_ptr = pCam->GetNextImage(10000);
            }

            // Set Image Time Stamp
            image->header.stamp.sec = image_ptr->GetTimeStamp() * 1e-9;
            image->header.stamp.nsec = image_ptr->GetTimeStamp();

            // Check the bits per pixel.
            size_t bitsPerPixel = image_ptr->GetBitsPerPixel();

            // --------------------------------------------------
            // Set the image encoding
            std::string imageEncoding = sensor_msgs::image_encodings::MONO8;

            Spinnaker::GenApi::CEnumerationPtr color_filter_ptr =
                static_cast<Spinnaker::GenApi::CEnumerationPtr>(nodeMap->GetNode("PixelColorFilter"));

            Spinnaker::GenICam::gcstring color_filter_str = color_filter_ptr->ToString();
            Spinnaker::GenICam::gcstring bayer_rg_str = "BayerRG";
            Spinnaker::GenICam::gcstring bayer_gr_str = "BayerGR";
            Spinnaker::GenICam::gcstring bayer_gb_str = "BayerGB";
            Spinnaker::GenICam::gcstring bayer_bg_str = "BayerBG";

            // if(isColor_ && bayer_format != NONE)
            if (color_filter_ptr->GetCurrentEntry() != color_filter_ptr->GetEntryByName("None"))
            {
                if (bitsPerPixel == 16)
                {
                // 16 Bits per Pixel
                if (color_filter_str.compare(bayer_rg_str) == 0)
                {
                    imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB16;
                }
                else if (color_filter_str.compare(bayer_gr_str) == 0)
                {
                    imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG16;
                }
                else if (color_filter_str.compare(bayer_gb_str) == 0)
                {
                    imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG16;
                }
                else if (color_filter_str.compare(bayer_bg_str) == 0)
                {
                    imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR16;
                }
                else
                {
                    throw std::runtime_error("[SpinnakerCamera::grabImage] Bayer format not recognized for 16-bit format.");
                }
                }
                else
                {
                // 8 Bits per Pixel
                if (color_filter_str.compare(bayer_rg_str) == 0)
                {
                    imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB8;
                }
                else if (color_filter_str.compare(bayer_gr_str) == 0)
                {
                    imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG8;
                }
                else if (color_filter_str.compare(bayer_gb_str) == 0)
                {
                    imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG8;
                }
                else if (color_filter_str.compare(bayer_bg_str) == 0)
                {
                    imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR8;
                }
                else
                {
                    throw std::runtime_error("[Acquisition] Bayer format not recognized for 8-bit format.");
                }
                }
            }
            else  // Mono camera or in pixel binned mode.
            {
                if (bitsPerPixel == 16)
                {
                imageEncoding = sensor_msgs::image_encodings::MONO16;
                }
                else if (bitsPerPixel == 24)
                {
                imageEncoding = sensor_msgs::image_encodings::RGB8;
                }
                else
                {
                imageEncoding = sensor_msgs::image_encodings::MONO8;
                }
            }

            
            int width = image_ptr->GetWidth(); 
            int height = image_ptr->GetHeight();
            int stride = image_ptr->GetStride();

            fillImage(*image, imageEncoding, height, width, stride, image_ptr->GetData());
            image->header.frame_id = frame_id;
        }
        catch (const Spinnaker::Exception& e)
        {
            throw std::runtime_error("[Acquisition]: Failed to retrieve buffer with error: " +
                                std::string(e.what()));
            result = -1;
        }
        return result;
    }

    int Camera::configureTrigger()
    {
        int result = 0;
        infoTitle("CONFIGURING TRIGGER");
        ROS_INFO_STREAM("[Trigger]: Hardware trigger chosen...");
        try
        {
            setProperty("TriggerMode", (const std::string&)"Off");
            setProperty("TriggerSelector", (const std::string&)"FrameStart");
            const std::string &triggerSelector = "Line0";
            setProperty("TriggerSource", triggerSelector);
            setProperty("TriggerMode", (const std::string&)"On");
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR_STREAM("Error: " << e.what());
            result = -1;
        }
        return result;
    }

    int Camera::resetTrigger()
    {
        int result = 0;
        infoTitle("RESET TRIGGER");
        try
        {
            ROS_INFO_STREAM("[Trigger]: ----- Trigger mode disabled -----");
            setProperty("TriggerMode", (const std::string&)"Off");
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR_STREAM("Error: " << e.what());
            result = -1;
        }
        return result;
    }

}