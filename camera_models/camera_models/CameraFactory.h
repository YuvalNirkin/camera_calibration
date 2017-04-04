#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H

#include "camera_models/Camera.h"
#include <opencv2/core/core.hpp>

namespace cc
{

class CameraFactory
{
public:
    CameraFactory();

    static std::shared_ptr<CameraFactory> instance(void);

    CameraPtr generateCamera(Camera::ModelType modelType,
                             const std::string& cameraName,
                             const std::string& cameraType,
                             cv::Size imageSize) const;

private:
    static std::shared_ptr<CameraFactory> m_instance;
};

}

#endif
