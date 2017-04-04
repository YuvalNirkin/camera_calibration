#include "camera_models/CameraFactory.h"

#include "ceres/ceres.h"
#include "camera_models/CataCamera.h"
#include "camera_models/EquidistantCamera.h"
#include "camera_models/PinholeCamera.h"

namespace cc
{

std::shared_ptr<CameraFactory> CameraFactory::m_instance;

CameraFactory::CameraFactory()
{

}

std::shared_ptr<CameraFactory>
CameraFactory::instance(void)
{
    if (!m_instance)
    {
        m_instance = std::make_shared<CameraFactory>();
    }

    return m_instance;
}

CameraPtr
CameraFactory::generateCamera(Camera::ModelType modelType,
                              const std::string& cameraName,
                              const std::string& cameraType,
                              cv::Size imageSize) const
{
    switch (modelType)
    {
    case Camera::KANNALA_BRANDT:
    {
        EquidistantCameraPtr camera = std::make_shared<EquidistantCamera>();

        EquidistantCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.cameraType() = cameraType;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::PINHOLE:
    {
        PinholeCameraPtr camera = std::make_shared<PinholeCamera>();

        PinholeCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.cameraType() = cameraType;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::MEI:
    default:
    {
        CataCameraPtr camera = std::make_shared<CataCamera>();

        CataCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.cameraType() = cameraType;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    }
}

}

