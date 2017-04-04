#ifndef _UNDISTORTER_KANNALA_BRANDT_HPP_
#define _UNDISTORTER_KANNALA_BRANDT_HPP_

#include "rectification/Undistorter.h"
#include <Eigen/Core>
#include <Eigen/Dense>

namespace cc
{
    class UndistorterKannalaBrandt : public Undistorter
    {
    public:
        /**
        * Creates an Undistorter by reading the distortion parameters from a file.
        *
        * The file format is as follows:
        * mu mv u0 v0 k2 k3 k4 k5 d
        * inputWidth inputHeight
        * crop / full / none
        * outputWidth outputHeight
        */
        UndistorterKannalaBrandt(const char* configFileName);
        
        /** Creates an Undistorter by specifying calibration parameters directly.
            \param inputWidth Input frame width
            \param inputHeight Input frame height
            \param outputWidth Output frame width
            \param outputHeight Output frame height
            \param mu Focal length's x component
		    \param mv Focal length's y component
		    \param u0 Principal point's x component
		    \param v0 Principal point's y component
            \param k2 Distortion parameter #1
            \param k3 Distortion parameter #2
            \param k4 Distortion parameter #3
            \param k5 Distortion parameter #4
        */
        UndistorterKannalaBrandt(int inputWidth, int inputHeight,
            int outputWidth, int outputHeight,
            float mu, float mv, float u0, float v0,
            float k2, float k3, float k4, float k5);

        /**
        * Destructor.
        */
        ~UndistorterKannalaBrandt();

        UndistorterKannalaBrandt& operator=(const UndistorterKannalaBrandt&) = delete;

        /**
        * Undistorts the given image and returns the result image.
        */
        void undistort(const cv::Mat& image, cv::OutputArray result) const;

        /**
        * Returns the intrinsic parameter matrix of the undistorted images.
        */
        const cv::Mat& getK() const;

        /**
        * Returns the intrinsic parameter matrix of the original images,
        */
        const cv::Mat& getOriginalK() const;

        /**
        * Returns the width of the undistorted images in pixels.
        */
        int getOutputWidth() const;

        /**
        * Returns the height of the undistorted images in pixels.
        */
        int getOutputHeight() const;


        /**
        * Returns the width of the input images in pixels.
        */
        int getInputWidth() const;

        /**
        * Returns the height of the input images in pixels.
        */
        int getInputHeight() const;

        /**
        * Returns if the undistorter was initialized successfully.
        */
        bool isValid() const;

    private:
        cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
            float fx, float fy, float cx, float cy, cv::Size imageSize) const;

        // Projects 3D points to the image plane (Pi function)
        void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const;

    private:
        cv::Mat K_;
        cv::Mat originalK_;

        float inputCalibration[10];
        float outputCalibration;
        int out_width, out_height;
        int in_width, in_height;
        cv::Mat map1, map2;

        /// Is true if the undistorter object is valid (has been initialized with
        /// a valid configuration)
        bool valid;

        // projection
        double m_k2;
        double m_k3;
        double m_k4;
        double m_k5;

        double m_mu;
        double m_mv;
        double m_u0;
        double m_v0;
    };
}



#endif	// _UNDISTORTER_KANNALA_BRANDT_HPP_