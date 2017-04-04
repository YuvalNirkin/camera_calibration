#include "rectification/UndistorterKannalaBrandt.h"
#include <sstream>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace cc
{
    UndistorterKannalaBrandt::UndistorterKannalaBrandt(const char* configFileName)
    {
        valid = true;

        // read parameters
        std::ifstream infile(configFileName);
        assert(infile.good());

        std::string l1, l2, l3, l4;

        std::getline(infile, l1);
        std::getline(infile, l2);
        std::getline(infile, l3);
        std::getline(infile, l4);

        // l1 & l2
        if (std::sscanf(l1.c_str(), "%f %f %f %f %f %f %f %f %f",
            &inputCalibration[0], &inputCalibration[1], &inputCalibration[2], &inputCalibration[3], &inputCalibration[4],
            &inputCalibration[5], &inputCalibration[6], &inputCalibration[7], &inputCalibration[8]
            ) == 9 &&
            std::sscanf(l2.c_str(), "%d %d", &in_width, &in_height) == 2)
        {
            printf("Input resolution: %d %d\n", in_width, in_height);
            printf("In: %f %f %f %f %f %f %f %f %f\n",
                inputCalibration[0], inputCalibration[1], inputCalibration[2], inputCalibration[3], inputCalibration[4],
                inputCalibration[5], inputCalibration[6], inputCalibration[7], inputCalibration[8]);
        }
        else
        {
            printf("Failed to read camera calibration (invalid format?)\nCalibration file: %s\n", configFileName);
            valid = false;
        }

        // l3
        if (l3 == "crop")
        {
            outputCalibration = -1;
            printf("Out: Crop\n");
        }
        else if (l3 == "full")
        {
            outputCalibration = -2;
            printf("Out: Full\n");
        }
        else if (l3 == "none")
        {
            printf("NO RECTIFICATION\n");
            valid = false;
        }
        else
        {
            printf("Out: Failed to Read Output pars... not rectifying.\n");
            valid = false;
        }

        // l4
        if (std::sscanf(l4.c_str(), "%d %d", &out_width, &out_height) == 2)
        {
            printf("Output resolution: %d %d\n", out_width, out_height);
        }
        else
        {
            printf("Out: Failed to Read Output resolution... not rectifying.\n");
            valid = false;
        }

        cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_32F);
        for (int i = 0; i < 4; ++i)
            distCoeffs.at<float>(1 + i, 0) = inputCalibration[4 + i];

        if (inputCalibration[2] < 1.0f)
        {
            printf("WARNING: cx = %f < 1, which should not be the case for normal cameras.!\n", inputCalibration[2]);
            printf("Possibly this is due to a recent change in the calibration file format, please see the README.md.\n");

            inputCalibration[0] *= in_width;
            inputCalibration[2] *= in_width;
            inputCalibration[1] *= in_height;
            inputCalibration[3] *= in_height;

            printf("auto-changing calibration file to fx=%f, fy=%f, cx=%f, cy=%f\n",
                inputCalibration[0],
                inputCalibration[1],
                inputCalibration[2],
                inputCalibration[3]);
        }

        // Set projection parameters
        m_mu = inputCalibration[0];
        m_mv = inputCalibration[1];
        m_u0 = inputCalibration[2];
        m_v0 = inputCalibration[3];
        m_k2 = inputCalibration[4];
        m_k3 = inputCalibration[5];
        m_k4 = inputCalibration[6];
        m_k5 = inputCalibration[7];

        // Construct calibration matrix K
        originalK_ = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
        originalK_.at<double>(0, 0) = m_mu;
        originalK_.at<double>(1, 1) = m_mv;
        originalK_.at<double>(2, 2) = 1;
        //originalK_.at<double>(0, 2) = in_width / 2;
        //originalK_.at<double>(1, 2) = in_height / 2;
        originalK_.at<double>(0, 2) = m_u0;
        originalK_.at<double>(1, 2) = m_v0;

        if (valid)
        {
            // Calculate output intrinsic parameters
            double fx = (m_mu / in_width)*out_width;
            double fy = (m_mv / in_height)*out_height;
            double cx = (m_u0 / in_width)*out_width;
            double cy = (m_v0 / in_height)*out_height;

            // Build new camera matrix
            K_ = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
            K_.at<double>(0, 0) = fx;
            K_.at<double>(1, 1) = fy;
            K_.at<double>(2, 2) = 1;
            K_.at<double>(0, 2) = cx;
            K_.at<double>(1, 2) = cy;

            //distCoeffs = distCoeffs.rowRange(0, 4);//
            //K_ = cv::getOptimalNewCameraMatrix(originalK_, distCoeffs, cv::Size(in_width, in_height), (outputCalibration == -2) ? 1 : 0, cv::Size(out_width, out_height), nullptr, false);

            /*cv::initUndistortRectifyMap(originalK_, distCoeffs, cv::Mat(), K_,
                cv::Size(out_width, out_height), CV_16SC2, map1, map2);*/

            initUndistortRectifyMap(map1, map2, fx, fy, cx, cy, cv::Size(out_width, out_height));

            originalK_.at<double>(0, 0) /= in_width;
            originalK_.at<double>(0, 2) /= in_width;
            originalK_.at<double>(1, 1) /= in_height;
            originalK_.at<double>(1, 2) /= in_height;
        }

        originalK_ = originalK_.t();
        K_ = K_.t();
    }

    UndistorterKannalaBrandt::UndistorterKannalaBrandt(int inputWidth, int inputHeight,
        int outputWidth, int outputHeight,
        float mu, float mv, float u0, float v0,
        float k2, float k3, float k4, float k5)
    {
        // Set frame dimensions
        in_width = inputWidth;
        in_height = inputHeight;
        out_width = outputWidth;
        out_height = outputHeight;

        // Set projection parameters
        m_mu = mu;
        m_mv = mv;
        m_u0 = u0;
        m_v0 = v0;
        m_k2 = k2;
        m_k3 = k3;
        m_k4 = k4;
        m_k5 = k5;

        // Construct calibration matrix K
        originalK_ = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
        originalK_.at<double>(0, 0) = m_mu;
        originalK_.at<double>(1, 1) = m_mv;
        originalK_.at<double>(2, 2) = 1;
        originalK_.at<double>(0, 2) = m_u0;
        originalK_.at<double>(1, 2) = m_v0;

        // Calculate output intrinsic parameters
        double fx = (m_mu / in_width)*out_width;
        double fy = (m_mv / in_height)*out_height;
        double cx = (m_u0 / in_width)*out_width;
        double cy = (m_v0 / in_height)*out_height;

        // Build new camera matrix
        K_ = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
        K_.at<double>(0, 0) = fx;
        K_.at<double>(1, 1) = fy;
        K_.at<double>(2, 2) = 1;
        K_.at<double>(0, 2) = cx;
        K_.at<double>(1, 2) = cy;

        initUndistortRectifyMap(map1, map2, fx, fy, cx, cy, cv::Size(out_width, out_height));

        originalK_.at<double>(0, 0) /= in_width;
        originalK_.at<double>(0, 2) /= in_width;
        originalK_.at<double>(1, 1) /= in_height;
        originalK_.at<double>(1, 2) /= in_height;

        originalK_ = originalK_.t();
        K_ = K_.t();
    }

    UndistorterKannalaBrandt::~UndistorterKannalaBrandt()
    {
    }

    void UndistorterKannalaBrandt::undistort(const cv::Mat& image, cv::OutputArray result) const
    {
        cv::remap(image, result, map1, map2, cv::INTER_LINEAR);
    }

    const cv::Mat& UndistorterKannalaBrandt::getK() const
    {
        return K_;
    }

    const cv::Mat& UndistorterKannalaBrandt::getOriginalK() const
    {
        return originalK_;
    }

    int UndistorterKannalaBrandt::getOutputWidth() const
    {
        return out_width;
    }

    int UndistorterKannalaBrandt::getOutputHeight() const
    {
        return out_height;
    }
    int UndistorterKannalaBrandt::getInputWidth() const
    {
        return in_width;
    }

    int UndistorterKannalaBrandt::getInputHeight() const
    {
        return in_height;
    }

    bool UndistorterKannalaBrandt::isValid() const
    {
        return valid;
    }

    cv::Mat UndistorterKannalaBrandt::initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
        float fx, float fy, float cx, float cy, cv::Size imageSize) const
    {
        cv::Mat mapX = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);
        cv::Mat mapY = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);

        Eigen::Matrix3f K_rect;

        K_rect << fx, 0, cx,
            0, fy, cy,
            0, 0, 1;

        Eigen::Matrix3f K_rect_inv = K_rect.inverse();

        Eigen::Matrix3f R, R_inv;
        cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F);
        cv::cv2eigen(rmat, R);
        R_inv = R.inverse();

        for (int v = 0; v < imageSize.height; ++v)
        {
            for (int u = 0; u < imageSize.width; ++u)
            {
                Eigen::Vector3f xo;
                xo << u, v, 1;

                Eigen::Vector3f uo = R_inv * K_rect_inv * xo;

                Eigen::Vector2d p;
                spaceToPlane(uo.cast<double>(), p);

                mapX.at<float>(v, u) = p(0);
                mapY.at<float>(v, u) = p(1);
            }
        }

        cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);

        cv::Mat K_rect_cv;
        cv::eigen2cv(K_rect, K_rect_cv);
        return K_rect_cv;
    }

    template<typename T>
    T r(T k2, T k3, T k4, T k5, T theta)
    {
        // k1 = 1
        return theta +
            k2 * theta * theta * theta +
            k3 * theta * theta * theta * theta * theta +
            k4 * theta * theta * theta * theta * theta * theta * theta +
            k5 * theta * theta * theta * theta * theta * theta * theta * theta * theta;
    }

    void UndistorterKannalaBrandt::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
    {
        double theta = acos(P(2) / P.norm());
        double phi = atan2(P(1), P(0));

        Eigen::Vector2d p_u = r(m_k2, m_k3, m_k4, m_k5, theta) * Eigen::Vector2d(cos(phi), sin(phi));

        // Apply generalised projection matrix
        p << m_mu * p_u(0) + m_u0,
            m_mv * p_u(1) + m_v0;
    }
}