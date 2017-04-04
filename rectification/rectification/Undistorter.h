#ifndef _UNDISTORTER_HPP_
#define _UNDISTORTER_HPP_

#include <memory>
#include <opencv2/core.hpp>

namespace cc
{
	class Undistorter
	{
	public:
		virtual ~Undistorter();

		/**
		 * Undistorts the given image and returns the result image.
		 */
		virtual void undistort(const cv::Mat& image, cv::OutputArray result) const = 0;

		/**
		 * Returns the intrinsic parameter matrix of the undistorted images.
		 */
		virtual const cv::Mat& getK() const = 0;

		/**
		 * Returns the intrinsic parameter matrix of the original images,
		 */
		virtual const cv::Mat& getOriginalK() const = 0;

		/**
		 * Returns the width of the undistorted images in pixels.
		 */
		virtual int getOutputWidth() const = 0;

		/**
		 * Returns the height of the undistorted images in pixels.
		 */
		virtual int getOutputHeight() const = 0;

		/**
		 * Returns the width of the input images in pixels.
		 */
		virtual int getInputWidth() const = 0;

		/**
		 * Returns the height of the input images in pixels.
		 */
		virtual int getInputHeight() const = 0;


		/**
		 * Returns if the undistorter was initialized successfully.
		 */
		virtual bool isValid() const = 0;

		/**
		 * Creates and returns an Undistorter of the type used by the given
		 * configuration file. If the format is not recognized, returns nullptr.
		 */
		static std::shared_ptr<Undistorter> create(const std::string& cfg_file);
	};
}

#endif
