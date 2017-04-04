#include "rectification/Undistorter.h"
#include "rectification/UndistorterKannalaBrandt.h"

#include <sstream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>

namespace cc
{

	Undistorter::~Undistorter()
	{
	}

	std::shared_ptr<Undistorter> Undistorter::create(const std::string & cfg_file)
	{
		float inputCalibration[10];
		int in_width, in_height, out_width, out_height;

		// read parameters
		std::ifstream infile(cfg_file);
		assert(infile.good());

		std::string l0, l1, l2, l3, l4;

		std::getline(infile, l0);
		std::getline(infile, l1);
		std::getline(infile, l2);
		std::getline(infile, l3);
		std::getline(infile, l4);

		if (l0 == "KANNALA_BRANDT")
		{
			// l1 & l2
			if (std::sscanf(l1.c_str(), "%f %f %f %f %f %f %f %f",
				&inputCalibration[0], &inputCalibration[1], &inputCalibration[2], &inputCalibration[3], &inputCalibration[4],
				&inputCalibration[5], &inputCalibration[6], &inputCalibration[7], &inputCalibration[8]
			) == 8 &&
				std::sscanf(l2.c_str(), "%d %d", &in_width, &in_height) == 2)
			{
				printf("Input resolution: %d %d\n", in_width, in_height);
				printf("In: %f %f %f %f %f %f %f %f %f\n",
					inputCalibration[0], inputCalibration[1], inputCalibration[2], inputCalibration[3], inputCalibration[4],
					inputCalibration[5], inputCalibration[6], inputCalibration[7], inputCalibration[8]);
			}
			else
			{
				printf("Failed to read camera calibration (invalid format?)\nCalibration file: %s\n", cfg_file);
				return nullptr;
			}

			// l4
			if (std::sscanf(l4.c_str(), "%d %d", &out_width, &out_height) == 2)
			{
				printf("Output resolution: %d %d\n", out_width, out_height);
			}
			else
			{
				printf("Out: Failed to Read Output resolution... not rectifying.\n");
				return nullptr;
			}

			// Set projection parameters
			double mu = inputCalibration[0];
			double mv = inputCalibration[1];
			double u0 = inputCalibration[2];
			double v0 = inputCalibration[3];
			double k2 = inputCalibration[4];
			double k3 = inputCalibration[5];
			double k4 = inputCalibration[6];
			double k5 = inputCalibration[7];

			return std::make_shared<UndistorterKannalaBrandt>(in_width, in_height,
				out_width, out_height, mu, mv, u0, v0, k2, k3, k4, k5);
		}
		else return nullptr;
	}

}
