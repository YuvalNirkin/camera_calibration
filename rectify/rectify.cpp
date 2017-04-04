//#include "Config.h"

// std
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>

// Boost
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/timer/timer.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

// vsal
#include <vsal/VideoStreamFactory.h>
#include <vsal/VideoStreamOpenCV.h>

// rectification
#include <rectification/Undistorter.h>

// Namespaces
using std::cout;
using std::endl;
using std::cerr;
using std::ofstream;
using std::string;
using std::vector;
using std::runtime_error;
using namespace boost::filesystem;
using namespace boost::program_options;
using boost::timer::cpu_timer;

int main(int argc, char** argv)
{
    // Parse command line arguments
    string inputPath, cfgPath, calibPath;
    int device;
    unsigned int width, height;
    double fps;

    try {
        options_description desc("Allowed options");
        desc.add_options()
            ("help", "display the help message")
            ("input,i", value<string>(&inputPath), "input path")
            ("device,d", value<int>(&device)->default_value(-1), "device id")
            ("width,w", value<unsigned int>(&width)->default_value(0), "frame width")
            ("height,h", value<unsigned int>(&height)->default_value(0), "frame height")
            ("fps,f", value<double>(&fps)->default_value(30.0), "frames per second")
			("calib,c", value<string>(&calibPath)->required(), "calibration file (.cfg)")
            ("cfg", value<string>(&cfgPath)->default_value("rectify.cfg"), "configuration file (.cfg)")
            ;
        variables_map vm;
        store(command_line_parser(argc, argv).options(desc).
            positional(positional_options_description().add("input", -1)).run(), vm);

        if (vm.count("help")) {
            cout << "Usage: rectify [options]" << endl;
            cout << desc << endl;
            exit(0);
        }

        // Read config file
        ifstream ifs(vm["cfg"].as<string>());
        store(parse_config_file(ifs, desc), vm);

        notify(vm);

		if (!is_regular_file(calibPath)) throw error("calib must be a path to a calibration file!");
    }
    catch (const error& e) {
        cout << "Error while parsing command-line arguments: " << e.what() << endl;
        cout << "Use --help to display a list of options." << endl;
        exit(0);
    }

    try
    {
        cout << "Initializing..." << endl;

        // Create video source
        bool live = device >= 0;
        vsal::VideoStreamFactory& vsf = vsal::VideoStreamFactory::getInstance();
        vsal::VideoStreamOpenCV* vs = nullptr;
        if (live) vs = (vsal::VideoStreamOpenCV*)vsf.create(device, width, height);
        else if (!inputPath.empty()) vs = (vsal::VideoStreamOpenCV*)vsf.create(inputPath);
        else throw runtime_error("No video source specified!");

        // Open video source
        if (!vs->open()) throw runtime_error("Failed to open video source!");
        width = vs->getWidth();
        height = vs->getHeight();
        fps = vs->getFPS();

		// Initialize rectification
		std::shared_ptr<cc::Undistorter> undistorter = cc::Undistorter::create(calibPath);
		cv::Mat K = undistorter->getK();
		std::cout << "K = " << K << std::endl;

        // Initialize frame
        cv::Mat frame, frameGray, view;
        while (vs->read())
        {
            frame = vs->getFrame();

			// Undistort image
			cv::Mat frameUndist;
			undistorter->undistort(frame, frameUndist);

            // Show frame
            cv::putText(frameUndist, "Calibrated", cv::Point(10, 20),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                1, CV_AA);

            cv::imshow("rectify", frameUndist);
            int key = cv::waitKey(1);
            if (key >= 0)
                break;
        }

        // Cleanup
        delete vs;
    }
    catch (std::exception& e)
    {
        cerr << e.what() << endl;
        return 1;
    }

    return 0;
}