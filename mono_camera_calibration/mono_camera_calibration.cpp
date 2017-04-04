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

// camera_calibration
#include "camera_calibration/Chessboard.h"
#include "camera_calibration/CameraCalibration.h"

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
    string inputPath, outputDir, cfgPath, camModelName, camName;
    unsigned int verbose;
    int device;
    unsigned int width, height;
    double fps, squareSize;
    unsigned int boardWidthCorners, boardHeightCorners;
    unsigned int snap_total;
    double snap_delay, snap_translation;

    cc::Camera::ModelType camModel;
    try {
        options_description desc("Allowed options");
        desc.add_options()
            ("help", "display the help message")
            ("verbose,v", value<unsigned int>(&verbose)->default_value(1), "output debug information")
            ("input,i", value<string>(&inputPath), "input path")
            ("output,o", value<string>(&outputDir), "output directory")
            ("device,d", value<int>(&device)->default_value(-1), "device id")
            ("width,w", value<unsigned int>(&width)->default_value(0), "frame width")
            ("height,h", value<unsigned int>(&height)->default_value(0), "frame height")
            ("fps,f", value<double>(&fps)->default_value(30.0), "frames per second")
            ("model,m", value<string>(&camModelName)->default_value("KANNALA_BRANDT"), "camera model [KANNALA_BRANDT|MEI|PINHOLE]")
            ("square_size,s", value<double>(&squareSize)->default_value(30.0), "board square size [MM]")
            ("width_corners", value<unsigned int>(&boardWidthCorners)->default_value(7), "interior number of corners along the board width")
            ("height_corners", value<unsigned int>(&boardHeightCorners)->default_value(7), "interior number of corners along the board height")
            ("camera_name,c", value<string>(&camName)->default_value("unknown_camera"), "camera name")
            ("snap_total", value<unsigned int>(&snap_total)->default_value(50), "total number of snapshots for calibration")
            ("snap_delay", value<double>(&snap_delay)->default_value(0.5), "minimum time between snapshots [s]")
            ("snap_translation", value<double>(&snap_translation)->default_value(20.0), "minimum translation between snapshots [pixels]")
            ("cfg", value<string>(&cfgPath)->default_value("mono_camera_calibration.cfg"), "configuration file (.cfg)")
            ;
        variables_map vm;
        store(command_line_parser(argc, argv).options(desc).
            positional(positional_options_description().add("input", -1)).run(), vm);

        if (vm.count("help")) {
            cout << "Usage: mono_camera_calibration [options]" << endl;
            cout << desc << endl;
            exit(0);
        }

        // Read config file
        ifstream ifs(vm["cfg"].as<string>());
        store(parse_config_file(ifs, desc), vm);

        notify(vm);

        if (camModelName == "KANNALA_BRANDT")
            camModel = cc::Camera::KANNALA_BRANDT;
        else if (camModelName == "MEI")
            camModel = cc::Camera::MEI;
        else if (camModelName == "PINHOLE")
            camModel = cc::Camera::PINHOLE;
        else throw error("Invalid camera model!");

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

        // Initialize calibration tool
        cc::CameraCalibration calibration(camModel, camName, cv::Size(width, height),
            cv::Size(boardWidthCorners, boardHeightCorners), squareSize / 1000.0f);
        calibration.setVerbose(verbose > 0);

        // Initialize frame
        cv::Mat frame, frameGray, view;

        // Check for calibration data
        path dat = path(outputDir) / (camName + "_chessboard_data.dat");
        bool collectBoardData = true;
        if (exists(dat))
        {
            cout << "The chessboard data file " << dat << " already exists." << endl;
            char type;
            do
            {
                cout << "Use existing chessboard data? [y/n]" << endl;
                std::cin >> type;
            } while (!std::cin.fail() && type != 'y' && type != 'n');
            collectBoardData = (type == 'n');
        }

        //if (collectBoardData && !cap.isOpened())
        if (collectBoardData && vs == nullptr)
            throw runtime_error("Failed to open video capture!");

        cpu_timer timer;
        
        if (collectBoardData)
        {
            cout << "Collecting board data..." << endl;

            // Preview loop
            while (vs->read())
            {
                frame = vs->getFrameGrayscale();

                cv::imshow("view", frame);
                int key = cv::waitKey(1);
                if (key >= 0)
                    break;
            }

            // Main loop
            double timestamp = 0, lastFrameTimestamp = 0;
            cv::Point2f lastFirstCorner = cv::Point2f(std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max());

            timer.start();
            while (calibration.sampleCount() < snap_total)
            {
                if (!vs->read())
                //if (!cap.read(frame))
                    throw runtime_error("Failed to read video frame!");

                frame = vs->getFrameGrayscale();

                // Get frame time
                timestamp = timer.elapsed().wall*1e-9;
                //timestamp = cap.get(CV_CAP_PROP_POS_MSEC)*1e-3;

                // Convert to grayscale
                //cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);

                cc::Chessboard chessboard(cv::Size(boardWidthCorners, boardHeightCorners), frame);
                chessboard.findCorners();

                if (chessboard.cornersFound())
                {
                    chessboard.getSketch().copyTo(view);
                }
                else
                {
                    frame.copyTo(view);
                }

                // Update corner data
                if (chessboard.cornersFound() &&
                    (timestamp - lastFrameTimestamp) > snap_delay &&
                    cv::norm(cv::Mat(lastFirstCorner - chessboard.getCorners()[0])) > snap_translation)
                {
                    lastFirstCorner = chessboard.getCorners()[0];
                    lastFrameTimestamp = timestamp;

                    cv::bitwise_not(view, view);
                    calibration.addChessboardData(chessboard.getCorners());
                }

                // Show frame
                string msg = (boost::format("%d / %d") % calibration.sampleCount() % snap_total).str();
                cv::putText(view, msg, cv::Point(10, 20),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                    1, CV_AA);

                cv::imshow("mono_camera_calibration", view);
                cv::waitKey(1);
            }
        }
        else // Read existing calibration data from file
        {
            calibration.readChessboardData(dat.string());
        }

        ///////////////////////////////////////////////////////////////////////////////////////////
        // Calibration
        ///////////////////////////////////////////////////////////////////////////////////////////
        cout << "Calibrating..." << endl;
        timer.start();

        calibration.calibrate();

        cout << "Calibration took a total time of " << timer.elapsed().wall*1e-9 << " sec." << endl;

        ///////////////////////////////////////////////////////////////////////////////////////////
        // Output
        ///////////////////////////////////////////////////////////////////////////////////////////

        // Make sure output directory exists
        create_directory(outputDir);

        path yaml = path(outputDir) / (camName + "_camera_calib.yaml");
        path calibCfg = path(outputDir) / (camName + "_camera_calib.cfg");
        calibration.writeParameters(yaml.string());
        cout << "Wrote calibration yaml file to " << yaml.string() << endl;
        calibration.writeParametersToCfgFile(calibCfg.string());
        cout << "Wrote calibration cfg file to " << calibCfg.string() << endl;
        if (collectBoardData)
        {
            calibration.writeChessboardData(dat.string());
            cout << "Wrote chessboard data file to " << dat.string() << endl;
        }
                  
        //cout << "Wrote calibration file to " << yaml.string() << endl;               

        ///////////////////////////////////////////////////////////////////////////////////////////
        // Undistortion
        ///////////////////////////////////////////////////////////////////////////////////////////
        cv::Mat mapX, mapY;
        cc::CameraPtr& camera = calibration.camera();

        if (camera->modelType() == cc::Camera::PINHOLE)
        {
            camera->initUndistortRectifyMap(mapX, mapY);
        }
        else
        {
            /*
            camera->initUndistortRectifyMap(mapX, mapY, 300.0f, 300.0f,
                cv::Size(camera->imageWidth(),
                camera->imageHeight()),
                -1.0f, -1.0f);
             */ 

            vector<double> params;
            camera->writeParameters(params);

            
            camera->initUndistortRectifyMap(mapX, mapY, params[4], params[5],
                cv::Size(camera->imageWidth(),
                camera->imageHeight()),
                params[6], params[7]);
            
            /*
            int w = 640;
            int h = 480;
            float fx = (params[4] / camera->imageWidth())*w;
            float fy = (params[5] / camera->imageHeight())*h;
            float cx = (params[6] / camera->imageWidth())*w;
            float cy = (params[7] / camera->imageHeight())*h;

            camera->initUndistortRectifyMap(mapX, mapY, fx, fy,
                cv::Size(w, h),
                cx, cy);
                */
        }

        while (vs->read())
        {
            frame = vs->getFrameGrayscale();

            // Convert to grayscale
            //cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);

            cv::remap(frame, view, mapX, mapY, cv::INTER_LINEAR);

            // Show frame
            cv::putText(view, "Calibrated", cv::Point(10, 20),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                1, CV_AA);

            cv::imshow("mono_camera_calibration", view);
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