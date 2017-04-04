/************************************************************************************
*									   Includes										*
************************************************************************************/
#include "Config.h"
#include <iostream>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

/************************************************************************************
*									  Namespaces									*
************************************************************************************/
using std::cout;
using std::endl;
using std::string;
using namespace boost::filesystem;

/************************************************************************************
*									 Declarations									*
************************************************************************************/
#define EPS 1.0e-6

/************************************************************************************
*									Implementation									*
************************************************************************************/

Config::Config() :
	boardWidthItems(0), boardHeightItems(0),
	squareSize(0),
	pattern(NOT_EXISTING),
	flags(0),
	fixAspectRatio(false),
    fixPrincipalPoint(false),
	frameStep(1),
    modelType(px::Camera::ModelType::KANNALA_BRANDT),
    cameraName("unknown_camera")
{
}

Config::Config(const string& configPath) :
boardWidthItems(0), boardHeightItems(0),
squareSize(0),
pattern(NOT_EXISTING),
flags(0),
frameStep(1)
{
	readFromFile(configPath);
}

void Config::readFromFile(const std::string& configPath)
{
	if (!exists(configPath)) return;

	// Read config file
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(configPath, pt);

	// Parse data
	boardWidthItems = pt.get<unsigned int>("General.boardWidthItems", boardWidthItems);
	boardHeightItems = pt.get<unsigned int>("General.boardHeightItems", boardHeightItems);
	squareSize = pt.get<double>("General.squareSize", squareSize);

    // Calibration board pattern
	string sPattern;
	sPattern = pt.get<string>("General.pattern", sPattern);
	if (sPattern == "CHESSBOARD") pattern = CHESSBOARD;
	else if (sPattern == "CIRCLES_GRID") pattern = CIRCLES_GRID;
	else if (sPattern == "ASYMMETRIC_CIRCLES_GRID") pattern = ASYMMETRIC_CIRCLES_GRID;

	fixAspectRatio = pt.get<bool>("General.fixAspectRatio", fixAspectRatio);
    fixPrincipalPoint = pt.get<bool>("General.fixPrincipalPoint", fixPrincipalPoint);

	flags = 0;
	if (fixAspectRatio) flags |= cv::CALIB_FIX_ASPECT_RATIO;
    if (fixPrincipalPoint) flags |= cv::CALIB_FIX_PRINCIPAL_POINT;

	frameStep = pt.get<unsigned int>("General.frameStep", frameStep);

    // Camera model
    string sModelType = "KANNALA_BRANDT";
    sModelType = pt.get<string>("General.modelType", sModelType);
    if (sModelType == "KANNALA_BRANDT") modelType = px::Camera::ModelType::KANNALA_BRANDT;
    else if (sModelType == "MEI") modelType = px::Camera::ModelType::MEI;
    else if (sModelType == "PINHOLE") modelType = px::Camera::ModelType::PINHOLE;

    cameraName = pt.get<string>("General.cameraName", cameraName);
}

bool Config::isValid() const
{
	return (pattern != NOT_EXISTING && boardWidthItems != 0 && boardHeightItems != 0
		&& squareSize > EPS && frameStep >= 1);
}