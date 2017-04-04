#ifndef __CONFIG__
#define __CONFIG__

/************************************************************************************
*									   Includes										*
************************************************************************************/
#include <string>
#include "camera_models/Camera.h"

/************************************************************************************
*									 Declarations									*
************************************************************************************/


/************************************************************************************
*										Classes										*
************************************************************************************/

class Config
{
public:
	enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

	Config();

	Config(const std::string& configPath);

	void readFromFile(const std::string& configPath);

	bool isValid() const;

	unsigned int boardWidthItems;	// Interior number of corners along the width of the board
	unsigned int boardHeightItems;	// Interior number of corners along the height of the board
	double squareSize;
	Pattern pattern;
	int flags;
	unsigned int frameStep;			// The step size from image to image in the dataset
	bool fixAspectRatio;
    bool fixPrincipalPoint;
    px::Camera::ModelType modelType;
    std::string cameraName;
};

#endif	// __CONFIG__