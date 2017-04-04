# Camera Calibration
Created by Yuval Nirkin.

[nirkin.com](http://www.nirkin.com/)

## Overview
This project provides a utility for monocular camera calibration from camera feeds, videos and image sequences, using 3 different camera models. The calibration process is completely automatic. The user is only required to capture a board with square patterns using spherical movements around it.

## Dependencies
| Library                                                            | Minimum Version | Notes                                    |
|--------------------------------------------------------------------|-----------------|------------------------------------------|
| [Boost](http://www.boost.org/)                                     | 1.47            |                                          |
| [OpenCV](http://opencv.org/)                                       | 3.0             |                                          |
| [vsal](https://github.com/YuvalNirkin/vsal)                        | 1.0             |                                          |
| [Ceres Solver](https://github.com/ceres-solver/ceres-solver)       | 1.10.0          |                                          |

## Installation
- Use CMake and your favorite compiler to build and install the library.
- Add camera_calibration/bin to path.

## Usage
- Use mono_camera_calibration to calibrate your camera. This will output "camera_name_calib.cfg".
- The rectify example shows how to use "camera_name_calib.cfg" for rectifying your camera video stream.
