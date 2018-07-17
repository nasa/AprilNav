#include <iostream>
#include "TagOptimization.h"
#include "Coordinates.h"
#include <cmath>
#include <vector>
#include <array>

using namespace std;

namespace AprilTags{
	void TagOptimization::set(double TIME, int TAGID, double x, double y, double PITCH, double ROLL, double YAW , std::vector<std::array<double,2> > COORDS){
	time = TIME; tagID = TAGID; X = x; Y = y; pitch = PITCH; roll = ROLL; yaw = YAW;
	copy(COORDS.begin(),COORDS.end(),back_inserter(coords));
	}
	void TagOptimization::optimize(){
		//Determine QR's coordinates given ID
		QR_X = coords.at(tagID)[0];
		QR_Y = coords.at(tagID)[1];

		//Coordinate System has to be rotated to account for orientation of camera
		X_Rot = X*cos(yaw) - Y*sin(yaw);
		Y_Rot = Y*cos(yaw) + X*sin(yaw);
		CAMERA_X = QR_X - X_Rot;
		CAMERA_Y = QR_Y - Y_Rot;

		//Determine Camera Position in relation to QR Coordinate
		//convert radians to degrees
		pitch = pitch * 180 / M_PI;
		roll = roll * 180 / M_PI;
		yaw = yaw * 180 / M_PI;
	}

	void TagOptimization::print() {
	std::cout << "Camera X: " << CAMERA_X << " Camera Y: " << CAMERA_Y << std::endl;
	}

	double TagOptimization::getCamera_X(){
		return CAMERA_X;
	}
	double TagOptimization::getCamera_Y(){
		return CAMERA_Y;
	}
	double TagOptimization::getPitch(){
		return pitch;
	}
	double TagOptimization::getRoll(){
		return roll;
	}
	double TagOptimization::getYaw(){
		return yaw;
	}
}
