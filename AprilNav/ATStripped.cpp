/*Stripped Version of Apri AprilTags
*/
#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

using namespace std;
using namespace cv;

AprilTags::TagDetector* m_tagDetector;

bool m_draw = true; // draw image and April tag detections?
bool m_arduino; // send tag detections to serial port?
bool m_timing = false; // print timing information for each tag extraction call

int m_width = 640; // image size in pixels
int m_height = 480;
double m_tagSize = .2; // April tag side length in meters of square black frame
double m_fx = 600; // camera focal length in pixels
double m_fy = 600;
double m_px = m_width/2; // camera principal point
double m_py = m_width/2;

int m_deviceId; // camera id (in case of multiple cameras)
list<string> m_imgNames;
cv::VideoCapture m_cap;

#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

inline double standardRad(double t) {
	if (t >= 0.) {
		t = fmod(t + PI, TWOPI) - PI;
	}
	else {
		t = fmod(t - PI, -TWOPI) + PI;
	}
	return t;
}

//Convert rotation matrix to Euler angles
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

void videosetup(){
  //Update to support more tag families
  m_tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);
  m_cap.open(0);

  if(!m_cap.isOpened())  // Check if we succeeded
  {
      cout << "camera is not open" << endl;
  }
  else
  {
      cout << "camera is open" << endl;
  }
}

void print_detection(AprilTags::TagDetection& detection) {
    cout << "  Id: " << detection.id
         << " (Hamming: " << detection.hammingDistance << ")";

    // recovering the relative pose of a tag:

    // NOTE: for this to be accurate, it is necessary to use the
    // actual camera parameters here as well as the actual tag size
    // (m_fx, m_fy, m_px, m_py, m_tagSize)

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                             translation, rotation);

    Eigen::Matrix3d F;
    F <<
      1, 0,  0,
      0,  -1,  0,
      0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

    cout << "  distance=" << translation.norm()
         << "m, x=" << translation(0)
         << ", y=" << translation(1)
         << ", z=" << translation(2)
         << ", yaw=" << yaw
         << ", pitch=" << pitch
         << ", roll=" << roll
         << endl;
  }

void processImage(cv::Mat& image, cv::Mat& image_gray) {
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);

  // print out each detection
  cout << detections.size() << " tags detected:" << endl;
  for (int i=0; i<detections.size(); i++) {
    print_detection(detections[i]);
  }

  // show the current image including any detections
  if (m_draw) {
    for (int i=0; i<detections.size(); i++) {
      // also highlight in the image
      detections[i].draw(image);
    }
    //Crosshair
    line(image, Point((image.cols / 2) - 50, image.rows / 2), Point((image.cols / 2) + 50, image.rows / 2), Scalar(0, 0, 255), 1);
		line(image, Point(image.cols / 2, (image.rows / 2) - 50), Point(image.cols / 2, (image.rows / 2) + 50), Scalar(0, 0, 255), 1);
    imshow("CALIBRATE",image);
  }
}

void loop() {
  cv::Mat image;
  cv::Mat image_gray;

  while (true) {
    m_cap >> image;
    processImage(image, image_gray);
    if (cv::waitKey(1) >= 0) break;
  }
}

// here is were everything begins
int main(int argc, char* argv[]) {
  videosetup();
  loop();
  return 0;
}
