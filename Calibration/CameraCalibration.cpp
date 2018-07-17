// Code from: http://aishack.in/tutorials/calibrating-undistorting-opencv-oh-yeah/


#include <sstream>
#include <stdio.h>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <string>
using namespace cv;
using namespace std;


int main() {
    //create variables
    int numBoards = 0;
    int numCornersHor;
    int numCornersVer;

    //user input
    printf("Enter number of corners along width: ");
    scanf("%d", &numCornersHor);

    printf("Enter number of corners along height: ");
    scanf("%d", &numCornersVer);

    printf("Enter number of boards: ");
    scanf("%d", &numBoards);

    //more variables
    int numSquares = numCornersHor * numCornersVer;
    Size board_sz = Size(numCornersHor, numCornersVer);

    //open webcam
    VideoCapture capture = VideoCapture(0);

    //create object and image points
    vector<vector<Point3f>> object_points; //list of 3D points, object_points is location of corners in 3D space(measured by user)
    vector<vector<Point2f>> image_points;  //list of 2D points, 2D location of corners

    //list of corners(holds current snapshot of chessboard's corners)
    vector<Point2f> corners;
    int successes=0;

    //two images and first snapshot
    Mat image;
    Mat gray_image;
    capture >> image;

    //assign a constant position to each vertex(assume board is at (0,0,0) and camera is moving)
    //creates a list of coordinates
    vector<Point3f> obj;
    for(int j=0;j<numSquares;j++)
        obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

    //loop
    while(successes<numBoards)
    {
        cvtColor(image, gray_image, CV_BGR2GRAY); //convert to grayscale
        bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if(found)
        {
            cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray_image, board_sz, corners, found);
        }

        //update display and grab another frame
        imshow("win1", image);
        imshow("win2", gray_image);
        capture >> image;
        int key = waitKey(1);

        //if escape is pressed, program stops
        //if space bar is pressed, stores data to lists
        if(key==27)
            return 0;

        if(key==' ' && found!=0)
        {
            image_points.push_back(corners);
            object_points.push_back(obj);
            printf("Snap stored!");
            successes++;
            if(successes>=numBoards)
                break;
        }
    }

    //calibration, declare variables
    Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;

    //camera's aspect ratio is 1
    //Elements (0,0) and (1,1) are the focal lengths along the X and Y axis.
    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;

    //calibration:
    calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
    //now we have the intrinsic matrix, distortion coefficients and the rotation+translation vectors
    cout << distCoeffs;
    cout << intrinsic; //camera matrix
    //undistort images
    Mat imageUndistorted;
    while(1)
    {
        capture >> image;
        undistort(image, imageUndistorted, intrinsic, distCoeffs);

        imshow("win1", image);
        imshow("win2", imageUndistorted);
        waitKey(1);
    }

    capture.release();

    return 0;
}
