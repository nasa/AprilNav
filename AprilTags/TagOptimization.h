#include<stdio.h>
#include <string>
#include <vector>
#include <array>

namespace AprilTags{
  class TagOptimization {
  public:
    void set(double TIME, int TAGID, double x, double y, double PITCH, double ROLL, double YAW, std::vector <std::array<double,2>> COORDS);
    void optimize();
    double getCamera_X();
    double getCamera_Y();
    double getPitch();
    double getRoll();
    double getYaw();
    void print();
  private:
    std::vector <std::array<double,2>> coords;
    int count;
    double time;
    int tagID;
    double X, Y, Z; //Meters
    double pitch, roll, yaw; //Radians
    double QR_X, QR_Y; // Meters
    double QR_Z = 8.1; //Meters
    double CAMERA_X, CAMERA_Y, CAMERA_Z; //Meters
    double X_Rot, Y_Rot;
  };
}
