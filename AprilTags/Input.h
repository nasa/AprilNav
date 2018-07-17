#include<stdio.h>
#include <string>
#include <vector>
#include <array>
using namespace std;

namespace AprilTags{
  class Input {
  public:
    void set(double TIME, int TAGID, double x, double y, double PITCH, double ROLL, double YAW);
    void inputParse(std::string str);
    void tail(int n);
    void setup(FILE* fp, vector<array<double,2> > COORDS);
    void verifyTag(int id);
    double getX();
    double getY();
  private:
    FILE* fp;
    int count;
    char oldwp[255];
    std::vector<double> vect;
    double WPX = NULL;
    double WPY = NULL;
    vector <array<double,2> > coords;
  };
}
